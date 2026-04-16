#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Soaring/ExtendedKalmanFilter.h>

// Sparse Gaussian Process (SGP) Config
#define SGP_MAX_OBSERVATIONS 200
#define SGP_NUM_INDUCING_POINTS 77 // 9x9 minus 4 corners
#define SGP_GRID_RES_M 10.0f
#define SGP_FORWARD_BIAS_M 0.0f

#define GRID_HALF_EXTENT_M (4.0f * SGP_GRID_RES_M)
#define OBS_GATE_MARGIN_M 5.0f
//#define SHAPE_SAMPLE_DIST_M 10.0f // radius from centre of updraft that estimator probes for shape
#define SGP_VETO_OBS_SNAPSHOT 40 

// Memory Config
#define MAX_UPDRAFT_MEM 20

struct UpdraftType {
    uint8_t id;
    bool advects;
    uint32_t decay_time_us;
    float k_stretch_parallel; // Stretch along the wind axis
    float k_stretch_perp;     // Stretch across the wind axis

    bool operator==(const UpdraftType& other) const { return id == other.id; }
    bool operator!=(const UpdraftType& other) const { return id != other.id; }
};

namespace UpdraftTypes {
    constexpr UpdraftType UNKNOWN         = {0, false, 180000000UL, 1.0f, 0.0f}; 
    
    // THERMALS: Stretches moderately along the wind due to leaning
    constexpr UpdraftType THERMAL_BUBBLE  = {1, true,  180000000UL, 1.5f, 0.0f}; 
    constexpr UpdraftType THERMAL_CHIMNEY = {2, false,  900000000UL, 2.0f, 0.0f}; 
    constexpr UpdraftType THERMAL_COMPLEX = {6, true,  180000000UL, 1.5f, 0.5f};
    
    // OROGRAPHIC/RIDGE: Terrain dictated
    constexpr UpdraftType OROGRAPHIC      = {3, false, UINT32_MAX,  0.0f, 0.0f}; 
    constexpr UpdraftType RIDGE           = {5, false, UINT32_MAX,  0.0f, 0.1f}; 
    
    // SHEAR: Convergence/Shear lines align with the wind. 
    constexpr UpdraftType SHEAR           = {4, true,  180000000UL, 4.0f, 0.0f}; 
    

}

struct UpdraftObservation {
    Vector3f local_pos; // NED (m)
    float wz;           // airmass vertical velocity (m/s)
    uint32_t time_us;
};

struct UpdraftLKF {
    // State Vector
    float x_W0, x_Ru, x_Rv, x_N, x_E;
    
    // Covariance Vector
    float p_W0, p_Ru, p_Rv, p_N, p_E;
    
    // Process Noise
    float q_W0 = 0.05f;  // Strength variation
    float q_Ru = 0.5f;   // Shape variation
    float q_Rv = 0.5f;
    float q_N  = 2.0f;   // Random drift variance (m/s)
    float q_E  = 2.0f;

    // Measurement Noise
    float r_W0 = 0.05f;   // SGP strength confidence
    float r_Ru = 25.0f;  // SGP shape boundary confidence
    float r_Rv = 25.0f;
    float r_N  = 100.0f; // SGP peak localization confidence
    float r_E  = 100.0f;
    
    void set_noise_params(float qw0, float qrad, float qpos, float rw0, float rrad, float rpos) {
        q_W0 = qw0; q_Ru = qrad; q_Rv = qrad; q_N = qpos; q_E = qpos;
        r_W0 = rw0; r_Ru = rrad; r_Rv = rrad; r_N = rpos; r_E = rpos;
    }

    void init(float z_W0p, float z_Rup, float z_Rvp, float z_Np, float z_Ep) {
        x_W0 = z_W0p;
        x_Ru = z_Rup;
        x_Rv = z_Rvp;
        x_N = z_Np;
        x_E = z_Ep;
        // Initial high uncertainty
        p_W0 = 1.0f; p_Ru = 200.0f; p_Rv = 200.0f; p_N = 100.0f; p_E = 100.0f;
    }

    void predict(float dt_s, float wind_n, float wind_e, float k_stretch_p, float k_stretch_c) {
        // Advection
        x_N += wind_n * dt_s;
        x_E += wind_e * dt_s;

        // Wind Stretching
        float wind_speed = sqrtf(sq(wind_n) + sq(wind_e));
        
        float target_Ru = x_Ru + (k_stretch_p * wind_speed);
        float target_Rv = x_Rv + (k_stretch_c * wind_speed);

        // Morph towards the target shape at a rate of 10% per second
        float morph_rate = 0.1f * dt_s; 
        x_Ru += (target_Ru - x_Ru) * morph_rate;
        x_Rv += (target_Rv - x_Rv) * morph_rate;

        // Covariance
        p_W0 += q_W0 * dt_s;
        p_Ru += q_Ru * dt_s;
        p_Rv += q_Rv * dt_s;
        p_N  += q_N  * dt_s;
        p_E  += q_E  * dt_s;
    }

    void update_strength_pos(float z_W0, float z_N, float z_E) {
        float k_W0 = p_W0 / (p_W0 + r_W0);
        x_W0 += k_W0 * (z_W0 - x_W0);
        p_W0 *= (1.0f - k_W0);

        float k_N = p_N / (p_N + r_N);
        x_N += k_N * (z_N - x_N);
        p_N *= (1.0f - k_N);

        float k_E = p_E / (p_E + r_E);
        x_E += k_E * (z_E - x_E);
        p_E *= (1.0f - k_E);
    }

    void update_radii(float z_Ru, float z_Rv) {
        float k_Ru = p_Ru / (p_Ru + r_Ru);
        x_Ru += k_Ru * (z_Ru - x_Ru);
        p_Ru *= (1.0f - k_Ru);

        float k_Rv = p_Rv / (p_Rv + r_Rv);
        x_Rv += k_Rv * (z_Rv - x_Rv);
        p_Rv *= (1.0f - k_Rv);
    }
};

struct UpdraftObject {
    UpdraftLKF kf;
    UpdraftType type = UpdraftTypes::UNKNOWN;
    
    uint32_t last_update_us = 0;
    uint32_t last_predict_us = 0;
    uint32_t created_us = 0;
    bool active = false;
    bool shape_gate_open = false;
    bool updated_this_tick = false;
    
    Vector2f start_pos;
    float strength_w0() const { return kf.x_W0; }
    float pos_north()   const { return kf.x_N; }
    float pos_east()    const { return kf.x_E; }
    float radius_u()    const { return kf.x_Ru; }
    float radius_v()    const { return kf.x_Rv; }
    float axis_heading = 0.0f;  // Radians (0 = North, East is positive)
    
    // Calculates where updraft should currently be based on its advection.
    Vector2f expected_pos(uint32_t now_us, const Vector2f& wind_vel) const {
        Vector2f pos(kf.x_N, kf.x_E);
        if (type.advects) {
            float dt_s = MAX((now_us - last_predict_us) * 1.0e-6f, 0.0f);
            pos.x += wind_vel.x * dt_s;
            pos.y += wind_vel.y * dt_s;
        }
        return pos;
    }
};

struct UpdraftFeatures {
    Vector3f peak_pos;
    float max_lift;
    float core_variance;  // Turbulence indicator
    float length_major;   // lambda_1 (from Eigenvalue)
    float length_minor;   // lambda_2 (from Eigenvalue)
    float axis_heading;   // from Eigenvector
    uint8_t cluster_support; // minimum required points to count
};

class UpdraftEstimator {
public:
    UpdraftEstimator();
    
    static const struct AP_Param::GroupInfo var_info[];

    void init(bool start_thread = true);

    void push_observation(const Location &loc, float alt_m, float wz, uint32_t time_us);
    
    float get_lift_prediction(const Location &pred_loc, float alt_m, uint32_t now_us);
    
    float get_variance(const Location &pred_loc, float alt_m);
    
    bool is_initialised() const { return initialised; }
    
    void trigger_update(const Location &loc, float alt_m, const Vector2f &ground_vel, const Vector2f &wind_vel);

    bool get_best_global_updraft(const Location &current_loc, Location &target_loc, float &strength, uint32_t now_us);
    
    const UpdraftObject& get_catalog_entry(uint8_t index) const {
        if (index >= MAX_UPDRAFT_MEM) {
            return _catalogue[0];
        }
        return _catalogue[index];
    }
    
    void update_estimate(uint32_t now_us);
    
    // logging
    void log_ue_sgp(uint32_t now_us);
    void log_ue_cat(uint32_t now_us);
    void set_logging_enabled(bool enable) { _log_enable = enable; }
    
    void log_updraft_to_csv(const Location &plane_loc,
                      float plane_alt_m,
                      float plane_heading_deg,
                      float wz,
                      uint32_t time_ms);

private:

    // AP Parameters
    AP_Float obs_noise_var;
    AP_Float obs_tau_decay;
    AP_Float kern_length_xy;
    AP_Float kern_length_z;
    AP_Float kern_variance;
    AP_Float bl_wind_factor;
    
    AP_Float lkf_q_w0;
    AP_Float lkf_q_rad;
    AP_Float lkf_q_pos;
    AP_Float lkf_r_w0;
    AP_Float lkf_r_rad;
    AP_Float lkf_r_pos;

    AP_Float veto_age_max;
    AP_Float veto_radius;

    AP_Float lift_min;
    AP_Float peak_fraction;
    AP_Float noise_fraction;
    
    AP_Float spawn_var_frac;
    AP_Float nms_suppress_rad;

    // Compile time constants
    static constexpr uint8_t M = SGP_NUM_INDUCING_POINTS;
    static constexpr uint16_t kuu_size = M * M;
    static constexpr uint8_t GRID_CENTER_IDX = 38; // for 77 points
    static constexpr uint8_t SGP_MIN_CLUSTER_POINTS = 3; // nodes above lift_min required to spawn catalogue entry
    
    float sigma_n_sq;
    float tau_decay;
    
    // Threading 
    HAL_Semaphore _data_sem;    // Protect shared memory
    bool _update_requested = false;

    void update_thread(); 
    bool initialised = false;
    
    Location _local_origin;
    bool _origin_set = false;

    struct SharedState {
        UpdraftObservation obs_buffer[SGP_MAX_OBSERVATIONS];
        uint16_t obs_head = 0;
        uint16_t obs_count = 0;
        
        Vector3f inducing_points[SGP_NUM_INDUCING_POINTS]; // The grid
        float alpha[SGP_NUM_INDUCING_POINTS];              // The solved weights
                
        Location loc;
        float alt_m;
        Vector2f ground_vel;
        Vector2f wind_vel;
    }; 
    
    SharedState _shared_state{};
    SharedState _local_state{}; // local copy of shared state to work on

    UpdraftObject _catalogue[MAX_UPDRAFT_MEM];
    
    UpdraftObservation _obs_snapshot[SGP_MAX_OBSERVATIONS];
    UpdraftObject _catalogue_snapshot[MAX_UPDRAFT_MEM];
    
    float* _Kuu_prior = nullptr;
    bool _prior_ready = false;

    float* _Kuu_matrix = nullptr;
    float* L_kuu_local = nullptr;

    // Cholesky factors
    float* _L_A_shared = nullptr;    // size M*M, lower-triangular stored in full matrix buffer
    float* _L_Kuu_shared = nullptr;  // size M*M, lower-triangular    
    bool _matrices_valid = false;
    
    float alpha_local[SGP_NUM_INDUCING_POINTS];
    Vector3f ip_local[SGP_NUM_INDUCING_POINTS];
    UpdraftObservation obs_local[SGP_VETO_OBS_SNAPSHOT];
    UpdraftObject catalog_local[MAX_UPDRAFT_MEM];
        
    // Sparse Gaussian Process Functions
    void setup_grid_geometry();
    
    bool calculate_discrete_snap(const Vector2f &glider_pos, const Vector2f &ground_vel, float current_alt);
    
    float matern_kernel(const Vector3f &p1, const Vector3f &p2) const;
    
    void build_prior_matrix(float* kuu_matrix, const float* kuu_prior, const Vector3f* inducing_points, bool prior_ready);
    
    void accumulate_posterior(float* kuu_matrix, float* p_vector, const UpdraftObservation* obs, uint16_t obs_count, uint32_t now_us, const Vector2f& wind_vel, const Vector3f* inducing_points, const UpdraftObject* catalogue);
    
    void extract_updraft_features(const uint8_t* cluster_indices, uint8_t cluster_size, const float* alpha, const Vector3f* inducing_points, UpdraftFeatures& out_features) const;
    void process_updrafts(UpdraftObject* catalogue, const float* new_alpha, const Vector3f* inducing_points, uint32_t now_us, const Vector2f& wind_vel, float current_alt);
    void classify_updraft(UpdraftObject& updraft, const UpdraftFeatures& features, uint32_t now_us, const Vector2f& wind_vel) const;
    void merge_overlapping_updrafts(UpdraftObject* catalogue);
    
    inline float query_sgp_lift(const Vector3f& pos, const float* alpha, const Vector3f* inducing_points) const {
        float lift = 0.0f;
        for (uint8_t j = 0; j < M; j++) {
            lift += matern_kernel(pos, inducing_points[j]) * alpha[j];
        }
        return lift;
    }

    void update_catalogue(UpdraftObject* catalogue, const UpdraftFeatures& features, uint32_t now_us, const Vector2f& wind_vel, float current_alt, const Vector3f* inducing_points);
    
    void prune_inactive(UpdraftObject* catalogue, const float* new_alpha, const Vector3f* inducing_points, float current_alt);
    
   
    
    // matrix maths
    bool cholesky_decompose(float* matrix, uint8_t size);
    void cholesky_solve(const float* L_matrix, uint8_t size, const float* in_vector, float* out_vector);
    
    //logging
    bool _log_enable = false;
    uint32_t _last_sgp_log_us = 0;
    uint32_t _last_cat_log_us = 0;
    uint32_t _last_csv_log_us = 0;
};