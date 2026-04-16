#include "UpdraftEstimator.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo UpdraftEstimator::var_info[] = {
    // @Param: OBS_NOISE
    // @DisplayName: Observation Noise Variance
    // @Description: Expected noise in the vertical wind estimates. Higher values smooth the SGP data but cause lag.
    // @Range: 0.1 2.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OB_VAR", 1, UpdraftEstimator, obs_noise_var, 0.5f),

    // @Param: OBS_DECAY
    // @DisplayName: Observation Temporal Decay
    // @Description: Time in seconds for how quickly old observations are phased out.
    // @Units: s
    // @Range: 60 600
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("OB_DEC", 2, UpdraftEstimator, obs_tau_decay, 300.0f),
    
    // @Param: KERN_L_XY
    // @DisplayName: SGP Horiz Length Scale
    // @Description: Expected horizontal radius of air or updrafts with similar behaviour.
    // @Units: m
    // @Range: 5.0 50.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("KRN_LXY", 3, UpdraftEstimator, kern_length_xy, 10.0f),

    // @Param: KERN_L_Z
    // @DisplayName: SGP Vertical Length Scale
    // @Description: Expected vertical distance of air or updrafts with similar behaviour.
    // @Units: m
    // @Range: 10.0 100.0
    // @Increment: 5.0
    // @User: Advanced
    AP_GROUPINFO("KRN_LZ", 4, UpdraftEstimator, kern_length_z, 45.0f),

    // @Param: KERN_VAR
    // @DisplayName: SGP Updraft Variance
    // @Description: The maximum expected strength of updrafts. 
    // @Range: 1.0 20.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("KRN_VAR", 5, UpdraftEstimator, kern_variance, 8.0f),
    
    // @Param: WIND_BL_FACT
    // @DisplayName: Wind Bound. Layer Factor
    // @Description: Fraction of true wind speed used to advect thermals.
    // @Range: 0.1 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("WIND_BL", 6, UpdraftEstimator, bl_wind_factor, 1.0f),

    // @Param: LKF_Q_W0
    // @DisplayName: LKF Process Noise - Strength
    // @Description: Expected variation in updraft strength over time.
    // @Range: 0.01 0.5
    // @User: Advanced
    AP_GROUPINFO("Q_W0", 7, UpdraftEstimator, lkf_q_w0, 0.05f),

    // @Param: LKF_Q_RAD
    // @DisplayName: LKF Process Noise - Radius
    // @Description: Expected variation in updraft shape/radius over time.
    // @Range: 0.1 5.0
    // @User: Advanced
    AP_GROUPINFO("Q_RAD", 8, UpdraftEstimator, lkf_q_rad, 0.5f),

    // @Param: LKF_Q_POS
    // @DisplayName: LKF Process Noise - Position
    // @Description: Expected drift variation in updraft position.
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("Q_POS", 9, UpdraftEstimator, lkf_q_pos, 2.0f),

    // @Param: LKF_R_W0
    // @DisplayName: LKF Meas Noise - Strength
    // @Description: Measurement confidence in the SGP strength estimate.
    // @Range: 0.01 0.5
    // @User: Advanced
    AP_GROUPINFO("R_W0", 10, UpdraftEstimator, lkf_r_w0, 0.05f),

    // @Param: LKF_R_RAD
    // @DisplayName: LKF Meas Noise - Radius
    // @Description: Measurement confidence in the SGP shape boundary estimate.
    // @Range: 5.0 100.0
    // @User: Advanced
    AP_GROUPINFO("R_RAD", 11, UpdraftEstimator, lkf_r_rad, 25.0f),

    // @Param: LKF_R_POS
    // @DisplayName: LKF Meas Noise - Position
    // @Description: Measurement confidence in the SGP peak localization.
    // @Range: 10.0 200.0
    // @User: Advanced
    AP_GROUPINFO("R_POS", 12, UpdraftEstimator, lkf_r_pos, 200.0f),

    // @Param: VETO_AGE
    // @DisplayName: Veto Max Age
    // @Description: Maximum age of local observations to be used for temporal veto.
    // @Units: s
    // @Range: 1.0 10.0
    // @User: Advanced
    AP_GROUPINFO("VETO_T", 13, UpdraftEstimator, veto_age_max, 5.0f),

    // @Param: VETO_RAD
    // @DisplayName: Veto Radius
    // @Description: Distance limit around the glider for temporal veto checks.
    // @Units: m
    // @Range: 5.0 50.0
    // @User: Advanced
    AP_GROUPINFO("VETO_R", 14, UpdraftEstimator, veto_radius, 15.0f),

    // @Param: LIFT_MIN
    // @DisplayName: Minimum Usable Lift
    // @Description: Absolute minimum updraft strength required to be considered above noise.
    // @Units: m/s
    // @Range: 0.2 2.0
    // @User: Standard
    AP_GROUPINFO("L_MIN", 15, UpdraftEstimator, lift_min, 0.4f),

    // @Param: PEAK_FRAC
    // @DisplayName: Peak Relative Fraction
    // @Description: Fraction of the global max lift required to detect another updraft peak.
    // @Range: 0.1 0.8
    // @User: Advanced
    AP_GROUPINFO("PK_FRC", 16, UpdraftEstimator, peak_fraction, 0.3f),
    
    // @Param: NOISE_FRAC
    // @DisplayName: Noise Relative Fraction
    // @Description: Fraction of the peak lift below which considered noise.
    // @Range: 0.05 0.85
    // @User: Advanced
    AP_GROUPINFO("NSE_FRC", 17, UpdraftEstimator, noise_fraction, 0.1f),
    
    // @Param: SPAWN_VAR
    // @DisplayName: Spawn Variance Threshold
    // @Description: Fraction of prior variance that the SGP peak must be below before a
    //               new catalogue entry is spawned. Higher = require more observations.
    // @Range: 0.5 0.95
    // @User: Advanced
    AP_GROUPINFO("SPN_VAR", 18, UpdraftEstimator, spawn_var_frac, 0.70f),

    // @Param: NMS_RAD
    // @DisplayName: NMS Suppression Radius (grid cells)
    // @Description: Non-maximum suppression radius in units of grid cells. Increase to
    //               merge nearby noise peaks into a single detection.
    // @Range: 2.0 5.0
    // @User: Advanced
    AP_GROUPINFO("NMS_R", 19, UpdraftEstimator, nms_suppress_rad, 4.0f),

    AP_GROUPEND
};

UpdraftEstimator::UpdraftEstimator() {}

void UpdraftEstimator::init(bool start_thread) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GLIDE: UpdraftEstimator Initialising");
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);
    
    _Kuu_matrix   = new float[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    L_kuu_local   = new float[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    _Kuu_prior    = new float[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    _L_A_shared   = new float[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    _L_Kuu_shared = new float[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    
    
    if (_Kuu_matrix == nullptr || L_kuu_local == nullptr || _Kuu_prior == nullptr || _L_A_shared == nullptr || _L_Kuu_shared == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UpdraftEstimator: Failed to allocate GP matrices in memory.");
        return;
    }

    setup_grid_geometry();
    
    if (start_thread) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&UpdraftEstimator::update_thread, void), 
                                        "UpdraftGP", 
                                        8192, 
                                        AP_HAL::Scheduler::PRIORITY_IO, 
                                        0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UpdraftEstimator: Fail to start UpdraftGP thread.");
            return;
        }
    }
    for (uint16_t i = 0; i < kuu_size; i++) {
        _L_A_shared[i] = 0.0f;
        _L_Kuu_shared[i] = 0.0f;
    }

    _matrices_valid = false;
    initialised = true;
}

void UpdraftEstimator::update_thread() {
    while (true) {
        hal.scheduler->delay(20);
        update_estimate(AP_HAL::micros());
    }
}


// MAIN FUNCTIONS

void UpdraftEstimator::update_estimate(uint32_t now_us) {
    if (!initialised || !_origin_set) {
        return;
    }
    
    sigma_n_sq = MAX(obs_noise_var.get(), 0.001f);
    tau_decay = MAX(obs_tau_decay.get(), 1.0f);

    // ---------- copy lock ----------
    bool do_update = false;
    uint16_t local_obs_count = 0;

    if (_data_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_update_requested) {
            _update_requested = false;
            _local_state = _shared_state;   // snapshot all shared fields
            do_update = true;

            // Snapshot ring-buffer observations in chronological order
            local_obs_count = _local_state.obs_count;
            for (uint16_t i = 0; i < local_obs_count; i++) {
                uint16_t idx = (_local_state.obs_head + SGP_MAX_OBSERVATIONS - local_obs_count + i) % SGP_MAX_OBSERVATIONS;
                _obs_snapshot[i] = _local_state.obs_buffer[idx];
            }

            // Snapshot catalogue for unlocked processing
            for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
                _catalogue_snapshot[i] = _catalogue[i];
            }
        }
        _data_sem.give();
    }

    if (!do_update || local_obs_count < 10) {
        return;
    }

    if (_Kuu_matrix == nullptr || L_kuu_local == nullptr) {
        return;
    }

    Vector2f local_pos = _local_origin.get_distance_NE(_local_state.loc);
    const bool grid_shifted = calculate_discrete_snap(local_pos, _local_state.ground_vel, _local_state.alt_m);

    float p_vector[SGP_NUM_INDUCING_POINTS] {};
    float new_alpha[SGP_NUM_INDUCING_POINTS] {};

    // ---------- unlocked compute ----------

    build_prior_matrix(_Kuu_matrix, _Kuu_prior, _local_state.inducing_points, _prior_ready);

    // Save/decompose pure prior into L_kuu_local
    for (uint16_t i = 0; i < kuu_size; i++) {
        L_kuu_local[i] = _Kuu_matrix[i];
    }
    const bool prior_ok = cholesky_decompose(L_kuu_local, M);

    // A = Kuu + sigma^-2 * Kuf*Kfu
    accumulate_posterior(_Kuu_matrix, p_vector, _obs_snapshot, local_obs_count, now_us, _local_state.wind_vel, _local_state.inducing_points, _catalogue_snapshot);
    
    const bool post_ok = cholesky_decompose(_Kuu_matrix, M);
    if (post_ok) {
        cholesky_solve(_Kuu_matrix, M, p_vector, new_alpha);
    } else {
        for (uint8_t i = 0; i < M; i++) {
            new_alpha[i] = 0.0f;
        }
    }

    process_updrafts(_catalogue_snapshot, new_alpha, _local_state.inducing_points, now_us, _local_state.wind_vel, _local_state.alt_m);
    if (!grid_shifted) {
        prune_inactive(_catalogue_snapshot, new_alpha, _local_state.inducing_points, _local_state.alt_m);
    }


    // Forget stale updrafts
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (_catalogue_snapshot[i].active && 
            (now_us - _catalogue_snapshot[i].last_update_us > _catalogue_snapshot[i].type.decay_time_us)) {
            _catalogue_snapshot[i].active = false;
        }
    }

    // ---------- paste lock ----------
    
    if (_data_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        // Publish Cholesky factors into estimator-owned shared buffers
        if (prior_ok && post_ok && _L_A_shared != nullptr && _L_Kuu_shared != nullptr) {
            for (uint16_t i = 0; i < kuu_size; i++) {
                _L_A_shared[i]   = _Kuu_matrix[i];
                _L_Kuu_shared[i] = L_kuu_local[i];
            }
            _matrices_valid = true;
        } else {
            // Keep last known good matrices - _matrices_valid = false;
        }

        for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
            _catalogue[i] = _catalogue_snapshot[i];
        }
        
        for (uint8_t i = 0; i < M; i++) {
            _shared_state.alpha[i] = new_alpha[i];
            _shared_state.inducing_points[i] = _local_state.inducing_points[i];
        }

        _data_sem.give();
    }
    
    const uint32_t LOG_PERIOD_US = 500000; // 0.5s
    if (_last_csv_log_us == 0 || (now_us - _last_csv_log_us) >= LOG_PERIOD_US) {
        _last_csv_log_us = now_us;
        log_updraft_to_csv(_local_state.loc, _local_state.alt_m, 0.0f, 0.0f, now_us * 0.001f);
    }
}


// INTERFACE FUNCTIONS
void UpdraftEstimator::push_observation(const Location &loc, float alt_m, float wz, uint32_t time_us) {
    if (!initialised) return;
    if (!_data_sem.take_nonblocking()) return;
    if (!_origin_set) {
        _local_origin = loc;
        _origin_set = true;
    }

    Vector2f pos_ne = _local_origin.get_distance_NE(loc);
    
    uint16_t head = _shared_state.obs_head;
    _shared_state.obs_buffer[head].local_pos = Vector3f(pos_ne.x, pos_ne.y, alt_m); 
    _shared_state.obs_buffer[head].wz = wz;
    _shared_state.obs_buffer[head].time_us = time_us;

    _shared_state.obs_head = (head + 1) % SGP_MAX_OBSERVATIONS;
    if (_shared_state.obs_count < SGP_MAX_OBSERVATIONS) {
        _shared_state.obs_count++;
    }
    _data_sem.give();
    
}

void UpdraftEstimator::trigger_update(const Location &loc, float alt_m, const Vector2f &ground_vel, const Vector2f &wind_vel) {
    if (!initialised) return;

    if (_data_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _shared_state.loc = loc;
        _shared_state.alt_m = alt_m;
        _shared_state.ground_vel = ground_vel;
        _shared_state.wind_vel = wind_vel;
        _update_requested = true;
        _data_sem.give();
    }
}

// OUT
float UpdraftEstimator::get_lift_prediction(const Location &pred_loc, float alt_m, uint32_t now_us) {
    if (!initialised || !_origin_set) {
        return 0.0f;
    }

    Vector2f pos_ne = _local_origin.get_distance_NE(pred_loc);
    Vector3f query_pos(pos_ne.x, pos_ne.y, alt_m);

    float expected_lift_sgp = 0.0f;
    float expected_lift_cat = 0.0f;

    // Snapshot shared data

    Vector2f wind_local;
    uint16_t obs_count_local = 0;


    if (!_data_sem.take_nonblocking()) {
        return 0.0f;
    }

    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        alpha_local[i] = _shared_state.alpha[i];
        ip_local[i] = _shared_state.inducing_points[i];
    }

    wind_local = _shared_state.wind_vel;
    const uint16_t obs_count_total = _shared_state.obs_count;
    const uint16_t obs_head_local  = _shared_state.obs_head;

    // Copy only the newest N in chrono order
    const uint16_t N = MIN<uint16_t>(obs_count_total, SGP_VETO_OBS_SNAPSHOT);
    obs_count_local = N;

    for (uint16_t i = 0; i < N; i++) {
        const uint16_t back = (N - 1 - i);
        const uint16_t idx  = (obs_head_local + SGP_MAX_OBSERVATIONS - 1 - back) % SGP_MAX_OBSERVATIONS;
        obs_local[i] = _shared_state.obs_buffer[idx];
    }

    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        catalog_local[i] = _catalogue[i];
    }

    _data_sem.give();

    // SGP contribution to lift
    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        float k_star = matern_kernel(query_pos, ip_local[i]);
        expected_lift_sgp += k_star * alpha_local[i];
    }

    // Catalogue contribution to lift
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!catalog_local[i].active) {
            continue;
        }

        // Use smart advection
        float bl_factor = constrain_float(bl_wind_factor.get(), 0.1f, 1.0f);
        Vector2f eff_wind = wind_local * bl_factor;
        Vector2f exp_pos2d = catalog_local[i].expected_pos(now_us, eff_wind);
        Vector3f predicted_pos(exp_pos2d.x, exp_pos2d.y, 0.0f);

        float dx = query_pos.x - predicted_pos.x;
        float dy = query_pos.y - predicted_pos.y;

        float theta = catalog_local[i].axis_heading;
        float cos_t = cosf(theta);
        float sin_t = sinf(theta);

        float d_u = (dx * cos_t) + (dy * sin_t);
        float d_v = -(dx * sin_t) + (dy * cos_t);

        // Apply physical wind stretching to the footprint
        float def_ru = catalog_local[i].radius_u();
        float def_rv = catalog_local[i].radius_v();

        float r_u_sq = MAX(sq(def_ru), 1.0f);
        float r_v_sq = MAX(sq(def_rv), 1.0f);

        float exponent = -((d_u * d_u) / r_u_sq + (d_v * d_v) / r_v_sq);
        float local_lift = catalog_local[i].strength_w0() * expf(exponent);

        if (local_lift > expected_lift_cat) {
            expected_lift_cat = local_lift;
        }
    }

    // Temporal veto using local obs
    if (expected_lift_cat > expected_lift_sgp) {
        bool recently_measured = false;

        for (int16_t i = (int16_t)obs_count_local - 1; i >= 0; i--) { // newest -> oldest
            const float age_s = (now_us - obs_local[i].time_us) * 1.0e-6f;

            // Stop if old
            if (age_s > veto_age_max.get()) {
                break;
            }

            const float dx = query_pos.x - obs_local[i].local_pos.x;
            const float dy = query_pos.y - obs_local[i].local_pos.y;

            // within rad
            float v_rad = veto_radius.get();
            if ((sq(dx) + sq(dy)) < sq(v_rad)) {
                recently_measured = true;
                break;
            }
        }

        if (recently_measured) {
            expected_lift_cat = expected_lift_sgp;
        }
    }

    return MAX(expected_lift_sgp, expected_lift_cat);
}

float UpdraftEstimator::get_variance(const Location &pred_loc, float alt_m) {
    if (!initialised || !_origin_set) {
        return kern_variance.get();
    }

    Vector2f pos_ne = _local_origin.get_distance_NE(pred_loc);
    Vector3f query_pos(pos_ne.x, pos_ne.y, alt_m);
    const float prior = matern_kernel(query_pos, query_pos);

    // Snapshot inducing points
    Vector3f local_ips[SGP_NUM_INDUCING_POINTS];
    if (!_data_sem.take_nonblocking()) {
        return prior;
    }
    for (uint8_t i = 0; i < M; i++) {
        local_ips[i] = _shared_state.inducing_points[i];
    }
    _data_sem.give();

    // calculate kernel cross-covariance
    float k_star[SGP_NUM_INDUCING_POINTS];
    for (uint8_t i = 0; i < M; i++) {
        k_star[i] = matern_kernel(query_pos, local_ips[i]);
    }

    float v_kuu[SGP_NUM_INDUCING_POINTS] = {0};
    float v_A[SGP_NUM_INDUCING_POINTS]   = {0};

    // Take lock
    if (!_data_sem.take_nonblocking()) {
        return prior;
    }

    if (!_matrices_valid || _L_A_shared == nullptr || _L_Kuu_shared == nullptr) {
        _data_sem.give();
        return prior;
    }

    // Forward Solve L_Kuu * v_kuu = k_star
    for (int i = 0; i < M; i++) {
        float sum = k_star[i];
        const int row = i * M;
        for (int k = 0; k < i; k++) { 
            sum -= _L_Kuu_shared[row + k] * v_kuu[k]; 
        }
        v_kuu[i] = sum / MAX(_L_Kuu_shared[row + i], 0.0001f);
    }

    // Forward Solve L_A * v_A = k_star
    for (int i = 0; i < M; i++) {
        float sum = k_star[i];
        const int row = i * M;
        for (int k = 0; k < i; k++) { 
            sum -= _L_A_shared[row + k] * v_A[k]; 
        }
        v_A[i] = sum / MAX(_L_A_shared[row + i], 0.0001f);
    }

    _data_sem.give();

    // Final vector dot products
    float vkuu_dot = 0.0f, vA_dot = 0.0f;
    for (uint8_t i = 0; i < M; i++) {
        vkuu_dot += sq(v_kuu[i]);
        vA_dot   += sq(v_A[i]);
    }

    return constrain_float(prior - vkuu_dot + vA_dot, 0.0f, prior);
}

bool UpdraftEstimator::get_best_global_updraft(const Location &current_loc, Location &target_loc, float &strength, uint32_t now_us) {
    if (!initialised || !_origin_set) return false;

    Vector2f pos_ne = _local_origin.get_distance_NE(current_loc);
    
    int16_t best_idx = -1;
    float best_score = -1.0f;
    Vector2f best_pos_ne{0, 0}; 
    float best_strength_val = 0.0f;

    if (_data_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
            // Skip inactive or weak updrafts
            if (!_catalogue[i].active || _catalogue[i].strength_w0() < lift_min.get()) {
                continue;
            }

            // Calculate advection
            float bl_factor = constrain_float(bl_wind_factor.get(), 0.1f, 1.0f);
            Vector2f eff_wind = _shared_state.wind_vel * bl_factor;
            Vector2f predicted_pos = _catalogue[i].expected_pos(now_us, eff_wind);

            float dx = pos_ne.x - predicted_pos.x;
            float dy = pos_ne.y - predicted_pos.y;
            float dist_sq = sq(dx) + sq(dy);

            float score = _catalogue[i].strength_w0() / sqrtf(MAX(dist_sq, 1.0f));

            if (score > best_score) {
                best_score = score;
                best_idx = i;
                best_pos_ne = predicted_pos;
                best_strength_val = _catalogue[i].strength_w0();
            }
        }
        _data_sem.give();
    }

    if (best_idx != -1) {
        target_loc = _local_origin;
        target_loc.offset(best_pos_ne.x, best_pos_ne.y); 
        strength = best_strength_val;
        return true;
    }

    return false;
}


// SPARSE GAUSSIAN PROCESS FUNCTIONS

void UpdraftEstimator::setup_grid_geometry() {
    if (_Kuu_prior == nullptr) {
        _prior_ready = false;
        return;
    }

    // Maps 9x9 grid (-4 to +4) skips the 4 corners (-4,-4), (-4,4), (4,-4), (4,4)
    uint8_t idx = 0;
    for (int8_t x = -4; x <= 4; x++) {
        for (int8_t y = -4; y <= 4; y++) {
            if ((x == 4 || x == -4) && (y == 4 || y == -4)) continue;
            
            // Initialize at origin. The discrete snap will move it over the glider.
            _shared_state.inducing_points[idx] = Vector3f(x * SGP_GRID_RES_M, y * SGP_GRID_RES_M, 80.0f);
            idx++;
        }
    }
    
    if (idx != SGP_NUM_INDUCING_POINTS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UpdraftEstimator: inducing point count mismatch");
        _prior_ready = false;
        return;
    }
        
    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        for (uint8_t j = 0; j < SGP_NUM_INDUCING_POINTS; j++) {
            _Kuu_prior[i * SGP_NUM_INDUCING_POINTS+ j] = matern_kernel(
                _shared_state.inducing_points[i],
                _shared_state.inducing_points[j]);
        }
    }
    _prior_ready = true;
}

bool UpdraftEstimator::calculate_discrete_snap(const Vector2f &current_pos, const Vector2f &ground_vel, float current_alt) {
    Vector2f vel_norm = ground_vel;
    if (vel_norm.length() > 0.1f) {
        vel_norm.normalize();
    }
    
    float ideal_x = current_pos.x + (vel_norm.x * SGP_FORWARD_BIAS_M);
    float ideal_y = current_pos.y + (vel_norm.y * SGP_FORWARD_BIAS_M);

    // Snap to nearest interval
    float snapped_x = roundf(ideal_x / SGP_GRID_RES_M) * SGP_GRID_RES_M;
    float snapped_y = roundf(ideal_y / SGP_GRID_RES_M) * SGP_GRID_RES_M;

   Vector2f current_center(_local_state.inducing_points[GRID_CENTER_IDX].x, _local_state.inducing_points[GRID_CENTER_IDX].y);

    bool shifted = false;
    if (!is_equal(snapped_x, current_center.x) || !is_equal(snapped_y, current_center.y)) {
        float shift_x = snapped_x - current_center.x;
        float shift_y = snapped_y - current_center.y;
        for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            _local_state.inducing_points[i].x += shift_x;
            _local_state.inducing_points[i].y += shift_y;
        }
        for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            _local_state.alpha[i] = 0.0f;
        }
        shifted = true;
    }

    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        _local_state.inducing_points[i].z = current_alt;
    }
    return shifted;
}

float UpdraftEstimator::matern_kernel(const Vector3f &p1, const Vector3f &p2) const {
    // Length scales
    const float l_xy = MAX(kern_length_xy.get(), 1.0f);
    const float l_z = MAX(kern_length_z.get(), 1.0f);
    const float variance = MAX(kern_variance.get(), 0.1f);

    // Calculate anisotropic distance
    float dx = (p1.x - p2.x) / l_xy;
    float dy = (p1.y - p2.y) / l_xy;
    float dz = (p1.z - p2.z) / l_z;

    float d_sq = sq(dx) + sq(dy) + sq(dz);
    float d = sqrtf(MAX(d_sq, 0.0f));

    // sigma^2 * (1 + sqrt(3)*d) * exp(-sqrt(3)*d)    sqrt(3)* precalculated
    float term = 1.73205f * d;
    return variance * (1.0f + term) * expf(-term);
}

void UpdraftEstimator::build_prior_matrix(float* kuu_matrix, const float* kuu_prior, const Vector3f* inducing_points, bool prior_ready) {

    if (prior_ready && kuu_prior != nullptr) {
        // use the precomputed prior
        memcpy(kuu_matrix, kuu_prior, kuu_size * sizeof(float));
    } else {
        // calculate from scratch if prior is unavailable
        for (uint16_t i = 0; i < kuu_size; i++) {
            kuu_matrix[i] = 0.0f;
        }
        for (uint8_t i = 0; i < M; i++) {
            for (uint8_t j = i; j < M; j++) {
                float k = matern_kernel(inducing_points[i], inducing_points[j]);
                kuu_matrix[i * M + j] += k;
                if (i != j) {
                    kuu_matrix[j * M + i] += k;
                }
            }
        }
    }

    // Prior jitter added to diagonal for stability
    for (uint8_t i = 0; i < M; i++) {
        kuu_matrix[i * M + i] += sigma_n_sq * 0.1f;
    }
}

void UpdraftEstimator::accumulate_posterior(float* kuu_matrix, float* p_vector, const UpdraftObservation* obs, uint16_t obs_count, uint32_t now_us, const Vector2f& wind_vel, const Vector3f* inducing_points, const UpdraftObject* catalogue) {
    
    // Grid-derived observation gate
    Vector2f grid_center(inducing_points[GRID_CENTER_IDX].x, inducing_points[GRID_CENTER_IDX].y);
    const float obs_gate = GRID_HALF_EXTENT_M + OBS_GATE_MARGIN_M;
    const float bl_factor = constrain_float(bl_wind_factor.get(), 0.1f, 1.0f);

    for (uint16_t i = 0; i < obs_count; i++) {
        float age_s = MAX((int32_t)(now_us - obs[i].time_us) * 1.0e-6f, 0.0f);

        // Decay the confidence of older observations
        float effective_noise_sq = sigma_n_sq * expf(age_s / tau_decay);
        Vector2f eff_wind = wind_vel * bl_factor;
        
        // check for classified non-advecting in catalogue
        for (uint8_t c = 0; c < MAX_UPDRAFT_MEM; c++) {
            if (!catalogue[c].active || catalogue[c].type.advects) {
                continue;
            }
            
            // distance check to the stationary updraft core
            float dx = obs[i].local_pos.x - catalogue[c].pos_north();
            float dy = obs[i].local_pos.y - catalogue[c].pos_east();
            
            // If the observation taken inside footprint anchor it.
            float max_rad = MAX(catalogue[c].radius_u(), catalogue[c].radius_v());
            if ((sq(dx) + sq(dy)) < sq(max_rad)) {
                eff_wind.zero();
                break;
            }
        }

        // Advect the observation downwind based on its age
        Vector3f advected_pos = obs[i].local_pos;
        advected_pos.x += eff_wind.x * age_s;
        advected_pos.y += eff_wind.y * age_s;

        // Skip if it has drifted too far outside the active grid
        if (fabsf(advected_pos.x - grid_center.x) > obs_gate ||
            fabsf(advected_pos.y - grid_center.y) > obs_gate) {
            continue;
        }

        // Calculate covariance between this observation and all inducing points
        float kfu[M]; 
        for (uint8_t j = 0; j < M; j++) {
            kfu[j] = matern_kernel(advected_pos, inducing_points[j]);
            p_vector[j] += (kfu[j] * obs[i].wz) / effective_noise_sq;
        }

        // Accumulate into the main Kuu matrix
        for (uint8_t j = 0; j < M; j++) {
            for (uint8_t k = j; k < M; k++) {
                float val = (kfu[j] * kfu[k]) / effective_noise_sq;
                kuu_matrix[j * M + k] += val;
                if (k != j) {
                    kuu_matrix[k * M + j] += val;
                }
            }
        }
    }
}

void UpdraftEstimator::extract_updraft_features(const uint8_t* cluster_indices, uint8_t cluster_size, const float* lift_weights, const Vector3f* inducing_points, UpdraftFeatures& out_features) const {
    
    const float l_xy = MAX(kern_length_xy.get(), 1.0f);
    out_features.length_major = 2.0f * l_xy;
    out_features.length_minor = 2.0f * l_xy;
    out_features.axis_heading = 0.0f;

    if (cluster_size == 0) {
        return;
    }

    // peak alpha and threshold
    float peak_val = 0.0f;
    for (uint8_t i = 0; i < cluster_size; i++) {
        const float a = lift_weights[cluster_indices[i]];
        if (a > peak_val) {
            peak_val = a;
        }
    }
    const float noise_threshold = peak_val * noise_fraction.get();

    // weighted centroid
    float sum_alpha = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    uint8_t support = 0;

    for (uint8_t i = 0; i < cluster_size; i++) {
        const uint8_t idx = cluster_indices[i];
        const float a = lift_weights[idx];
        if (a < noise_threshold) continue;

        support++;
        sum_alpha += a;
        sum_x += a * inducing_points[idx].x;
        sum_y += a * inducing_points[idx].y;
    }

    if (sum_alpha < 1e-4f || support < 4) {
        if (sum_alpha >= 1e-4f) {
            out_features.peak_pos.x = sum_x / sum_alpha;
            out_features.peak_pos.y = sum_y / sum_alpha;
        }
        return;
    }

    out_features.peak_pos.x = sum_x / sum_alpha;
    out_features.peak_pos.y = sum_y / sum_alpha;

    //  weighted covariance / PCA
    float Ixx = 0.0f, Iyy = 0.0f, Ixy = 0.0f;
    for (uint8_t i = 0; i < cluster_size; i++) {
        const uint8_t idx = cluster_indices[i];
        const float a = lift_weights[idx];
        if (a < noise_threshold) continue;

        const float dx = inducing_points[idx].x - out_features.peak_pos.x;
        const float dy = inducing_points[idx].y - out_features.peak_pos.y;
        Ixx += a * sq(dx);
        Iyy += a * sq(dy);
        Ixy += a * dx * dy;
    }
    
    // eigenvalues

    Ixx /= sum_alpha;
    Iyy /= sum_alpha;
    Ixy /= sum_alpha;

    const float trace = Ixx + Iyy;
    const float det = (Ixx * Iyy) - (Ixy * Ixy);
    const float diff = sqrtf(MAX((trace * trace) * 0.25f - det, 0.0f));

    float pca_major = sqrtf(MAX(trace * 0.5f + diff, 0.0f)) * 2.828f;
    float pca_minor = sqrtf(MAX(trace * 0.5f - diff, 0.0f)) * 2.828f;

    out_features.length_major = MAX(pca_major, l_xy);
    out_features.length_minor = MAX(pca_minor, l_xy);
    out_features.axis_heading = 0.5f * atan2f(2.0f * Ixy, Ixx - Iyy);
}

void UpdraftEstimator::process_updrafts(UpdraftObject* catalogue, const float* new_alpha, const Vector3f* inducing_points, uint32_t now_us, const Vector2f& wind_vel, float current_alt) {
    
    const float qw0 = lkf_q_w0.get(), qrad = lkf_q_rad.get(), qpos = lkf_q_pos.get();
    const float rw0 = lkf_r_w0.get(), rrad = lkf_r_rad.get(), rpos = lkf_r_pos.get();
    
    // advect all updrafts
    const float bl_factor = constrain_float(bl_wind_factor.get(), 0.1f, 1.0f);
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!catalogue[i].active) continue;
        catalogue[i].updated_this_tick = false;
        catalogue[i].kf.set_noise_params(qw0, qrad, qpos, rw0, rrad, rpos);
        
        float dt_s = MAX((now_us - catalogue[i].last_predict_us) * 1.0e-6f, 0.0f);
        float eff_wind_n = catalogue[i].type.advects ? (wind_vel.x * bl_factor) : 0.0f;
        float eff_wind_e = catalogue[i].type.advects ? (wind_vel.y * bl_factor) : 0.0f;
        
        catalogue[i].kf.predict(dt_s, eff_wind_n, eff_wind_e, catalogue[i].type.k_stretch_parallel, catalogue[i].type.k_stretch_perp);
        catalogue[i].last_predict_us = now_us;
    }

    // lift for ALL points & global maximum
    float grid_lift[SGP_NUM_INDUCING_POINTS];
    float global_max_lift = 0.0f;
    
    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        grid_lift[i] = query_sgp_lift(inducing_points[i], new_alpha, inducing_points);
        if (grid_lift[i] > global_max_lift) {
            global_max_lift = grid_lift[i];
        }
    }

    // NMS to find peak nodes
    Vector3f peaks[MAX_UPDRAFT_MEM];
    uint8_t num_peaks = 0;
    
    float detection_floor = MAX(lift_min.get(), global_max_lift * peak_fraction.get());
    float suppression_radius_sq = sq(MAX(nms_suppress_rad.get(), 2.0f) * SGP_GRID_RES_M);

    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        float my_lift = grid_lift[i];
        if (my_lift < detection_floor) continue; 

        bool is_local_max = true;
        for (uint8_t j = 0; j < SGP_NUM_INDUCING_POINTS; j++) {
            if (i == j) continue;
            if ((inducing_points[i] - inducing_points[j]).length_squared() < suppression_radius_sq) {
                if (grid_lift[j] > my_lift) {
                    is_local_max = false; 
                    break;
                }
            }
        }
        
        if (is_local_max && num_peaks < MAX_UPDRAFT_MEM) {
            peaks[num_peaks++] = inducing_points[i];
        }
    }

    // Voronoi Clustering
    float cluster_rad_m = MAX(4.0f * kern_length_xy.get(), nms_suppress_rad.get() * SGP_GRID_RES_M * 1.5f);
    float max_cluster_dist_sq = sq(cluster_rad_m);
    
    for (uint8_t p = 0; p < num_peaks; p++) {
        UpdraftFeatures features;
        features.peak_pos = peaks[p];
        uint8_t my_cluster[SGP_NUM_INDUCING_POINTS];
        uint8_t cluster_size = 0;

        // Associate nodes with positive physical lift to the closest detected peak
        for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            if (grid_lift[i] > lift_min.get()) { 
                bool is_closest = true;
                float my_dist_sq = (inducing_points[i] - peaks[p]).length_squared();
                if (my_dist_sq > max_cluster_dist_sq) {
                    continue; 
                }
                
                for (uint8_t other = 0; other < num_peaks; other++) {
                    if (other == p) continue;
                    if ((inducing_points[i] - peaks[other]).length_squared() < my_dist_sq) {
                        is_closest = false; 
                        break;
                    }
                }
                
                if (is_closest) { 
                    my_cluster[cluster_size++] = i; 
                }
            }
        }

        extract_updraft_features(my_cluster, cluster_size, grid_lift, inducing_points, features);
        features.cluster_support = cluster_size;
        features.max_lift = query_sgp_lift(features.peak_pos, new_alpha, inducing_points);

        Location peak_loc = _local_origin; 
        peak_loc.offset(features.peak_pos.x, features.peak_pos.y);
        features.core_variance = get_variance(peak_loc, current_alt);

        update_catalogue(catalogue, features, now_us, wind_vel, current_alt, inducing_points);
    }
    
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (catalogue[i].active && !catalogue[i].updated_this_tick) {
            catalogue[i].shape_gate_open = false;
        }
    }
    
    merge_overlapping_updrafts(catalogue);
    
}

void UpdraftEstimator::merge_overlapping_updrafts(UpdraftObject* catalogue) {
    const float OVERLAP_THRESHOLD = 0.10f; 

    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!catalogue[i].active) continue;

        for (uint8_t j = i + 1; j < MAX_UPDRAFT_MEM; j++) {
            if (!catalogue[j].active) continue;

            float dx = catalogue[i].pos_north() - catalogue[j].pos_north();
            float dy = catalogue[i].pos_east()  - catalogue[j].pos_east();
            float dist = sqrtf(MAX(sq(dx) + sq(dy), 0.0f));

            float avg_rad_i = (catalogue[i].radius_u() + catalogue[i].radius_v()) * 0.5f;
            float avg_rad_j = (catalogue[j].radius_u() + catalogue[j].radius_v()) * 0.5f;
            float sum_radii = (avg_rad_i + avg_rad_j) * 1.5f; // radii assume 1 sigma for usable aircraft lift, however 1.5 sigma used here to account for rest of lift skirt

            float overlap_ratio = 1.0f - (dist / MAX(sum_radii, 1.0f));

            if (overlap_ratio > OVERLAP_THRESHOLD) {
                // Determine which is stronger to keep
                uint8_t keep_idx = (catalogue[i].strength_w0() > catalogue[j].strength_w0()) ? i : j;
                uint8_t drop_idx = (keep_idx == i) ? j : i;

                catalogue[keep_idx].shape_gate_open = true;

                // inflate COVARIANCE
                catalogue[keep_idx].kf.p_Ru += 100.0f;
                catalogue[keep_idx].kf.p_Rv += 100.0f;
                catalogue[keep_idx].kf.p_N  += 50.0f;
                catalogue[keep_idx].kf.p_E  += 50.0f;

                catalogue[drop_idx].active = false;
            }
        }
    }
}

// CATALOGUE FUNCTIONS

void UpdraftEstimator::update_catalogue(UpdraftObject* catalogue, const UpdraftFeatures& features, uint32_t now_us, const Vector2f& wind_vel, float current_alt, const Vector3f* inducing_points) {
    
    // Evaluate 2D Shape Gate
    float current_var = features.core_variance;
    float prior_var = kern_variance.get();
    bool gate_conditions_met = false;
    
    // Only probe the surrounding variance if the core is confident
    if (current_var < (0.6f * prior_var)) {
        float check_dist = kern_length_xy.get();
        Location p_N = _local_origin; p_N.offset(features.peak_pos.x + check_dist, features.peak_pos.y);
        Location p_S = _local_origin; p_S.offset(features.peak_pos.x - check_dist, features.peak_pos.y);
        Location p_E = _local_origin; p_E.offset(features.peak_pos.x, features.peak_pos.y + check_dist);
        Location p_W = _local_origin; p_W.offset(features.peak_pos.x, features.peak_pos.y - check_dist);
        
        float var_N = get_variance(p_N, current_alt);
        float var_S = get_variance(p_S, current_alt);
        float var_E = get_variance(p_E, current_alt);
        float var_W = get_variance(p_W, current_alt);
        
        float avg_NS = (var_N + var_S) * 0.5f;
        float avg_EW = (var_E + var_W) * 0.5f;
        
        // ONLY open if BOTH axes have been explored
        if (avg_NS < (0.8f * prior_var) && avg_EW < (0.8f * prior_var)) {
            gate_conditions_met = true;
        }
    }

    // feature lengths to radii
    float meas_Ru = features.length_major * 0.5f;
    float meas_Rv = features.length_minor * 0.5f;
    
    float safe_axis = features.axis_heading;
    
    // Align PCA axes with wind

   float wind_heading = atan2f(wind_vel.y, wind_vel.x);
   float angle_diff = fabsf(wrap_PI(safe_axis - wind_heading)); // shortest angular distance between PCA major axis and wind
        
    // If the major axis is perpendicular to the wind (>45 deg off), swap Ru and Rv
    if (angle_diff > M_PI_4 && angle_diff < 3.0f * M_PI_4) {
        meas_Ru = features.length_minor * 0.5f;
        meas_Rv = features.length_major * 0.5f;
        
        // Rotate the reference axis 90 degrees so the LKF knows Ru is now the other way
        safe_axis = wrap_PI(safe_axis + M_PI_2);
    }
    
    float safe_Ru = gate_conditions_met ? meas_Ru : kern_length_xy.get();
    float safe_Rv = gate_conditions_met ? meas_Rv : kern_length_xy.get();
    
    // Association & Matching
    int best_idx = -1;
    float best_mah_sq = 1e9f;

    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!catalogue[i].active) continue;

        float dx = features.peak_pos.x - catalogue[i].pos_north();
        float dy = features.peak_pos.y - catalogue[i].pos_east();
        float dist_sq = sq(dx) + sq(dy);
        
        float S_N = catalogue[i].kf.p_N + catalogue[i].kf.r_N;
        float S_E = catalogue[i].kf.p_E + catalogue[i].kf.r_E;
        float d_mah_sq = sq(dx) / S_N + sq(dy) / S_E;
        
        float avg_rad = (catalogue[i].radius_u() + catalogue[i].radius_v()) * 0.5f;
        float search_radius_sq = sq(MAX(avg_rad, 15.0f)); 

        if ((d_mah_sq < 9.21f || dist_sq < search_radius_sq) && d_mah_sq < best_mah_sq) {
            best_mah_sq = d_mah_sq;
            best_idx = i;
        }
    }
    
    // Prevent double-association in the same tick
    if (best_idx != -1) {
        if (catalogue[best_idx].updated_this_tick) {
            // Another cluster already updated this entry force a spawn instead
            best_idx = -1; 
        } else {
            catalogue[best_idx].updated_this_tick = true;
        }
    }

    // Merge or Spawn
    if (best_idx != -1) {
        
        // Average Variance across the thermal footprint
        float sum_var = 0.0f;
        uint8_t count = 0;

        Location cat_loc = _local_origin;
        cat_loc.offset(catalogue[best_idx].pos_north(), catalogue[best_idx].pos_east());
        sum_var += get_variance(cat_loc, current_alt);
        count++;

        float r_u_sq = sq(MAX(catalogue[best_idx].radius_u(), 1.0f));
        float r_v_sq = sq(MAX(catalogue[best_idx].radius_v(), 1.0f));
        float cos_t = cosf(catalogue[best_idx].axis_heading);
        float sin_t = sinf(catalogue[best_idx].axis_heading);

        for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            float dx = inducing_points[i].x - catalogue[best_idx].pos_north();
            float dy = inducing_points[i].y - catalogue[best_idx].pos_east();

            float d_u = (dx * cos_t) + (dy * sin_t);
            float d_v = -(dx * sin_t) + (dy * cos_t);

            // If the cell falls inside the ellipse, query its variance
            if ((sq(d_u) / r_u_sq) + (sq(d_v) / r_v_sq) <= 1.0f) {
                Location grid_loc = _local_origin;
                grid_loc.offset(inducing_points[i].x, inducing_points[i].y);
                sum_var += get_variance(grid_loc, current_alt);
                count++;
            }
        }

        float avg_var = sum_var / (float)count;
        bool footprint_observed = (avg_var < (0.8f * prior_var));

        if (footprint_observed) {
            const float zW0 = MAX(features.max_lift, catalogue[best_idx].strength_w0() * 0.85f);
            
            if (features.cluster_support >= SGP_MIN_CLUSTER_POINTS) {
                catalogue[best_idx].kf.update_strength_pos(zW0, features.peak_pos.x, features.peak_pos.y);
                
                /* if (!catalogue[best_idx].shape_gate_open && gate_conditions_met) {
                    catalogue[best_idx].shape_gate_open = true;
                } else if (catalogue[best_idx].shape_gate_open && current_var > (0.8f * prior_var)) {
                    catalogue[best_idx].shape_gate_open = false;
                } */
                
                catalogue[best_idx].shape_gate_open = gate_conditions_met;

                if (catalogue[best_idx].shape_gate_open) {
                    catalogue[best_idx].kf.update_radii(meas_Ru, meas_Rv);
                    catalogue[best_idx].axis_heading = safe_axis;
                }
            } else {
                // Decay strength naturally, don't drag position
                catalogue[best_idx].kf.update_strength_pos(zW0, catalogue[best_idx].pos_north(), catalogue[best_idx].pos_east());
                catalogue[best_idx].shape_gate_open = false;
            }
        } else {
            // footprint is largely unobserved -> freeze the shape
            catalogue[best_idx].shape_gate_open = false;
        }

        catalogue[best_idx].last_update_us = now_us;
        
        if (footprint_observed && features.cluster_support >= SGP_MIN_CLUSTER_POINTS) {
            classify_updraft(catalogue[best_idx], features, now_us, wind_vel);
        }
    } 
    else {

        
        const bool has_confidence  = (current_var < (spawn_var_frac.get() * prior_var));
        const bool very_strong = (features.max_lift > (lift_min.get() * 3.0f));

        if (!(features.cluster_support >= SGP_MIN_CLUSTER_POINTS)) {
            return;
        }
        if (features.max_lift < lift_min.get() || features.max_lift > 10.0f) { // TODO - make this a parameter
            return;
        }
        if (!has_confidence && !very_strong) {
            return;
        }

        for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (!catalogue[i].active) {
                catalogue[i].active = true;
                catalogue[i].created_us = now_us;
                catalogue[i].updated_this_tick = true;
                catalogue[i].last_update_us = now_us;
                catalogue[i].last_predict_us = now_us;
                catalogue[i].type = UpdraftTypes::UNKNOWN;
                catalogue[i].start_pos = Vector2f(features.peak_pos.x, features.peak_pos.y);
                catalogue[i].shape_gate_open = gate_conditions_met;
                catalogue[i].axis_heading = safe_axis;
                catalogue[i].kf.init(features.max_lift, safe_Ru, safe_Rv,
                                     features.peak_pos.x, features.peak_pos.y);
                break;
            }
        }
    }
}

void UpdraftEstimator::classify_updraft(UpdraftObject& updraft, const UpdraftFeatures& features, uint32_t now_us, const Vector2f& wind_vel) const {
    float updraft_age_s = (now_us - updraft.created_us) * 1.0e-6f;

    float ratio = updraft.shape_gate_open ? (features.length_major / MAX(features.length_minor, 1.0f)) : 1.0f; // only trust PCA if shape gate is open
    // Turbulence classification: variance higher than 40% of the prior?
    bool is_turbulent = (features.core_variance > (0.4f * kern_variance.get()));
    
    Vector2f current_pos(updraft.pos_north(), updraft.pos_east());
    float abs_dist = (current_pos - updraft.start_pos).length();
    float expected_wind_dist = (wind_vel * updraft_age_s).length();
    bool is_anchored = (abs_dist < 40.0f && expected_wind_dist > 50.0f) || (wind_vel.length() < 0.5f);
    bool long_lived = (updraft_age_s > 300.0f);

    uint8_t type_sel = (is_anchored ? 2 : 0) + (is_turbulent ? 1 : 0);

    switch (type_sel) {
        case 2: // Anchored & Smooth
            if (ratio > 3.0f) {
                updraft.type = UpdraftTypes::RIDGE;
            } else if (long_lived) {
                updraft.type = UpdraftTypes::OROGRAPHIC;
            } else {
                updraft.type = UpdraftTypes::THERMAL_CHIMNEY;
            }
            break;

        case 3: // Anchored & Turbulent
            updraft.type = (ratio > 2.0f) ? UpdraftTypes::RIDGE : UpdraftTypes::OROGRAPHIC;
            break;

        case 1: // Drifting & Turbulent
            if (ratio > 3.0f)      updraft.type = UpdraftTypes::SHEAR;
            else if (ratio > 1.5f) updraft.type = UpdraftTypes::THERMAL_COMPLEX;
            else                   updraft.type = UpdraftTypes::THERMAL_BUBBLE;
            break;

        case 0: // Drifting & Smooth
            updraft.type = (ratio > 2.0f) ? UpdraftTypes::SHEAR : UpdraftTypes::THERMAL_BUBBLE;
            break;
    }
}

void UpdraftEstimator::prune_inactive(UpdraftObject* catalogue, const float* new_alpha, const Vector3f* inducing_points, float current_alt) {
    Vector2f current_grid_center(inducing_points[GRID_CENTER_IDX].x, inducing_points[GRID_CENTER_IDX].y);
    
    float min_lift = lift_min.get(); // Cache the parameter
    
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!catalogue[i].active) continue;
        
        if (catalogue[i].strength_w0() < min_lift || catalogue[i].strength_w0() > 10.0f) { // too weak or physically impossible
            catalogue[i].active = false;
            continue;
        }

        float dx = catalogue[i].pos_north() - current_grid_center.x;
        float dy = catalogue[i].pos_east() - current_grid_center.y;

        // If catalog has thermal close by but SGP doesn't - not active
        if ((sq(dx) + sq(dy)) < 1600.0f) { //sq(40.0f)
            Vector3f query_pos(catalogue[i].pos_north(), catalogue[i].pos_east(), current_alt);
            float actual_sgp_lift = query_sgp_lift(query_pos, new_alpha, inducing_points);

            if (actual_sgp_lift < min_lift) {
                catalogue[i].active = false;
            }
        }
    }
}



// MATRIX MATHS


bool UpdraftEstimator::cholesky_decompose(float* matrix, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            float sum = matrix[i * size + j];
            
            for (uint8_t k = 0; k < j; k++) {
                sum -= matrix[i * size + k] * matrix[j * size + k];
            }
            
            if (i == j) {
                // Matrix is not positive definite
                if (sum <= 0.0f) {
                    return false; 
                }
                matrix[i * size + i] = sqrtf(sum);
            } else {
                matrix[i * size + j] = sum / matrix[j * size + j];
                matrix[j * size + i] = 0.0f; 
            }
        }
    }
    return true;
}

void UpdraftEstimator::cholesky_solve(const float* L_matrix, uint8_t size, const float* in_vector, float* out_vector) {
    // Forward substitution
    for (uint8_t i = 0; i < size; i++) {
        float sum = in_vector[i];
        for (uint8_t k = 0; k < i; k++) {
            sum -= L_matrix[i * size + k] * out_vector[k];
        }
        out_vector[i] = sum / MAX(L_matrix[i * size + i], 0.0001f);
    }
    
    // Backward substitution
    for (int8_t i = size - 1; i >= 0; i--) {
        float sum = out_vector[i];
        for (uint8_t k = i + 1; k < size; k++) {
            sum -= L_matrix[k * size + i] * out_vector[k]; 
        }
        out_vector[i] = sum / MAX(L_matrix[i * size + i], 0.0001f);
    }
}


// LOGGING

void UpdraftEstimator::log_ue_sgp(uint32_t now_us) {
#if HAL_LOGGING_ENABLED
    if (!_log_enable || !initialised || !_origin_set) {
        return;
    }

    // 1 Hz
    if ((now_us - _last_sgp_log_us) < 1000000UL) {
        return;
    }
    _last_sgp_log_us = now_us;

    // Snapshot under lock
    Vector2f wind_local;
    Location origin_local;

    if (!_data_sem.take_nonblocking()) {
        return;
    }

    for (uint8_t idx = 0; idx < SGP_NUM_INDUCING_POINTS; idx++) {
        ip_local[idx] = _shared_state.inducing_points[idx];
        alpha_local[idx] = _shared_state.alpha[idx];
    }
    wind_local = _shared_state.wind_vel;
    origin_local = _local_origin;

    _data_sem.give();

    // One inducing point per record
    for (uint8_t idx = 0; idx < SGP_NUM_INDUCING_POINTS; idx++) {
        AP::logger().Write(
            "UPGP",
            "t,i,olat,olng,wn,we,n,e,z,a",
            "----------",
            "0000000000",
            "Qbiiffffff",
            (uint64_t)now_us,
            (int8_t)idx,
            (int32_t)origin_local.lat,
            (int32_t)origin_local.lng,
            wind_local.x,
            wind_local.y,
            ip_local[idx].x,
            ip_local[idx].y,
            ip_local[idx].z,
            alpha_local[idx]
        );
    }
#endif
}

void UpdraftEstimator::log_ue_cat(uint32_t now_us) {
#if HAL_LOGGING_ENABLED
    if (!_log_enable || !initialised) {
        return;
    }
    // 0.5 Hz
    if ((now_us - _last_cat_log_us) < 2000000UL) {
        return;
    }
    _last_cat_log_us = now_us;

    // Snapshot catalogue under lock
    UpdraftObject cat_snap[MAX_UPDRAFT_MEM];
    if (!_data_sem.take_nonblocking()) {
        return;
    }
    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        cat_snap[i] = _catalogue[i];
    }
    _data_sem.give();

    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        const auto& c = cat_snap[i];
        AP::logger().Write(
            "UPCT",
            "t,i,act,typ,w0,ru,rv,hdg,n,e,age",
            "-----------",
            "00000000000",
            "QBBBfffffff",
            (uint64_t)now_us,
            (uint8_t)i,
            (uint8_t)(c.active ? 1 : 0),
            (uint8_t)c.type.id,
            c.strength_w0(),
            c.radius_u(),
            c.radius_v(),
            c.axis_heading,
            c.pos_north(),
            c.pos_east(),
            (float)((now_us - c.last_update_us) * 1.0e-6f)
        );
    }
#endif
}

void UpdraftEstimator::log_updraft_to_csv(const Location &plane_loc, float plane_alt_m, float plane_heading_deg, float wz, uint32_t time_ms) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static FILE *csv_file = nullptr;
    static bool header_written = false;

    if (csv_file == nullptr) {
        csv_file = fopen("libraries/AP_MissionSoaring/updraft_state_output.csv", "w");
        if (csv_file == nullptr) {
            return;
        }
    }

    if (!initialised || !_origin_set) {
        return;
    }

    Vector2f wind_local;
    UpdraftObject cat_local[MAX_UPDRAFT_MEM];

    if (!_data_sem.take_nonblocking()) {
        return;
    }

    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        alpha_local[i] = _shared_state.alpha[i];
        ip_local[i]    = _shared_state.inducing_points[i];
    }
    wind_local = _shared_state.wind_vel;

    for (uint8_t i = 0; i < MAX_UPDRAFT_MEM; i++) {
        cat_local[i] = _catalogue[i];
    }

    _data_sem.give();

    if (!header_written) {
        fprintf(csv_file,
                "Time_ms,PlaneLat,PlaneLng,PlaneAlt,PlaneHdg,Wz,WindN,WindE");

        // Inducing points: C{i}_dN,C{i}_dE,C{i}_mean,C{i}_var
        for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            fprintf(csv_file, ",C%u_dN,C%u_dE,C%u_mean,C%u_var", i, i, i, i);
        }

        // Catalogue entries:
        // State: act,typ,w0,radU,radV,hdg,dN,dE
        // Cov:  p_w0,p_ru,p_rv,p_n,p_e
        for (uint8_t i = 0; i < 4; i++) {
            fprintf(csv_file,
                    ",Cat%u_act,Cat%u_typ,Cat%u_w0,Cat%u_radU,Cat%u_radV,Cat%u_hdg,Cat%u_dN,Cat%u_dE",
                    i, i, i, i, i, i, i, i);
        }

        fprintf(csv_file, "\n");
        header_written = true;
    }

    // Compute position in estimator local frame (NED)
    Vector2f plane_ne = _local_origin.get_distance_NE(plane_loc);

    fprintf(csv_file, "%u,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f",
            (unsigned)time_ms,
            (double)(plane_loc.lat * 1.0e-7),   // degrees
            (double)(plane_loc.lng * 1.0e-7),   // degrees
            (double)plane_alt_m,
            (double)plane_heading_deg,
            (double)wz,
            (double)wind_local.x,
            (double)wind_local.y);

    // Inducing points: mean + variance at each inducing point
    // mean: k(x, u)^T * alpha (u == inducing points)
    for (uint8_t i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {

        const float dN = ip_local[i].x - plane_ne.x;
        const float dE = ip_local[i].y - plane_ne.y;

        float mean = 0.0f;
        const Vector3f q(ip_local[i].x, ip_local[i].y, plane_alt_m);

        for (uint8_t j = 0; j < SGP_NUM_INDUCING_POINTS; j++) {
            const float k = matern_kernel(q, ip_local[j]);
            mean += k * alpha_local[j];
        }

        Location qloc = _local_origin;
        qloc.offset(ip_local[i].x, ip_local[i].y);

        const float var = get_variance(qloc, plane_alt_m);

        fprintf(csv_file, ",%.3f,%.3f,%.3f,%.3f",
                (double)dN, (double)dE, (double)mean, (double)var);
    }

    // Catalogue: state + covariance
    for (uint8_t i = 0; i < 4; i++) {
        const UpdraftObject &c = cat_local[i];

        const int act = c.active ? 1 : 0;
        const int typ = (int)c.type.id;

        const float dN = c.pos_north() - plane_ne.x;
        const float dE = c.pos_east()  - plane_ne.y;

        fprintf(csv_file,
                ",%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                act,
                typ,
                (double)c.strength_w0(),
                (double)c.radius_u(),
                (double)c.radius_v(),
                (double)c.axis_heading,
                (double)dN,
                (double)dE);
    }

    fprintf(csv_file, "\n");
    fflush(csv_file);
#endif
}