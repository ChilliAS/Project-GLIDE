/*
  Soaring Controller class
  by Amhar S for Project GLIDE
*/

#pragma once

#include "AP_MissionSoaring_config.h"
#if HAL_MISSIONSOARING_ENABLED

//#include <AP_TECS/AP_TECS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_FixedWing.h>

#include <AP_Soaring/ExtendedKalmanFilter.h>
#include <AP_Soaring/Variometer.h>
#include <AP_Logger/AP_Logger.h>

#include "ExtendedVariometer.h"
#include "UpdraftEstimator.h"

struct MSoaringVehicleState {
    float alt_m;
    float tas_m_s;
    float heading_true_rad;
    Vector3f wind;
    Location current_loc;
    float battery_remaining_mah;
    uint32_t time_us;
    bool is_armed;
    float dist_to_home_m;
    float cruise_spd_m_s;
    float airmass_rate_m_s;
    Vector2f pos_ne_m;
    Vector2f ground_vel_m_s; // for updraft estimator

    // Extended Variometer
    float roll_rad;
    float pitch_rate_rads;
    float roll_rate_rads;
    float batt_voltage;
    float batt_current_a;
    float air_density;
};


class MSoaringController {
public:
    MSoaringController(AP_AHRS &ahrs, const AP_FixedWing &parms); 
    void init();

	// DATA STRUCTURES
	
	static const struct AP_Param::GroupInfo var_info[];
	
	// Binary header for mission file
	struct PACKED missionHeader {
        uint32_t magic;           // 0x47503533 ('GP53')
        int32_t origin_lat_e7;    // Lat * 10^7
        int32_t origin_lng_e7;    // Lng * 10^7
        uint16_t grid_width_m;      // Grid Width (cells)
        uint16_t grid_height_m;     // Grid Height (cells)
        float resolution_m;       // Cell size (m)
        
        // Mission Parameters
        float floor_alt;          // Min altitude (m)
        float ceiling_alt;        // Max altitude (m)
        float start_battery_mah;  // Planned Battery Capacity
        uint16_t mission_time_s;  // Mission time (seconds)
        uint8_t reserve_rth;      // 1 = enable
        uint8_t padding;          // padding
    };

	// Throttle settings and associated performance 
    struct PerformanceStep {
        int8_t throttle_pct;
        float power_amps;
        float vz_still_air;
    };
	
    // Thermal Memory
    struct ThermalObject {float roll_rad;
        float pitch_rate_rads;
        float roll_rate_rads;
        float batt_voltage;
        float batt_current_a;
        bool  has_accel_fwd;
        float accel_fwd_m_s2;
        float air_density;
        Location center_loc; 
        float strength_w0;   // Peak lift (m/s)
        float radius_r0;     // Radius (m)
        uint32_t last_update_ms; 
        bool active;         
    };
	
	// Action space
    struct SoaringAction {
        float bank_angle;      // Deg
        int8_t throttle_pct;   // %
        float score_total;     
        float score_mission;   
        float score_energy;    
        float score_safety;    
        float current_lambda;  
        
        bool is_valid;         
    };
	
	
	UpdraftEstimator updraft_estimator;
    
	// VARIABLES
	
	float lambda_lagrange = 0.5f;	// "cost" of energy (0-1)	


	// FUNCTIONS

	// Operational Functions

    void update_tactical_loop();					// 20 Hz
	void update_strategic_loop();                   // 1 Hz
    void update_tactical_loop(const MSoaringVehicleState &state); // overloaded for unit testing
    void update_strategic_loop(const MSoaringVehicleState &state); // overloaded for unit testing
	SoaringAction get_last_action() const { return last_action; }
    SoaringAction calculate_optimal_action(const MSoaringVehicleState &state);
	
	// Helper Functions
	
	void load_mission();			// Load mission and parameters from file
	
	// Output Functions
	
	bool is_healthy();
    uint8_t get_failsafe_action() const;
	bool is_active() const { return msoar_enable.get() > 0; }
	float get_target_bank_angle_cd() const;
	int8_t get_target_throttle_pct() const;
	float get_thermalling_target_airspeed() const;


    void set_logging_enabled(bool enable) { _log_enable = enable; }
    bool logging_enabled() const { return _log_enable; }
    void log_estimators(uint32_t now_us) {
        if (!_log_enable) return;
        updraft_estimator.log_ue_sgp(now_us);
        updraft_estimator.log_ue_cat(now_us);
    }


private:
    // AP Objects
    AP_AHRS &_ahrs;
	const AP_FixedWing &_aparm;
	
    ExtendedKalmanFilter _ekf;
    ExtendedVariometer _extended_vario;

    static const uint32_t MAX_HEATMAP_BYTES = 125000;
    
    // PARAMETERS
    AP_Int8  msoar_enable;       // 1 = Active
    AP_Float msoar_alpha;        // Learning rate
    AP_Float msoar_beta;         // Mission vs Energy factor
    AP_Int8  msoar_mis_search_rad;      // mission Search Radius (cells)
    AP_Float tgt_alt;       // Optimal altitude (m)
	AP_Float msoar_v_glide;      // Target airspeed
	AP_Int8  msoar_fs_action;    // 0 = RTL, 1 = FBWA
	AP_Float msoar_cam_hfov;     // Camera horizontal FOV
    AP_Float msoar_cam_vfov;		// Camera vertical FOV
    AP_Float msoar_dep_fact;		// Mission Cell Depletion factor (priority/second)
	
    // Performance Table Parameters
    AP_Float msoar_p0_amp;
    AP_Float msoar_p0_vz;
    
    AP_Int8  msoar_p1_thr;
    AP_Float msoar_p1_amp;
    AP_Float msoar_p1_vz;
    
    AP_Int8  msoar_p2_thr;
    AP_Float msoar_p2_amp;
    AP_Float msoar_p2_vz;
    
    AP_Int8  msoar_p3_thr;
    AP_Float msoar_p3_amp;
    AP_Float msoar_p3_vz;
    
    AP_Int8  msoar_p4_thr;
    AP_Float msoar_p4_amp;
    AP_Float msoar_p4_vz;
    
    // Tuning Multipliers
    AP_Float msoar_m_glb_upd;   // Global updraft pull multiplier
    AP_Float msoar_m_glb_mis;   // Global mission pull multiplier
    AP_Float msoar_m_mis_tot;   // Total mission score multiplier
    AP_Float msoar_m_eng_tot;   // Total energy score multiplier
    
    // Tuning Constants
    AP_Float msoar_pen_mot;     // Motor start penalty
    AP_Float msoar_k_safe;      // Safety penalty multiplier
    AP_Float msoar_k_greed;     // Amps cost multiplier
    AP_Float msoar_alt_buf;     // Altitude floor buffer
    AP_Float msoar_k_thr_hyst;  // Throttle hysteresis penalty
    AP_Int8  msoar_max_bank;    // Maximum bank angle evaluated
    
	// MISSION FILE VARIABLES
	float mis_alt_min_m = 50.0f;     
    float mis_alt_max_m = 120.0f;    
    float mis_batt_mah = 5000.0f;  
    uint32_t mis_time_s = 1800;    
    bool mis_rth_enable = true;
    
    // IO THREADING
    HAL_Semaphore mission_sem;     // Mutex protect mission_heatmap
    bool io_thread_started = false;
    bool mission_map_changed = false;    // tell the background thread to save

    void io_thread();               
    void init_io_thread();          

    // STATE VARS
    
    float budget_slope = 0.0f;      // Expected mAh consumption per second
    uint32_t mission_start_time_us = 0;     // Mission start time
    float start_batt_mah = 0.0f;    // Battery capacity at start
    //uint32_t last_strategic_update_ms = 0;
	uint32_t last_tactical_update_us = 0;
    uint32_t last_vario_update_us = 0;
	
	Location nearest_target;
    bool has_nearest_target = false;
	uint8_t nearest_target_score = 0;
    
    Location global_thermal_loc;
    float global_thermal_strength = 0.0f;
    bool has_global_thermal = false;

    SoaringAction last_action;
    float last_best_score = -FLT_MAX;

    // Thermal Memory

    // mission Config
    uint8_t *mission_heatmap = nullptr;
    bool mission_loaded = false;
    bool mission_load_requested = true; 
    
    int16_t grid_width_m = 500;
    int16_t grid_height_m = 500;
    float grid_res_m = 10.0f;
    Location heatmap_origin;

    PerformanceStep perf_table[5];

    // FUNCTIONS
    
    // Core Logic
	MSoaringVehicleState get_current_state();
	void update_thermals(const MSoaringVehicleState &state, float dt);

    // Helpers
    Location predict_position_future(const MSoaringVehicleState &state, float bank_angle, float dt);
    float predict_thermal_lift(const MSoaringVehicleState &state, const Location &pred_loc);
    float get_local_density_score(const Location &loc);
    bool get_grid_coords_from_loc(const Location &loc, int16_t &x, int16_t &y);
    void update_perf_table();

    // Logging & Output
    void Log_Write_Soaring(const SoaringAction &action);
	void save_mission();
    bool _log_enable = true;
    uint32_t _last_soar_log_us = 0;
};

#endif // HAL_MISSIONSOARING_ENABLED
