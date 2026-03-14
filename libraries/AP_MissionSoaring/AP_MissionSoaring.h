/*
  Soaring Controller class
  by Amhar S for Project GLIDE
*/

#pragma once

#include "AP_MissionSoaring_config.h"
#if HAL_MISSIONSOARING_ENABLED

#include <AP_TECS/AP_TECS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_FixedWing.h>

#include "ExtendedKalmanFilter.h"
#include "Variometer.h"
#include <AP_Logger/AP_Logger.h>

class MSoaringController {
public:
    MSoaringController(AP_TECS &tecs, const AP_FixedWing &parms); //takes TECS instead of AHRS to maintain compatibility with standard ArduPilot where poss

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
	// TODO Move to config
    struct PerformanceStep {
        int8_t throttle_pct;
        float power_amps;
        float vz_still_air;
    };
	
	// Parameters from vehicle - by decoupling soaring controller from AP - can do unit testing
	struct VehicleState {
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
    };

    // Thermal Memory
    struct ThermalObject {
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
	
	// interfaces from old soaring code expected by other libraries
	//enum class ActiveStatus { 
    //    SOARING_DISABLED = 0,
    //    SOARING_ENABLED = 1,
	//	MANUAL_MODE_CHANGE = 2,
	//	AUTO_MODE_CHANGE = 3
    //};
	
	
	// VARIABLES
	
	float lambda_lagrange = 0.5f;	// "cost" of energy (0-1)	


	// FUNCTIONS

	// Operational Functions

    void update_tactical_loop();					// 20 Hz
	void update_strategic_loop();                   // 1 Hz
	SoaringAction calculate_optimal_action(const VehicleState &state);
	
	// Helper Functions
	
	void load_mission();			// Load mission and parameters from file
	
	// Output Functions
	
	bool is_healthy();
    uint8_t get_failsafe_action() const;
	bool is_active() const { return msoar_enable.get() > 0; }
	float get_vario_reading() const { return _vario.reading; }
	float get_target_bank_angle_cd() const;
	int8_t get_target_throttle_pct() const;
	float get_thermalling_target_airspeed() const;
	//float get_cruising_target_airspeed() const; 			// old AP_Soaring stuff
	//int8_t get_thermalling_flap() const { return 0; }
	//float get_alt_cutoff() const;
	//bool get_throttle_suppressed() const;

    //void init_cruising();
    //void update_active_state(bool override_disable);


private:
    // AP Objects
	AP_TECS &_tecs;
    AP_AHRS &_ahrs;
	const AP_FixedWing &_aparm;
	Variometer::PolarParams _polar_params;
	
	Variometer _vario;
    ExtendedKalmanFilter _ekf;

    static const uint32_t MAX_HEATMAP_BYTES = 125000;
    
    // PARAMETERS
    AP_Int8  msoar_enable;       // 1 = Active
    AP_Float msoar_alpha;        // Learning rate
    AP_Float msoar_beta;         // Mission vs Energy factor
    AP_Int8  msoar_mis_search_rad;      // mission Search Radius (cells)
    AP_Float tgt_alt;       // Optimal altitude (m)
	AP_Float msoar_v_glide;      // Target airspeed when gliding/thermalling
	AP_Int8  msoar_fs_action;    // 0 = RTL, 1 = FBWA
	AP_Float msoar_cam_hfov;     // Camera horizontal FOV
    AP_Float msoar_cam_vfov;		// Camera vertical FOV
    AP_Float msoar_dep_fact;		// Mission Cell Depletion factor (priority/second)
	
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
	
	Location nearest_target;
    bool has_nearest_target = false;
	uint8_t nearest_target_score = 0;

    SoaringAction last_action;
    float last_best_score = -FLT_MAX;

    // Thermal Memory
    static const uint8_t MAX_THERMALS = 10;
    ThermalObject thermal_memory[MAX_THERMALS];
	uint32_t last_thermal_update_ms = 0;

    // mission Config
    uint8_t *mission_heatmap = nullptr;
    bool mission_loaded = false;
    bool mission_load_requested = true; 
    
    int16_t grid_width_m = 500;
    int16_t grid_height_m = 500;
    float grid_res_m = 10.0f;
    Location heatmap_origin;

    static const PerformanceStep perf_table[3];

    // FUNCTIONS
    
    // Core Logic
	VehicleState get_current_state();
	void update_thermals(const VehicleState &state, float dt);

    // Helpers
    Location predict_position_future(const VehicleState &state, float bank_angle, float dt);
    float predict_thermal_lift(const VehicleState &state, const Location &pred_loc);
    float get_local_density_score(const Location &loc);
    bool get_grid_coords_from_loc(const Location &loc, int16_t &x, int16_t &y);

    // Logging & Output
    void Log_Write_Soaring(const SoaringAction &action);
	void log_state_to_csv(const VehicleState &state, const SoaringAction &action);
	void save_mission();
};

#endif // HAL_SOARING_ENABLED
