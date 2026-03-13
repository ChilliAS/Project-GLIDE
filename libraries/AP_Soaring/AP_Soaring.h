/*
  Soaring Controller class
  by Amhar S for Project GLIDE
*/

#pragma once

#include "AP_Soaring_config.h"
#if HAL_SOARING_ENABLED

#include <AP_TECS/AP_TECS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <stdio.h>

#include "ExtendedKalmanFilter.h"
#include "Variometer.h"
#include <AP_Logger/AP_Logger.h>


class SoaringController {
public:
    SoaringController(AP_TECS &tecs, const AP_FixedWing &parms); //takes TECS instead of AHRS to maintain compatibility with standard ArduPilot where poss

    // Call at loop rate -  20Hz
    void update();

    static const struct AP_Param::GroupInfo var_info[];
	
	// interfaces from old soaring code expected by other libraries
	enum class ActiveStatus { 
        SOARING_DISABLED = 0,
        SOARING_ENABLED = 1,
		MANUAL_MODE_CHANGE = 2,
		AUTO_MODE_CHANGE = 3
    };
	
	float get_vario_reading() const { return _vario.reading; }
    void init_cruising();
    bool get_throttle_suppressed() const;
    bool is_active() const { return soar_enable.get() > 0; }
    void update_active_state(bool override_disable);
	float get_thermalling_target_airspeed() const;
	float get_cruising_target_airspeed() const;
	int8_t get_thermalling_flap() const { return 0; }
	float get_alt_cutoff() const;

    // DATA STRUCTURES
	// 36-byte binary header for heatmap
	struct PACKED HeatmapHeader {
        uint32_t magic;           // 0x47503533 ('GP53')
        int32_t origin_lat_e7;    // Lat * 10^7
        int32_t origin_lng_e7;    // Lng * 10^7
        uint16_t grid_width;      // Grid Width (cells)
        uint16_t grid_height;     // Grid Height (cells)
        float resolution_m;       // Cell size (m)
        
        // Mission Parameters
        float floor_alt;          // Min altitude (m)
        float ceiling_alt;        // Max altitude (m)
        float start_battery_mah;  // Planned Battery Capacity
        uint16_t mission_time_s;  // Mission time (seconds)
        uint8_t reserve_rth;      // 1 = enable
        uint8_t padding;          // padding
    };
    
    // throttle settings and associated performance
    struct PerformanceStep {
        int8_t throttle_pct;
        float power_amps;
        float vz_still_air;
    };

    // action space
    struct SoaringAction {
        float bank_angle;      // Deg
        int8_t throttle_pct;   // %
        
        // Reward Breakdown for Logging
        float score_total;     
        float score_mission;   
        float score_energy;    
        float score_safety;    
        float current_lambda;  
        
        bool is_valid;         
    };
	
	// parameters from vehicle - by decoupling soaring controller from AP - can do unit testing
	struct VehicleState {
        float alt_m;
        float tas_m_s;
        float heading_true_rad;
        Vector3f wind;
        Location current_loc;
        float battery_remaining_mah;
        uint32_t time_ms;
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
	
	float get_target_bank_angle_cd() const;
	int8_t get_target_throttle_pct() const;
	float _lambda_lagrange = 0.5f;   // "cost" of energy (0-1)
	
	void load_heatmap(); 				// Load heatmap and parameters from file
	void update_strategic_loop(const VehicleState &state);       // Strategic Loop
	SoaringAction calculate_optimal_action(const VehicleState &state);
	
	// Failsafe
	bool is_healthy();
    uint8_t get_failsafe_action() const;
	

private:
    // AP Objects
	AP_TECS &_tecs;
    AP_AHRS &_ahrs;
	const AP_FixedWing &_aparm;
	Variometer::PolarParams _polar_params;
	
	Variometer _vario;
    ExtendedKalmanFilter _ekf;

    // PARAMETER
    AP_Int8  soar_enable;       // 1 = Active
    AP_Float soar_beta;         // Mission vs Energy factor
    AP_Int8  soar_map_rad;      // Heatmap Search Radius (cells)
    AP_Float max_gsd_alt;       // Optimal altitude (m)
    AP_Float cmdp_alpha;        // Learning rate
	AP_Int8  soar_fs_action;    // 0 = RTL, 1 = FBWA
	AP_Float soar_v_glide;      // Target airspeed when gliding/thermalling
	AP_Float soar_cam_hfov;     // Camera horizontal FOV
    AP_Float soar_cam_vfov;		// Camera vertical FOV
    AP_Float soar_dep_fact;		// Mission Cell Depletion factor (priority/second)
	
	// from mission file
	float _mission_alt_min = 50.0f;     
    float _mission_alt_max = 120.0f;    
    float _mission_batt_mah = 5000.0f;  
    uint32_t _mission_time_s = 1800;    
    bool _mission_rth_enable = true;

    // INTERNAL STATE
    // Strategic Loop
    
    float _budget_slope = 0.0f;      // Expected mAh consumption per second
    uint32_t _start_time_ms = 0;     // Mission start time
    float _start_batt_cap = 0.0f;    // Battery capacity at start
    uint32_t _last_strategic_update_ms = 0;
	uint32_t _last_tactical_update_ms = 0;
	
	Location _nearest_target;
    bool _has_nearest_target = false;
	uint8_t _nearest_target_score = 0;

    SoaringAction _last_action;
    float _last_best_score = -FLT_MAX;

    // Thermal Memory
    static const uint8_t MAX_THERMALS = 10;
    ThermalObject _thermal_memory[MAX_THERMALS];
	uint32_t _last_thermal_update_ms = 0;

    // Heatmap Config
    uint8_t *_heatmap_data = nullptr; 
    int16_t _grid_width = 500;
    int16_t _grid_height = 500;
    float _grid_resolution_m = 10.0f;
    Location _heatmap_origin;

    // Constants
    static const PerformanceStep _perf_table[3];

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
	void save_heatmap();
};

#endif // HAL_SOARING_ENABLED
