#include "AP_Soaring.h"

#if HAL_SOARING_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <stdint.h>

static const uint8_t LOG_SOAR_REWARD_MSG = 201;

extern const AP_HAL::HAL& hal;

// SOARING PARAMS
const AP_Param::GroupInfo SoaringController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable 3D Soaring
    // @Description: Activates the RC-MO-POMDP controller
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, SoaringController, soar_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: CMDP_ALPHA
    // @DisplayName: Energy Price Learning Rate
    // @Description: How fast Lambda reacts to budget deviation.
    AP_GROUPINFO("CMDP_ALPHA", 2, SoaringController, cmdp_alpha, 0.001f),

    // @Param: BETA
    // @DisplayName: Mission Reward Scaler
    // @Description: Multiplier to balance Heatmap score vs Amps.
    AP_GROUPINFO("BETA", 3, SoaringController, soar_beta, 10.0f),

    // @Param: MAP_RAD
    // @DisplayName: Heatmap Search Radius
    // @Description: Radius in grid cells (strided) to scan for density.
    AP_GROUPINFO("MAP_RAD", 4, SoaringController, soar_map_rad, 4),
	
	// @Param: TGT_ALT
    // @DisplayName: Optimal GSD Altitude
    // @Description: Altitude in meters for maximum sensor score (1.0 factor).
    // @Range: 10 500
    AP_GROUPINFO("TGT_ALT", 5, SoaringController, max_gsd_alt, 100.0f),
	
	// @Param: POLAR_K
    // @DisplayName: Polar Drag K
    // @Description: Induced drag coefficient for the airframe.
    AP_GROUPINFO("POLAR_K", 6, SoaringController, _polar_params.K, 25.6f),

    // @Param: POLAR_CD0
    // @DisplayName: Polar Drag CD0
    // @Description: Parasitic drag coefficient for the airframe.
    AP_GROUPINFO("POLAR_CD0", 7, SoaringController, _polar_params.CD0, 0.027f),

    // @Param: POLAR_B
    // @DisplayName: Polar Drag B
    // @Description: Sink rate base coefficient.
    AP_GROUPINFO("POLAR_B", 8, SoaringController, _polar_params.B, 0.036f),
	
	// @Param: V_GLIDE
    // @DisplayName: Soaring Glide Airspeed
    // @Description: Target airspeed to maintain while gliding or thermalling.
    // @Units: m/s
    // @Range: 8 30
    // @Increment: 0.5
    AP_GROUPINFO("V_GLIDE", 9, SoaringController, soar_v_glide, 12.0f),
	
	// @Param: FS_SOAR
    // @DisplayName: Soaring Failsafe Action
    // @Description: Mode to switch to if soaring controller becomes unhealthy. 0:RTL, 1:FBWA.
    // @Values: 0:RTL, 1:FBWA
    AP_GROUPINFO("FS_SOAR", 10, SoaringController, soar_fs_action, 1),
	
	// @Param: CAM_HFOV
    // @DisplayName: Camera Horizontal FOV
    // @Description: Horizontal FOV in degrees (across the flight path).
    // @Range: 10 150
    AP_GROUPINFO("CAM_HFOV", 11, SoaringController, soar_cam_hfov, 102.0f),

    // @Param: CAM_VFOV
    // @DisplayName: Camera Vertical FOV
    // @Description: Vertical FOV in degrees (along the flight path).
    // @Range: 10 150
    AP_GROUPINFO("CAM_VFOV", 12, SoaringController, soar_cam_vfov, 67.0f),

    // @Param: DEP_FACT
    // @DisplayName: Cell Depletion Factor
    // @Description: Multiplier applied to cell score per second when overflown (0.0 to 1.0).
    AP_GROUPINFO("DEP_FACT", 13, SoaringController, soar_dep_fact, 0.5f),

    AP_GROUPEND
};

const SoaringController::PerformanceStep SoaringController::_perf_table[] = {
    { 0,   1.0f,  -0.8f }, // Gliding
    { 60, 12.0f,   1.5f }, // Cruise
    { 90, 35.0f,   5.5f }  // Climb
};

// Initialise AHRS, Airframe, and the Estimators
SoaringController::SoaringController(AP_TECS &tecs, const AP_FixedWing &parms) : 
    _tecs(tecs),
	_ahrs(AP::ahrs()),
    _aparm(parms),
    _vario(parms, _polar_params), // Variometer needs airframe params for polar
	_last_action{} 
{
    AP_Param::setup_object_defaults(this, var_info);
    
	_heatmap_data = nullptr;
	
    // Initialise thermal memory array
    for (uint8_t i = 0; i < MAX_THERMALS; i++) {
        _thermal_memory[i].active = false;
    }
		
}

// function to load data from file - currently will only load one map - potential to refresh in flight in future (with rate limit)
void SoaringController::load_heatmap() {
    if (_heatmap_data != nullptr) return;

    // I/O rate limit
    static uint32_t last_attempt_ms = 0;
    uint32_t now = AP_HAL::millis();
    if (last_attempt_ms != 0 && (now - last_attempt_ms) < 5000) {
        return;
    }
    last_attempt_ms = now;

    const char* fname = "/fs/microsd/APM/glide.bin";
    FILE *f = fopen(fname, "rb");
    if (!f) f = fopen("glide.bin", "rb"); // SITL fallback
    if (!f) f = fopen("libraries/AP_Soaring/tests/glide.bin", "rb"); // Unit test fallback

    if (!f) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GLIDE: No glide.bin found! Retrying in 5s...");
        return;
    }

    HeatmapHeader header;
    if (fread(&header, 1, sizeof(header), f) != sizeof(header)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: File too short");
        fclose(f);
        return;
    }

    if (header.magic != 0x47503533) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Invalid Magic Number");
        fclose(f);
        return;
    }

    _grid_width = header.grid_width;
    _grid_height = header.grid_height;
    _grid_resolution_m = header.resolution_m;
    _heatmap_origin.lat = header.origin_lat_e7;
    _heatmap_origin.lng = header.origin_lng_e7;
    _heatmap_origin.alt = 0;

    _mission_alt_min = header.floor_alt;
    _mission_alt_max = header.ceiling_alt;
    _mission_batt_mah = header.start_battery_mah;
    _mission_time_s = header.mission_time_s;
    _mission_rth_enable = (header.reserve_rth > 0);

    // 2 cells per byte -> Size = (W * H) / 2
    uint32_t payload_size = (_grid_width * _grid_height) / 2;
    _heatmap_data = (uint8_t *)malloc(payload_size);
    
    if (_heatmap_data != nullptr) {
        if (fread(_heatmap_data, 1, payload_size, f) == payload_size) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GLIDE: Mission Loaded");
        } else {
            free(_heatmap_data);
            _heatmap_data = nullptr;
        }
    }
    fclose(f);
}

// Get A/C state information - decouple hardware to allow unit testing of soaring controller
SoaringController::VehicleState SoaringController::get_current_state() {
    VehicleState state;
    
    _ahrs.get_relative_position_D_home(state.alt_m);
    state.alt_m *= -1.0f;
    
    if (!_ahrs.airspeed_TAS(state.tas_m_s) || state.tas_m_s < _aparm.airspeed_min) {
        state.tas_m_s = _aparm.airspeed_cruise; 
    }
    
	const Matrix3f &rot = AP::ahrs().get_rotation_body_to_ned();
	Vector3f forward = rot * Vector3f(1,0,0);
    state.heading_true_rad = atan2f(forward.y, forward.x);
    state.wind = _ahrs.wind_estimate();
    _ahrs.get_location(state.current_loc);
	
	float consumed_mah = 0.0f;
	if (!AP::battery().consumed_mah(consumed_mah, 0)) { // batt instance 0
    consumed_mah = 0.0f; // TODO - this should be set to max capacity and as a failsafe?
	} 
	float total_capacity = AP::battery().pack_capacity_mah(0);
	state.battery_remaining_mah = MAX(0.0f, total_capacity - consumed_mah);
	
    state.time_ms = AP_HAL::millis();
    state.is_armed = hal.util->get_soft_armed();
    state.cruise_spd_m_s = MAX(_aparm.airspeed_cruise, 5.0f);
    
    Location home = _ahrs.get_home();
    state.dist_to_home_m = state.current_loc.get_distance(home);
    
    return state;
}

// Main Soaring Loop - call at 20Hz. This triggers the: vehicle state fetch, strategic loop, thermal update, tactical loop
void SoaringController::update() {
    if (!soar_enable) return;
	
	auto check_err = [](const char* location) {
        static uint32_t last_err_count = 0;
        uint32_t err_count = AP::internalerror().count();
        if (err_count != last_err_count) {
            last_err_count = err_count;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: fault after %s", location);
        }
    };
	
	static uint32_t last_err_count = 0;
    uint32_t err_count = AP::internalerror().count();
    if (err_count != last_err_count) {
        last_err_count = err_count;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Internal error at line %u",
                      (unsigned)AP::internalerror().last_error_line());
    }	
	
    VehicleState state = get_current_state();
	check_err("get_current_state");

    // Strategic Loop
    if (state.time_ms - _last_strategic_update_ms >= 1000) {
        update_strategic_loop(state);
		check_err("update_strategic_loop");
        _last_strategic_update_ms = state.time_ms;
    }

    // Tactical Loop
    if (state.time_ms - _last_tactical_update_ms >= 50) {
        float dt = (state.time_ms - _last_tactical_update_ms) * 0.001f;
		update_thermals(state, dt);
		check_err("update_thermals");
        
        SoaringAction action = calculate_optimal_action(state);
		check_err("calculate_optimal_action");
		
		log_state_to_csv(state, action);

        if (action.is_valid) {
            //Log_Write_Soaring(action);
            _last_action = action; 
        }
        _last_tactical_update_ms = state.time_ms;
    }
}

// Update thermal belief
void SoaringController::update_thermals(const VehicleState &state, float dt) {
	
	auto check_err = [](const char* location) {
        static uint32_t last_err_count = 0;
        uint32_t err_count = AP::internalerror().count();
        if (err_count != last_err_count) {
            last_err_count = err_count;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: fault after %s", location);
        }
    };
	
	// Update Variometer
    _vario.update(_ahrs.get_roll());
    float airmass_rate = _vario.reading;
	check_err("vario.update");

    // Get North/East position relative to home
    Vector2f pos;
    if (!_ahrs.get_relative_position_NE_home(pos)) {
        return; 
    }
	
	check_err("get_position");

    // Update EKF
    // thermal drift: wind_velocity * dt
    float dist_to_thermal = sqrtf(powf(pos.x - _ekf.X[2], 2) + powf(pos.y - _ekf.X[3], 2));
    float thermal_radius = _ekf.X[1];
	
	if (dist_to_thermal > 0.01f && thermal_radius > 0.01f) {
               
        _ekf.update(airmass_rate, pos.x, pos.y, state.wind.x * dt, state.wind.y * dt);
		check_err("ekf.update");
    }
	
    if (isnan(_ekf.X[0]) || isnan(_ekf.X[1]) || isnan(_ekf.X[2]) || isnan(_ekf.X[3]) ||
        isinf(_ekf.X[0]) || isinf(_ekf.X[1]) || isinf(_ekf.X[2]) || isinf(_ekf.X[3])) {
        // EKF has diverged - skip this update
        return;
    }

    // sanity check the radius is physically plausible
    if (_ekf.X[1] < 0.1f || _ekf.X[1] > 2000.0f) {
        return;
    }
	
    if (_ekf.X[0] > 0.5f) { // X[0] is Strength (W)
        float north = constrain_float(_ekf.X[2], -50000.0f, 50000.0f);
		float east  = constrain_float(_ekf.X[3], -50000.0f, 50000.0f); // X[2] is North Pos, X[3] is East Pos
	
        Location thermal_loc = state.current_loc;
        
        thermal_loc.offset(north, east);

        int8_t target_idx = -1;
        float min_dist = FLT_MAX;
        
        for (uint8_t i = 0; i < MAX_THERMALS; i++) {
            if (!_thermal_memory[i].active) {
                if (target_idx == -1) target_idx = i;
                continue;
            }
            
            float dist = thermal_loc.get_distance(_thermal_memory[i].center_loc);
            if (dist < 50.0f && dist < min_dist) {
                min_dist = dist;
                target_idx = i;
            }
        }

        if (target_idx != -1) {
            _thermal_memory[target_idx].center_loc = thermal_loc;
            _thermal_memory[target_idx].strength_w0 = _ekf.X[0];
            _thermal_memory[target_idx].radius_r0 = _ekf.X[1]; // X[1] is Radius
            _thermal_memory[target_idx].last_update_ms = state.time_ms;
            _thermal_memory[target_idx].active = true;
        }
    }
}


// STRATEGIC LOOP
void SoaringController::update_strategic_loop(const VehicleState &state) {
    if (_start_time_ms == 0 && state.is_armed) {
        _start_time_ms = state.time_ms;
        _start_batt_cap = state.battery_remaining_mah;
        _budget_slope = _start_batt_cap / MAX((float)_mission_time_s, 1.0f); 
        _lambda_lagrange = 0.5f; 
    }
    if (!state.is_armed) return;

    float t_elapsed = (state.time_ms - _start_time_ms) / 1000.0f;
    float mah_ideal = t_elapsed * _budget_slope;
    float mah_burned = _mission_batt_mah - state.battery_remaining_mah;
    float mah_virtual = mah_burned;

    if (_mission_rth_enable) {
        float mah_rth = (state.dist_to_home_m / state.cruise_spd_m_s) * (12.0f / 3600.0f) * 1000.0f * 1.2f;
        mah_virtual += mah_rth;
    }

    float error = mah_virtual - mah_ideal;
    _lambda_lagrange += (cmdp_alpha.get() * error);
    _lambda_lagrange = constrain_float(_lambda_lagrange, 0.0f, 1.0f);
	
	// mission cell depletion
    bool grid_updated = false;
    
    if (_heatmap_data != nullptr && state.alt_m > 0) {
        
        // swath calc
        float hfov_rad = radians(soar_cam_hfov.get());
        float vfov_rad = radians(soar_cam_vfov.get());
        float swath_width_m = 2.0f * state.alt_m * tanf(hfov_rad * 0.5f);  // cross track
        float swath_height_m = 2.0f * state.alt_m * tanf(vfov_rad * 0.5f); // along track
        
        // Need whole cell within swath - max distance from a to corner is (res * sqrt(2) / 2)
        float cell_margin = _grid_resolution_m * 0.7071f; 
        float max_local_x = (swath_height_m * 0.5f) - cell_margin; 
        float max_local_y = (swath_width_m * 0.5f) - cell_margin;
        
        if (max_local_x > 0 && max_local_y > 0) { // sanity check swath larger than cell - eg. not on the ground
            
            int16_t plane_cx, plane_cy;
            if (get_grid_coords_from_loc(state.current_loc, plane_cx, plane_cy)) {
                
                // max grid area we need to check whether falls in swath
                float max_swath_dim = MAX(swath_width_m, swath_height_m);
                int16_t cell_rad = ceilf((max_swath_dim * 0.5f) / _grid_resolution_m);
                
                float cos_heading = cosf(state.heading_true_rad);
                float sin_heading = sinf(state.heading_true_rad);
                float dep_factor = constrain_float(soar_dep_fact.get(), 0.0f, 1.0f);
                
                for (int16_t y = plane_cy - cell_rad; y <= plane_cy + cell_rad; y++) {
                    for (int16_t x = plane_cx - cell_rad; x <= plane_cx + cell_rad; x++) {
                        
                        if (x >= 0 && x < _grid_width && y >= 0 && y < _grid_height) {
                            
                            float dx_m = (x - plane_cx) * _grid_resolution_m; // dist from plane to cell center
                            float dy_m = (y - plane_cy) * _grid_resolution_m;
                            
                            // aircraft local frame x y
                            float local_x = (dy_m * cos_heading) + (dx_m * sin_heading);
                            float local_y = (dx_m * cos_heading) - (dy_m * sin_heading);
                            
                            // check cell center within margins
                            if (fabsf(local_x) <= max_local_x && fabsf(local_y) <= max_local_y) {
                                
                                uint32_t cell_index = y * _grid_width + x;
                                uint32_t byte_index = cell_index / 2;
                                uint8_t raw_byte = _heatmap_data[byte_index];
                                
                                bool is_even = (cell_index % 2 == 0);
                                uint8_t val = is_even ? (raw_byte >> 4) : (raw_byte & 0x0F);
                                
                                if (val > 0) {
                                    uint8_t new_val = (uint8_t)(val * dep_factor);
                                    
                                    // Only write value changes
                                    if (new_val != val) {
                                        if (is_even) {
                                            _heatmap_data[byte_index] = (raw_byte & 0x0F) | (new_val << 4);
                                        } else {
                                            _heatmap_data[byte_index] = (raw_byte & 0xF0) | (new_val & 0x0F);
                                        }
                                        grid_updated = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Dump new grid to SD if changes made
    if (grid_updated) {
        save_heatmap();
    }
	
	//
	//
	// locate nearest target for global pull
	_has_nearest_target = false;
	
    if (_heatmap_data != nullptr) {
        int16_t curr_x, curr_y;
        if (get_grid_coords_from_loc(state.current_loc, curr_x, curr_y)) {
            
            float max_attraction = -1.0f;
            int16_t nearest_x = -1;
            int16_t nearest_y = -1;
            uint8_t nearest_val = 0;
            
            const int16_t stride = 3; 
            
            for (int16_t y = 0; y < _grid_height; y += stride) {
                for (int16_t x = 0; x < _grid_width; x += stride) {
                    uint32_t cell_index = y * _grid_width + x;
                    uint32_t byte_index = cell_index / 2;
                    uint8_t raw_byte = _heatmap_data[byte_index];

                    uint8_t val = (cell_index % 2 == 0) ? (raw_byte >> 4) : (raw_byte & 0x0F);
                    
                    if (val > 0) {
                        int32_t dx = x - curr_x;
                        int32_t dy = y - curr_y;
                        int32_t dist_sq = (dx * dx) + (dy * dy); 
                        
                        // Prevent division by zero
                        if (dist_sq == 0) dist_sq = 1;
                        // Priority Score divided by Squared Distance
                        float attraction = (float)val / (float)dist_sq;
                        
                        if (attraction > max_attraction) {
                            max_attraction = attraction;
                            nearest_x = x;
                            nearest_y = y;
                            nearest_val = val;
                        }
                    }
                }
            }
			
            if (nearest_x != -1) {
                _nearest_target = _heatmap_origin;
                // Offset: Y is North, X is East
                _nearest_target.offset(nearest_y * _grid_resolution_m, nearest_x * _grid_resolution_m);
				_nearest_target_score = nearest_val;
                _has_nearest_target = true;
            }
        }
    }
}


// TACTICAL LOOP
// calculate reward function for each action and select best action
SoaringController::SoaringAction SoaringController::calculate_optimal_action(const VehicleState &state) {
    SoaringAction best_action = {};
	best_action.score_total = -FLT_MAX;
	best_action.is_valid = false;
    
    const float k_safe = 50.0f;
    const float k_greed = 5.0f;
    const float epsilon = 0.05f;
    const float buffer = 5.0f;


    for (int bank = -45; bank <= 45; bank += 5) {
        
        Location pred_loc_2d = predict_position_future(state, (float)bank, 5.0f);
        
        float thermal_lift = predict_thermal_lift(state, pred_loc_2d);
        float mission_density = get_local_density_score(pred_loc_2d);
		
		// global pull if no nearby mission density
		if (mission_density < 0.01f && _has_nearest_target) {
            float current_dist = state.current_loc.get_distance(_nearest_target);
            float pred_dist = pred_loc_2d.get_distance(_nearest_target);
            float safe_current_dist = MAX(current_dist, 10.0f);
            mission_density = _nearest_target_score / 15.0f * (current_dist - pred_dist) / safe_current_dist;
        }

        float bank_rad = radians(bank);
        float load_sq = 1.0f / (cosf(bank_rad) * cosf(bank_rad));
        if (load_sq > 2.0f) load_sq = 2.0f; 

        for (const auto& step : _perf_table) {
            
            float sink_penalty = (step.throttle_pct == 0) ? load_sq : 1.0f;
            float net_climb = (step.vz_still_air * sink_penalty) + thermal_lift;
            float pred_alt = state.alt_m + (net_climb * 5.0f);

            float r_mis = 0.0f;
            if (pred_alt > 0) {
                //float gsd_factor = MIN(1.0f, pred_alt / max_gsd_alt.get());
                r_mis = (1.0f - _lambda_lagrange) * mission_density * 1 * soar_beta.get();
            }

            float r_greed = 0.0f;
            float glide_sink = _perf_table[0].vz_still_air * load_sq;
            if (step.throttle_pct > 0 && (thermal_lift + glide_sink) > 0.5f) {
                r_greed = step.power_amps * k_greed;
            }
            float cost = step.power_amps + r_greed;
            float r_eng = (_lambda_lagrange + epsilon) * (net_climb - cost);

            float r_safe = 0.0f;
			if (pred_alt < (_mission_alt_min + buffer)) {
                float diff = (_mission_alt_min + buffer) - pred_alt;
                r_safe -= k_safe * diff * diff;
            }
            if (pred_alt > _mission_alt_max) {
                float diff = pred_alt - _mission_alt_max;
                r_safe -= k_safe * diff * diff;
            }

            float total = r_mis + r_eng + r_safe;
			
			// general hysteresis factor +0.005 for doing the same thing again
			if (fabsf((float)bank - _last_action.bank_angle) < 0.5f && step.throttle_pct == _last_action.throttle_pct) {
                total += 0.001f;
            }
			
			// Motor start penalty
            if (_last_action.throttle_pct == 0 && step.throttle_pct > 0) {
                total -= 0.5f;
            }

            if (total > best_action.score_total) {
                best_action.bank_angle = (float)bank;
                best_action.throttle_pct = step.throttle_pct;
                best_action.score_total = total;
                best_action.score_mission = r_mis;
                best_action.score_energy = r_eng;
                best_action.score_safety = r_safe;
                best_action.current_lambda = _lambda_lagrange;
                best_action.is_valid = true;
            }
        }
    }

    
    _last_action = best_action;
    _last_best_score = best_action.score_total;
    return best_action;
}

// PHYSICS FUNCTIONS
Location SoaringController::predict_position_future(const VehicleState &state, float bank_angle, float dt) {
    Location loc = state.current_loc;

    float bank_rad = radians(constrain_float(bank_angle, -60, 60));
    float rate = (9.81f * tanf(bank_rad)) / MAX(state.tas_m_s, 1.0f); // no div by zero

    float avg_heading = state.heading_true_rad + (rate * dt * 0.5f);

    float vn = (state.tas_m_s * cosf(avg_heading)) + state.wind.x;
    float ve = (state.tas_m_s * sinf(avg_heading)) + state.wind.y;

    loc.offset(vn * dt, ve * dt);
    return loc;
}


float SoaringController::predict_thermal_lift(const VehicleState &state, const Location &pred_loc) {
    float max_lift = 0.0f;
    
    for (uint8_t i = 0; i < MAX_THERMALS; i++) {
        if (!_thermal_memory[i].active) continue;
        
		if (_thermal_memory[i].radius_r0 < 0.1f || 
        isnan(_thermal_memory[i].strength_w0) ||
        isnan(_thermal_memory[i].radius_r0)) {
        _thermal_memory[i].active = false;
        continue;
		}
		
        // Age out thermals older than 2 minutes using injection time
        if (state.time_ms - _thermal_memory[i].last_update_ms > 120000) {
            _thermal_memory[i].active = false;
            continue;
        }

        float dist = pred_loc.get_distance(_thermal_memory[i].center_loc);
        float r0 = _thermal_memory[i].radius_r0;
        
        if (dist > (r0 * 3.0f)) continue; 

        float r0_sq = MAX(r0 * r0, 1.0f);
        float exponent = -1.0f * ((dist * dist) / r0_sq);
        float lift = _thermal_memory[i].strength_w0 * expf(exponent);

        if (lift > max_lift) max_lift = lift;
    }
    return max_lift;
}

// HEATMAP FUNCTIONS
float SoaringController::get_local_density_score(const Location &loc) {
    if (!_heatmap_data) return 0.0f;

    int16_t cx, cy;
    if (!get_grid_coords_from_loc(loc, cx, cy)) return 0.0f;

    const int16_t rad = soar_map_rad.get();
    const int16_t step = 2; 
    
    float total = 0.0f;
    int count = 0;

    for (int16_t dy = -rad; dy <= rad; dy += step) {
        for (int16_t dx = -rad; dx <= rad; dx += step) {
            int16_t x = cx + dx;
            int16_t y = cy + dy;
            
            if (x >= 0 && x < _grid_width && y >= 0 && y < _grid_height) {
                 // 4 bit unpack
                 uint32_t cell_index = y * _grid_width + x;
                 uint32_t byte_index = cell_index / 2;
                 uint8_t raw_byte = _heatmap_data[byte_index];
                 
                 // Extract Nibble - Even cell = High bits (7-4), Odd cell = Low bits (3-0)
                 uint8_t val = (cell_index % 2 == 0) ? (raw_byte >> 4) : (raw_byte & 0x0F);

                 total += (float)val;
                 count++;
            }
        }
    }

    if (count == 0) return 0.0f;
    return total / (float)count / 15.0f; // normalised score 0-1 instead of 0-15
}


bool SoaringController::get_grid_coords_from_loc(const Location &loc, int16_t &x, int16_t &y) {
    if (_heatmap_origin.is_zero()) return false;
	Vector2f off = _heatmap_origin.get_distance_NE(loc);
    x = (int16_t)(off.y / _grid_resolution_m); 
    y = (int16_t)(off.x / _grid_resolution_m); 
    return true;
}

// LOGGING
void SoaringController::Log_Write_Soaring(const SoaringAction &action) {
#if HAL_LOGGING_ENABLED
    AP::logger().Write(
        "SOAR",                         // message name (4 chars max)
        "Time_micros,Bank,Thr,Tot,Mis,Eng,Saf,Lam",  // field names
        "sdddddd-",                     // units
        "F-------",                     // multipliers
        "Qfbfffff",                     // format: Q=uint64, f=float, b=int8
        AP_HAL::micros64(),
        action.bank_angle,
        action.throttle_pct,
        action.score_total,
        action.score_mission,
        action.score_energy,
        action.score_safety,
        action.current_lambda
    );
#endif

#if HAL_GCS_ENABLED
    static uint32_t last_gcs_ms = 0;
    uint32_t now = AP_HAL::millis();
    if (now - last_gcs_ms >= 1000) {
        last_gcs_ms = now;
        gcs().send_named_float("SOAR_TOT", action.score_total);
        gcs().send_named_float("SOAR_MIS", action.score_mission);
        gcs().send_named_float("SOAR_ENG", action.score_energy);
        gcs().send_named_float("SOAR_SAF", action.score_safety);
        gcs().send_named_float("SOAR_LAM", action.current_lambda);
    }
#endif
}

void SoaringController::save_heatmap() {
    if (_heatmap_data == nullptr) return;

    const char* fname = "/fs/microsd/APM/glide_live.bin";
    FILE *f = fopen(fname, "wb");
    if (!f) f = fopen("glide_live.bin", "wb"); // SITL fallback
    if (!f) f = fopen("libraries/AP_Soaring/tests/glide_live.bin", "wb"); // Unit test fallback

    if (!f) return;

    // header
    HeatmapHeader header;
    header.magic = 0x47503533;
    header.origin_lat_e7 = _heatmap_origin.lat;
    header.origin_lng_e7 = _heatmap_origin.lng;
    header.grid_width = _grid_width;
    header.grid_height = _grid_height;
    header.resolution_m = _grid_resolution_m;
    header.floor_alt = _mission_alt_min;
    header.ceiling_alt = _mission_alt_max;
    header.start_battery_mah = _mission_batt_mah;
    header.mission_time_s = _mission_time_s;
    header.reserve_rth = _mission_rth_enable ? 1 : 0;
    fwrite(&header, 1, sizeof(header), f);
    
    // modified grid
    uint32_t payload_size = (_grid_width * _grid_height) / 2;
    fwrite(_heatmap_data, 1, payload_size, f);
    
    fclose(f);
}

void SoaringController::log_state_to_csv(const VehicleState &state, const SoaringAction &action) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static FILE *csv_file = nullptr;
    static bool header_written = false;

    if (csv_file == nullptr) {
        csv_file = fopen("libraries/AP_Soaring/flight_data.csv", "w");
        if (csv_file == nullptr) return;
    }

    if (!header_written) {
        fprintf(csv_file, "Time(ms),lat_e7,lng_e7,alt_m,tas_m_s,heading_deg,Lambda,Bank(deg),Thr(%%),TotScore,MisScore,EngScore,SafScore\n");
        header_written = true;
    }

    fprintf(csv_file, "%u,%d,%d,%.3f,%.3f,%.3f,%.4f,%.1f,%d,%.4f,%.4f,%.4f,%.4f\n",
            (unsigned)state.time_ms,
            (int)state.current_loc.lat,
            (int)state.current_loc.lng,
            (double)state.alt_m,
            (double)state.tas_m_s,
            (double)degrees(state.heading_true_rad),
            (double)action.current_lambda,
            (double)action.bank_angle,
            (int)action.throttle_pct,
            (double)action.score_total,
            (double)action.score_mission,
            (double)action.score_energy,
            (double)action.score_safety);

    fflush(csv_file);
#endif
}
//Pass roll command and throttle pct to autopilot
float SoaringController::get_target_bank_angle_cd() const {
    if (!_last_action.is_valid) {
        return 0.0f; // Fly straight if we don't have a valid action yet
    }
    // Convert degrees to centi-degrees for the Attitude Controller
    return _last_action.bank_angle * 100.0f; 
}

int8_t SoaringController::get_target_throttle_pct() const {
    if (!_last_action.is_valid) return 0;
    return _last_action.throttle_pct;
}

// FAILSAFE
bool SoaringController::is_healthy() {
	
	if (_heatmap_data == nullptr) {
        load_heatmap();
    }
    if (_heatmap_data == nullptr) return false;
    if (_mission_time_s == 0) return false; 
    if (max_gsd_alt.get() <= 0.0f) return false; 
    if (_grid_resolution_m <= 0.0f) return false; 
    if (soar_map_rad.get() < 0) return false; 
    if (_grid_width == 0 || _grid_height == 0) return false;
    if (_heatmap_origin.is_zero()) return false; 
    if (_mission_alt_min >= _mission_alt_max) return false; 
    if (_mission_batt_mah <= 0.0f) return false;
    if (!_ahrs.healthy()) return false;

    return true;
}

uint8_t SoaringController::get_failsafe_action() const {
    return (uint8_t)soar_fs_action.get();
}

// interfaces from old soaring code expected by other libraries
void SoaringController::update_active_state(bool override_disable) { 
    if (override_disable) {
        // TODO - override to Failsafe?
        return;
    }
}

void SoaringController::init_cruising() {
    // Reset timers so the controller doesn't use stale data
    _last_tactical_update_ms = AP_HAL::millis();
    _last_best_score = -FLT_MAX;
}

bool SoaringController::get_throttle_suppressed() const {
    return (soar_enable > 0 && _last_action.throttle_pct == 0);
}

float SoaringController::get_thermalling_target_airspeed() const {
    if (soar_v_glide.get() < 0.1f) {
        return _aparm.airspeed_cruise;
    }
    return constrain_float(soar_v_glide.get(), _aparm.airspeed_min, _aparm.airspeed_max);
}

float SoaringController::get_cruising_target_airspeed() const {
    if (soar_v_glide.get() < 0.1f) {
        return _aparm.airspeed_cruise;
    }
    return constrain_float(soar_v_glide.get(), _aparm.airspeed_min, _aparm.airspeed_max);
}

float SoaringController::get_alt_cutoff() const {
    return _mission_alt_max;
}

#endif // HAL_SOARING_ENABLED
