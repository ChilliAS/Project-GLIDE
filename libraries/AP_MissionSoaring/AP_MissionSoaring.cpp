#include "AP_MissionSoaring.h"

#if HAL_MISSIONSOARING_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <fcntl.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

static const uint8_t LOG_SOAR_REWARD_MSG = 201;

extern const AP_HAL::HAL& hal;

// INITIALISATION
const AP_Param::GroupInfo MSoaringController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Mission Soaring
    // @Description: Enables the Project Glide Mission Soaring controller
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, MSoaringController, msoar_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ALPHA
    // @DisplayName: Energy Cost Learning Rate
    // @Description: How fast Lambda reacts to budget deviation.
    AP_GROUPINFO("ALPHA", 2, MSoaringController, msoar_alpha, 0.001f),

    // @Param: BETA
    // @DisplayName: Mission Reward Scaler
    // @Description: Multiplier to balance mission score vs Amps.
    AP_GROUPINFO("BETA", 3, MSoaringController, msoar_beta, 10.0f),

    // @Param: MISSION_SEARCH_RADIUS
    // @DisplayName: mission Search Radius
    // @Description: Radius in grid cells (strided) to scan for density.
    AP_GROUPINFO("MISSION_SEARCH_RADIUS", 4, MSoaringController, msoar_mis_search_rad, 4),
    
    // @Param: TGT_ALT
    // @DisplayName: Optimal Altitude
    // @Description: Altitude in meters for maximum sensor score (1.0 factor).
    // @Range: 10 500
    AP_GROUPINFO("TGT_ALT", 5, MSoaringController, tgt_alt, 100.0f),
    
    // @Param: POLAR_K
    // @DisplayName: Polar Drag K
    // @Description: Induced drag coefficient for the airframe.
    AP_GROUPINFO("POLAR_K", 6, MSoaringController, _polar_params.K, 25.6f),

    // @Param: POLAR_CD0
    // @DisplayName: Polar Drag CD0
    // @Description: Parasitic drag coefficient for the airframe.
    AP_GROUPINFO("POLAR_CD0", 7, MSoaringController, _polar_params.CD0, 0.027f),

    // @Param: POLAR_B
    // @DisplayName: Polar Drag B
    // @Description: Sink rate base coefficient.
    AP_GROUPINFO("POLAR_B", 8, MSoaringController, _polar_params.B, 0.036f),
    
    // @Param: V_GLIDE
    // @DisplayName: Soaring Glide Airspeed
    // @Description: Target airspeed to maintain while gliding or thermalling.
    // @Units: m/s
    // @Range: 8 30
    // @Increment: 0.5
    AP_GROUPINFO("V_GLIDE", 9, MSoaringController, msoar_v_glide, 12.0f),
    
    // @Param: SOAR_FS_ACT
    // @DisplayName: Soaring Failsafe Action
    // @Description: Mode to switch to if soaring controller becomes unhealthy. 0:RTL, 1:FBWA.
    // @Values: 0:RTL, 1:FBWA
    AP_GROUPINFO("SOAR_FS_ACT", 10, MSoaringController, msoar_fs_action, 1),
    
    // @Param: CAM_HFOV
    // @DisplayName: Camera Horizontal FOV
    // @Description: Horizontal FOV in degrees (across the flight path).
    // @Range: 10 150
    AP_GROUPINFO("CAM_HFOV", 11, MSoaringController, msoar_cam_hfov, 102.0f),

    // @Param: CAM_VFOV
    // @DisplayName: Camera Vertical FOV
    // @Description: Vertical FOV in degrees (along the flight path).
    // @Range: 10 150
    AP_GROUPINFO("CAM_VFOV", 12, MSoaringController, msoar_cam_vfov, 67.0f),

    // @Param: DEP_FACT
    // @DisplayName: Cell Depletion Factor
    // @Description: Multiplier applied to cell score per second when overflown (0.0 to 1.0).
    AP_GROUPINFO("DEP_FACT", 13, MSoaringController, msoar_dep_fact, 0.5f),

    AP_GROUPEND
};

const MSoaringController::PerformanceStep MSoaringController::perf_table[] = {
    { 0,   1.0f,  -0.8f }, // Gliding
    { 60, 12.0f,   1.5f }, // Cruise
    { 90, 35.0f,   5.5f }  // Climb
};

// Initialise AHRS, Airframe, and the Estimators
MSoaringController::MSoaringController(AP_TECS &tecs, const AP_FixedWing &parms) : 
    _tecs(tecs),
    _ahrs(AP::ahrs()),
    _aparm(parms),
    _vario(parms, _polar_params), // Variometer needs airframe params for polar
    last_action{} 
{
    AP_Param::setup_object_defaults(this, var_info);
    
    // Allocate memory in a DMA-safe region
    mission_heatmap = (uint8_t*)hal.util->malloc_type(MAX_HEATMAP_BYTES, AP_HAL::Util::MEM_DMA_SAFE);
    
    // Initialise thermal memory array
    for (uint8_t i = 0; i < MAX_THERMALS; i++) {
        thermal_memory[i].active = false;
    }
    
    init_io_thread();
        
}


// PRIMARY LOOPS (run by AP scheduler)
void MSoaringController::update_strategic_loop() {
    VehicleState state = get_current_state();
    if (!msoar_enable || !state.is_armed) return;
    
    

    if (mission_start_time_us == 0 && state.is_armed) {
        mission_start_time_us = state.time_us;
        start_batt_mah = state.battery_remaining_mah;
        budget_slope = start_batt_mah / MAX((float)mis_time_s, 1.0f); 
        lambda_lagrange = 0.5f; 
    }

    float t_elapsed = (state.time_us - mission_start_time_us) * 1.0e-6f;
    float mah_ideal = t_elapsed * budget_slope;
    float mah_virtual = mis_batt_mah - state.battery_remaining_mah;

    if (mis_rth_enable) {
        float mah_rth = (state.dist_to_home_m / state.cruise_spd_m_s) * (12.0f / 3600.0f) * 1000.0f * 1.2f;
        mah_virtual += mah_rth;
    }

    float error = mah_virtual - mah_ideal;
    lambda_lagrange += (msoar_alpha.get() * error);
    lambda_lagrange = constrain_float(lambda_lagrange, 0.0f, 1.0f);
    
    // mission cell depletion
    bool grid_updated = false;
    
    if (mission_sem.take_nonblocking()) {
        if (mission_heatmap != nullptr && state.alt_m > 0) {
            
            // swath calc
            float hfov_rad = radians(msoar_cam_hfov.get());
            float vfov_rad = radians(msoar_cam_vfov.get());
            float swath_width_m = 2.0f * state.alt_m * tanf(hfov_rad * 0.5f);  // cross track
            float swath_height_m = 2.0f * state.alt_m * tanf(vfov_rad * 0.5f); // along track
            
            // Need whole cell within swath - max distance from a to corner is (res * sqrt(2) / 2)
            float cell_margin = grid_res_m * 0.7071f; 
            float max_local_x = (swath_height_m * 0.5f) - cell_margin; 
            float max_local_y = (swath_width_m * 0.5f) - cell_margin;
            
            if (max_local_x > 0 && max_local_y > 0) { // sanity check swath larger than cell - eg. not on the ground
                
                int16_t plane_cx, plane_cy;
                if (get_grid_coords_from_loc(state.current_loc, plane_cx, plane_cy)) {
                    
                    // max grid area we need to check whether falls in swath
                    float max_swath_dim = MAX(swath_width_m, swath_height_m);
                    int16_t cell_rad = ceilf((max_swath_dim * 0.5f) / grid_res_m);
                    
                    float cos_heading = cosf(state.heading_true_rad);
                    float sin_heading = sinf(state.heading_true_rad);
                    float dep_factor = constrain_float(msoar_dep_fact.get(), 0.0f, 1.0f);
                    
                    for (int16_t y = plane_cy - cell_rad; y <= plane_cy + cell_rad; y++) {
                        for (int16_t x = plane_cx - cell_rad; x <= plane_cx + cell_rad; x++) {
                            
                            if (x >= 0 && x < grid_width_m && y >= 0 && y < grid_height_m) {
                                
                                float dx_m = (x - plane_cx) * grid_res_m; // dist from plane to cell center
                                float dy_m = (y - plane_cy) * grid_res_m;
                                
                                // aircraft local frame x y
                                float local_x = (dy_m * cos_heading) + (dx_m * sin_heading);
                                float local_y = (dx_m * cos_heading) - (dy_m * sin_heading);
                                
                                // check cell center within margins
                                if (fabsf(local_x) <= max_local_x && fabsf(local_y) <= max_local_y) {
                                    
                                    uint32_t cell_index = y * grid_width_m + x;
                                    uint32_t byte_index = cell_index / 2;
                                    uint8_t raw_byte = mission_heatmap[byte_index];
                                    
                                    bool is_even = (cell_index % 2 == 0);
                                    uint8_t val = is_even ? (raw_byte >> 4) : (raw_byte & 0x0F);
                                    
                                    if (val > 0) {
                                        uint8_t new_val = (uint8_t)(val * dep_factor);
                                        
                                        // Only write value changes
                                        if (new_val != val) {
                                            if (is_even) {
                                                mission_heatmap[byte_index] = (raw_byte & 0x0F) | (new_val << 4);
                                            } else {
                                                mission_heatmap[byte_index] = (raw_byte & 0xF0) | (new_val & 0x0F);
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
            mission_map_changed = true;
        }
    
        // locate nearest target for global pull
        has_nearest_target = false;
        
        if (mission_heatmap != nullptr) {
            int16_t curr_x, curr_y;
            if (get_grid_coords_from_loc(state.current_loc, curr_x, curr_y)) {
                
                float max_attraction = -1.0f;
                int16_t nearest_x = -1;
                int16_t nearest_y = -1;
                uint8_t nearest_val = 0;
                
                const int16_t stride = 3; 
                
                for (int16_t y = 0; y < grid_height_m; y += stride) {
                    for (int16_t x = 0; x < grid_width_m; x += stride) {
                        uint32_t cell_index = y * grid_width_m + x;
                        uint32_t byte_index = cell_index / 2;
                        uint8_t raw_byte = mission_heatmap[byte_index];

                        uint8_t val = (cell_index % 2 == 0) ? (raw_byte >> 4) : (raw_byte & 0x0F);
                        
                        if (val > 0) {
                            int32_t dx = x - curr_x;
                            int32_t dy = y - curr_y;
                            int32_t dist_sq = sq(dx) + sq(dy);
                            
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
                    nearest_target = heatmap_origin;
                    // Offset: Y is North, X is East
                    nearest_target.offset(nearest_y * grid_res_m, nearest_x * grid_res_m);
                    nearest_target_score = nearest_val;
                    has_nearest_target = true;
                }
            }
        }
        mission_sem.give();
    }
    
}

void MSoaringController::update_tactical_loop() {
    if (!msoar_enable) return;
    
    VehicleState state = get_current_state(); 
    
    if (last_tactical_update_us == 0) {
        last_tactical_update_us = state.time_us;
        return; 
    }
    float dt = (state.time_us - last_tactical_update_us) * 1.0e-6f;
        
    update_thermals(state, dt);
    SoaringAction action = calculate_optimal_action(state);
    log_state_to_csv(state, action);

    if (action.is_valid) {
            //Log_Write_Soaring(action);
        last_action = action; 
    }
    
        last_tactical_update_us = state.time_us;
    

}


// POMDP Functions (inc input fetch)
// Get A/C state information - decouple hardware to allow unit testing of soaring controller
MSoaringController::VehicleState MSoaringController::get_current_state() {
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
    state.battery_remaining_mah = MAX(0.0f, AP::battery().pack_capacity_mah(0) - consumed_mah);
    
    state.time_us = AP_HAL::micros();
    state.is_armed = hal.util->get_soft_armed();
    state.cruise_spd_m_s = MAX(_aparm.airspeed_cruise, 5.0f);
    
    if (_ahrs.home_is_set()) {
    state.dist_to_home_m = state.current_loc.get_distance(_ahrs.get_home());
    } else {
    state.dist_to_home_m = 0.0f;
    }
    
    return state;
}

// Update thermal belief
void MSoaringController::update_thermals(const VehicleState &state, float dt) {
    
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
    float dist_to_thermal = pos.get_distance(Vector2f(_ekf.X[2], _ekf.X[3]));
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
            if (!thermal_memory[i].active) {
                if (target_idx == -1) target_idx = i;
                continue;
            }
            
            float dist = thermal_loc.get_distance(thermal_memory[i].center_loc);
            if (dist < 50.0f && dist < min_dist) {
                min_dist = dist;
                target_idx = i;
            }
        }

        if (target_idx != -1) {
            thermal_memory[target_idx].center_loc = thermal_loc;
            thermal_memory[target_idx].strength_w0 = _ekf.X[0];
            thermal_memory[target_idx].radius_r0 = _ekf.X[1]; // X[1] is Radius
            thermal_memory[target_idx].last_update_ms = state.time_us;
            thermal_memory[target_idx].active = true;
        }
    }
}

// determine reward functions and select optimum action
MSoaringController::SoaringAction MSoaringController::calculate_optimal_action(const VehicleState &state) {
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
        if (mission_density < 0.01f && has_nearest_target) {
            float current_dist = state.current_loc.get_distance(nearest_target);
            float pred_dist = pred_loc_2d.get_distance(nearest_target);
            float safe_current_dist = MAX(current_dist, 10.0f);
            mission_density = nearest_target_score / 15.0f * (current_dist - pred_dist) / safe_current_dist;
        }

        float bank_rad = radians(bank);
        float load_sq = 1.0f / sq(cosf(bank_rad));
        if (load_sq > 2.0f) load_sq = 2.0f; 

        for (const auto& step : perf_table) {
            
            float sink_penalty = (step.throttle_pct == 0) ? load_sq : 1.0f;
            float net_climb = (step.vz_still_air * sink_penalty) + thermal_lift;
            float pred_alt = state.alt_m + (net_climb * 5.0f);

            float r_mis = 0.0f;
            if (pred_alt > 0) {
                //float tgt_alt_factor = MIN(1.0f, pred_alt / tgt_alt.get());
                r_mis = (1.0f - lambda_lagrange) * mission_density * 1 * msoar_beta.get();
            }

            float r_greed = 0.0f;
            float glide_sink = perf_table[0].vz_still_air * load_sq;
            if (step.throttle_pct > 0 && (thermal_lift + glide_sink) > 0.5f) {
                r_greed = step.power_amps * k_greed;
            }
            float cost = step.power_amps + r_greed;
            float r_eng = (lambda_lagrange + epsilon) * (net_climb - cost);

            float r_safe = 0.0f;
            if (pred_alt < (mis_alt_min_m + buffer)) {
                float diff = (mis_alt_min_m + buffer) - pred_alt;
                r_safe -= k_safe * sq(diff);
            }
            if (pred_alt > mis_alt_max_m) {
                float diff = pred_alt - mis_alt_max_m;
                r_safe -= k_safe * diff * diff;
            }

            float total = r_mis + r_eng + r_safe;
            
            // general hysteresis factor +0.005 for doing the same thing again
            if (fabsf((float)bank - last_action.bank_angle) < 0.5f && step.throttle_pct == last_action.throttle_pct) {
                total += 0.001f;
            }
            
            // Motor start penalty
            if (last_action.throttle_pct == 0 && step.throttle_pct > 0) {
                total -= 0.5f;
            }

            if (total > best_action.score_total) {
                best_action.bank_angle = (float)bank;
                best_action.throttle_pct = step.throttle_pct;
                best_action.score_total = total;
                best_action.score_mission = r_mis;
                best_action.score_energy = r_eng;
                best_action.score_safety = r_safe;
                best_action.current_lambda = lambda_lagrange;
                best_action.is_valid = true;
            }
        }
    }

    
    last_action = best_action;
    last_best_score = best_action.score_total;
    return best_action;
}


// PHYSICS FUNCTIONS
Location MSoaringController::predict_position_future(const VehicleState &state, float bank_angle, float dt) {
    Location loc = state.current_loc;

    float bank_rad = radians(constrain_float(bank_angle, -60, 60));
    float rate = (GRAVITY_MSS * tanf(bank_rad)) / MAX(state.tas_m_s, 1.0f); // no div by zero

    float avg_heading = state.heading_true_rad + (rate * dt * 0.5f);

    float vn = (state.tas_m_s * cosf(avg_heading)) + state.wind.x;
    float ve = (state.tas_m_s * sinf(avg_heading)) + state.wind.y;

    loc.offset(vn * dt, ve * dt);
    return loc;
}

float MSoaringController::predict_thermal_lift(const VehicleState &state, const Location &pred_loc) {
    float max_lift = 0.0f;
    
    for (uint8_t i = 0; i < MAX_THERMALS; i++) {
        if (!thermal_memory[i].active) continue;
        
        if (thermal_memory[i].radius_r0 < 0.1f || 
        isnan(thermal_memory[i].strength_w0) ||
        isnan(thermal_memory[i].radius_r0)) {
        thermal_memory[i].active = false;
        continue;
        }
        
        // Age out thermals older than 2 minutes using injection time
        if (state.time_us - thermal_memory[i].last_update_ms > 120000) {
            thermal_memory[i].active = false;
            continue;
        }

        float dist = pred_loc.get_distance(thermal_memory[i].center_loc);
        float r0 = thermal_memory[i].radius_r0;
        
        if (dist > (r0 * 3.0f)) continue; 

        float r0_sq = MAX(sq(r0), 1.0f);
        float exponent = -1.0f * (sq(dist) / r0_sq);
        float lift = thermal_memory[i].strength_w0 * expf(exponent);

        if (lift > max_lift) max_lift = lift;
    }
    return max_lift;
}


// MISSION MAP FUNCTIONS
float MSoaringController::get_local_density_score(const Location &loc) {
    if (!mission_heatmap) return 0.0f;
    if (!mission_sem.take_nonblocking()) return 0.0f;

    int16_t cx, cy;
    if (!get_grid_coords_from_loc(loc, cx, cy)) return 0.0f;

    const int16_t rad = msoar_mis_search_rad.get();
    const int16_t step = 2; 
    
    float total = 0.0f;
    int count = 0;

    for (int16_t dy = -rad; dy <= rad; dy += step) {
        for (int16_t dx = -rad; dx <= rad; dx += step) {
            int16_t x = cx + dx;
            int16_t y = cy + dy;
            
            if (x >= 0 && x < grid_width_m && y >= 0 && y < grid_height_m) {
                 // 4 bit unpack
                 uint32_t cell_index = y * grid_width_m + x;
                 uint32_t byte_index = cell_index / 2;
                 uint8_t raw_byte = mission_heatmap[byte_index];
                 
                 // Extract Nibble - Even cell = High bits (7-4), Odd cell = Low bits (3-0)
                 uint8_t val = (cell_index % 2 == 0) ? (raw_byte >> 4) : (raw_byte & 0x0F);

                 total += (float)val;
                 count++;
            }
        }
    }
    mission_sem.give();
    if (count == 0) return 0.0f;
    return total / (float)count / 15.0f; // normalised score 0-1 instead of 0-15
}

bool MSoaringController::get_grid_coords_from_loc(const Location &loc, int16_t &x, int16_t &y) {
    if (heatmap_origin.is_zero()) return false;
    Vector2f off = heatmap_origin.get_distance_NE(loc);
    x = (int16_t)(off.y / grid_res_m); 
    y = (int16_t)(off.x / grid_res_m); 
    return true;
}


// INTERFACES
bool MSoaringController::is_healthy() {
    
    if (!mission_loaded) return false;
    if (mission_heatmap == nullptr) return false;
    if (mis_time_s == 0) return false; 
    if (tgt_alt.get() <= 0.0f) return false; 
    if (grid_res_m <= 0.0f) return false; 
    if (msoar_mis_search_rad.get() < 0) return false; 
    if (grid_width_m == 0 || grid_height_m == 0) return false;
    if (heatmap_origin.is_zero()) return false; 
    if (mis_alt_min_m >= mis_alt_max_m) return false; 
    if (mis_batt_mah <= 0.0f) return false;
    if (!_ahrs.healthy()) return false;

    return true;
}

uint8_t MSoaringController::get_failsafe_action() const {
    return (uint8_t)msoar_fs_action.get();
}

float MSoaringController::get_thermalling_target_airspeed() const {
    if (msoar_v_glide.get() < 0.1f) {
        return _aparm.airspeed_cruise;
    }
    return constrain_float(msoar_v_glide.get(), _aparm.airspeed_min, _aparm.airspeed_max);
}

float MSoaringController::get_target_bank_angle_cd() const {
    if (!last_action.is_valid) {
        return 0.0f; // Fly straight if we don't have a valid action yet
    }
    // Convert degrees to centi-degrees for the Attitude Controller
    return last_action.bank_angle * 100.0f; 
}

int8_t MSoaringController::get_target_throttle_pct() const {
    if (!last_action.is_valid) return 0;
    return last_action.throttle_pct;
}


// FILE SYSTEM IO
void MSoaringController::init_io_thread() {
    if (io_thread_started) {
        return;
    }
    
    if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&MSoaringController::io_thread, void),
                                     "SoarIO", 
                                     2048, 
                                     AP_HAL::Scheduler::PRIORITY_IO, 
                                     0)) {
        io_thread_started = true;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Failed to start IO thread");
    }
}

void MSoaringController::io_thread() {
    while (true) {
        hal.scheduler->delay(1000);
        
        if (mission_load_requested && !mission_loaded) {
            if (mission_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                load_mission();
                mission_load_requested = false;
                mission_sem.give();
            }
        }
        
        if (mission_map_changed) {
            if (mission_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                save_mission();
                mission_map_changed = false;
                mission_sem.give();
            }
        }
    }
}

void MSoaringController::load_mission() {
    if (mission_loaded || mission_heatmap == nullptr) return;

    const char* fname = "@SYS/glide.bin";
    int fd = AP::FS().open(fname, O_RDONLY);
    //if (!f) f = fopen("glide.bin", "rb"); // SITL fallback
    //if (!f) f = fopen("libraries/AP_Soaring/tests/glide.bin", "rb"); // Unit test fallback

    if (fd == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GLIDE: No glide.bin found!");
        return;
    }

    missionHeader header;
    if (AP::FS().read(fd, &header, sizeof(header)) != sizeof(header)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: File too short");
        AP::FS().close(fd);
        return;
    }

    if (header.magic != 0x47503533) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Invalid Magic Number");
        AP::FS().close(fd);
        return;
    }

    grid_width_m = header.grid_width_m;
    grid_height_m = header.grid_height_m;
    grid_res_m = header.resolution_m;
    heatmap_origin.lat = header.origin_lat_e7;
    heatmap_origin.lng = header.origin_lng_e7;
    heatmap_origin.alt = 0;

    mis_alt_min_m = header.floor_alt;
    mis_alt_max_m = header.ceiling_alt;
    mis_batt_mah = header.start_battery_mah;
    mis_time_s = header.mission_time_s;
    mis_rth_enable = (header.reserve_rth > 0);

    // 2 cells per byte -> Size = (W * H) / 2
    uint32_t payload_size = (grid_width_m * grid_height_m) / 2;
    if (payload_size > MAX_HEATMAP_BYTES) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Mission file exceeds memory limit.");
        AP::FS().close(fd);
        return;
    }
        
    if (AP::FS().read(fd, mission_heatmap, payload_size) == payload_size) {
        mission_loaded = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GLIDE: Mission Loaded");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Failed to read heatmap payload");
    }
    AP::FS().close(fd);
}

void MSoaringController::save_mission() {
    if (mission_heatmap == nullptr || !mission_loaded) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GLIDE: No mission data to save.");
        return;
    }

    const char* fname = "@SYS/glide.bin";
    int fd = AP::FS().open(fname, O_WRONLY | O_CREAT | O_TRUNC);

    if (fd == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Failed to open file for saving");
        return;
    }

    // header
    missionHeader header;
    header.magic = 0x47503533;
    header.origin_lat_e7 = heatmap_origin.lat;
    header.origin_lng_e7 = heatmap_origin.lng;
    header.grid_width_m = grid_width_m;
    header.grid_height_m = grid_height_m;
    header.resolution_m = grid_res_m;
    header.floor_alt = mis_alt_min_m;
    header.ceiling_alt = mis_alt_max_m;
    header.start_battery_mah = mis_batt_mah;
    header.mission_time_s = mis_time_s;
    header.reserve_rth = mis_rth_enable ? 1 : 0;
    
    ssize_t header_written = AP::FS().write(fd, &header, sizeof(header));
    if (header_written != sizeof(header)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Failed to write header");
        AP::FS().close(fd);
        return;
    }
    
    uint32_t payload_size = (grid_width_m * grid_height_m) / 2; 
    ssize_t payload_written = AP::FS().write(fd, mission_heatmap, payload_size);
    if (payload_written != payload_size) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Failed to write heatmap payload");
        AP::FS().close(fd);
        return;
    }

    AP::FS().close(fd);

}


// LOGGING
void MSoaringController::Log_Write_Soaring(const SoaringAction &action) {
#if HAL_LOGGING_ENABLED
    AP::logger().Write(
        "SOAR",                         
        "Time_micros,Bank,Thr,Tot,Mis,Eng,Saf,Lam", 
        "sdddddd-",                     // units
        "0-------",                     // multipliers
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

void MSoaringController::log_state_to_csv(const VehicleState &state, const SoaringAction &action) {
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
            (unsigned)state.time_us,
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


// FAILSAFE

// interfaces from old soaring code expected by other libraries
/* void MSoaringController::update_active_state(bool override_disable) { 
    if (override_disable) {
        // TODO - override to Failsafe?
        return;
    }
}

void MSoaringController::init_cruising() {
    // Reset timers so the controller doesn't use stale data
    last_tactical_update_ms = AP_HAL::millis();
    last_best_score = -FLT_MAX;
}

bool MSoaringController::get_throttle_suppressed() const {
    return (msoar_enable > 0 && last_action.throttle_pct == 0);
}
 */

/* 
float MSoaringController::get_cruising_target_airspeed() const {
    if (msoar_v_glide.get() < 0.1f) {
        return _aparm.airspeed_cruise;
    }
    return constrain_float(msoar_v_glide.get(), _aparm.airspeed_min, _aparm.airspeed_max);
}

float MSoaringController::get_alt_cutoff() const {
    return mis_alt_max_m;
}
 */
#endif
