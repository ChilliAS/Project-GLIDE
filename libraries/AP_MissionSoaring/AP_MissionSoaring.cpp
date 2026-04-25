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

    // @Param: SRCH_RAD
    // @DisplayName: mission Search Radius
    // @Description: Radius in grid cells (strided) to scan for density.
    AP_GROUPINFO("SRCH_RAD", 4, MSoaringController, msoar_mis_search_rad, 4),
    
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
    
    // @Param: FS_ACT
    // @DisplayName: Soaring Failsafe Action
    // @Description: Mode to switch to if soaring controller becomes unhealthy. 0:RTL, 1:FBWA.
    // @Values: 0:RTL, 1:FBWA
    AP_GROUPINFO("FS_ACT", 10, MSoaringController, msoar_fs_action, 1),
    
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
    
    // @Param: P0_AMP
    // @DisplayName: Point 0 Amps (0% Thr)
    // @Description: Current draw at 0% throttle.
    AP_GROUPINFO("P0_AMP", 14, MSoaringController, msoar_p0_amp, 0.2f),

    // @Param: P0_VZ
    // @DisplayName: Point 0 Climb (0% Thr)
    // @Description: Still air climb rate at 0% throttle (Negative for sink).
    AP_GROUPINFO("P0_VZ", 15, MSoaringController, msoar_p0_vz, -0.8f),

    // @Param: P1_THR
    // @DisplayName: Point 1 Throttle %
    // @Description: Throttle percentage for Point 1.
    AP_GROUPINFO("P1_THR", 16, MSoaringController, msoar_p1_thr, 30),
    
    // @Param: P1_AMP
    // @DisplayName: Point 1 Amps
    // @Description: Current draw at P1 throttle.
    AP_GROUPINFO("P1_AMP", 17, MSoaringController, msoar_p1_amp, 0.5f),
    AP_GROUPINFO("P1_VZ",  18, MSoaringController, msoar_p1_vz,  0.3f),

    // @Param: P2_THR
    // @DisplayName: Point 2 Throttle %
    // @Description: Throttle percentage for Point 2.
    AP_GROUPINFO("P2_THR", 19, MSoaringController, msoar_p2_thr, 50),
    
    // @Param: P2_AMP
    // @DisplayName: Point 2 Amps
    // @Description: Current draw at P2 throttle.
    AP_GROUPINFO("P2_AMP", 20, MSoaringController, msoar_p2_amp, 4.3f),
    AP_GROUPINFO("P2_VZ",  21, MSoaringController, msoar_p2_vz,  1.8f),

    // @Param: P3_THR
    // @DisplayName: Point 3 Throttle %
    // @Description: Throttle percentage for Point 3.
    AP_GROUPINFO("P3_THR", 22, MSoaringController, msoar_p3_thr, 60),
    
    // @Param: P3_AMP
    // @DisplayName: Point 3 Amps
    // @Description: Current draw at P3 throttle.
    AP_GROUPINFO("P3_AMP", 23, MSoaringController, msoar_p3_amp, 12.0f),
    AP_GROUPINFO("P3_VZ",  24, MSoaringController, msoar_p3_vz,  3.5f),

    // @Param: P4_THR
    // @DisplayName: Point 4 Throttle %
    // @Description: Throttle percentage for Point 4.
    AP_GROUPINFO("P4_THR", 25, MSoaringController, msoar_p4_thr, 85),
    
    // @Param: P4_AMP
    // @DisplayName: Point 4 Amps
    // @Description: Current draw at P4 throttle.
    AP_GROUPINFO("P4_AMP", 26, MSoaringController, msoar_p4_amp, 21.5f),
    AP_GROUPINFO("P4_VZ",  27, MSoaringController, msoar_p4_vz,  5.5f),
    
    // @Param: M_GLB_UPD
    // @DisplayName: Global Updraft Pull Multiplier
    // @Description: Scales the attraction to the best global updraft. Set to 0 to disable.
    AP_GROUPINFO("M_GLB_UPD", 28, MSoaringController, msoar_m_glb_upd, 1.0f),

    // @Param: M_GLB_MIS
    // @DisplayName: Global Mission Pull Multiplier
    // @Description: Scales the attraction to distant mission targets. Set to 0 to disable.
    AP_GROUPINFO("M_GLB_MIS", 29, MSoaringController, msoar_m_glb_mis, 1.0f),

    // @Param: M_MIS_TOT
    // @DisplayName: Total Mission Score Multiplier
    // @Description: Scales the final mission score. Set to 0 to disable mission behavior.
    AP_GROUPINFO("M_MIS_TOT", 30, MSoaringController, msoar_m_mis_tot, 1.0f),

    // @Param: M_ENG_TOT
    // @DisplayName: Total Energy Score Multiplier
    // @Description: Scales the final energy score. Set to 0 to disable energy optimization.
    AP_GROUPINFO("M_ENG_TOT", 31, MSoaringController, msoar_m_eng_tot, 1.0f),

    // @Param: PEN_MOT
    // @DisplayName: Motor Start Penalty
    // @Description: Score penalty applied for transitioning from 0% to >0% throttle.
    AP_GROUPINFO("PEN_MOT", 32, MSoaringController, msoar_pen_mot, 0.5f),

    // @Param: K_SAFE
    // @DisplayName: Safety Penalty Multiplier
    // @Description: Scales the penalty for breaching altitude limits.
    AP_GROUPINFO("K_SAFE", 33, MSoaringController, msoar_k_safe, 50.0f),

    // @Param: K_GREED
    // @DisplayName: Greed / Amp Cost Multiplier
    // @Description: Cost in equivalent m/s climb rate per Amp drawn.
    AP_GROUPINFO("K_GREED", 34, MSoaringController, msoar_k_greed, 0.5f),

    // @Param: ALT_BUF
    // @DisplayName: Altitude Floor Buffer
    // @Description: Buffer above minimum altitude where safety penalty begins applying.
    // @Units: m
    AP_GROUPINFO("ALT_BUF", 35, MSoaringController, msoar_alt_buf, 5.0f),

    // @Param: K_THR_HYST
    // @DisplayName: Throttle Hysteresis Penalty
    // @Description: Penalty applied per percent of throttle change from previous action.
    AP_GROUPINFO("K_THR_HYST", 36, MSoaringController, msoar_k_thr_hyst, 0.1f),

    // @Param: MAX_BANK
    // @DisplayName: Maximum Bank Angle
    // @Description: Maximum bank angle considered by the soaring tactical loop.
    // @Units: deg
    AP_GROUPINFO("MAX_BANK", 37, MSoaringController, msoar_max_bank, 30),

    // @Group: UPD_
    // @Path: UpdraftEstimator.cpp
    AP_SUBGROUPINFO(updraft_estimator, "U_", 38, MSoaringController, UpdraftEstimator),

    AP_GROUPEND
};


// Initialise AHRS, Airframe, and the Estimators
MSoaringController::MSoaringController(AP_AHRS &ahrs, const AP_FixedWing &parms) : 
    _ahrs(ahrs),
    _aparm(parms),
    _vario(parms, _polar_params),
    last_action{} 
{
    AP_Param::setup_object_defaults(this, var_info);
                   
}

void MSoaringController::init() {
    hal.scheduler->delay(3000); 
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GLIDE: Initialsing!");
    if (mission_heatmap == nullptr) {
        mission_heatmap = (uint8_t*)hal.util->malloc_type(MAX_HEATMAP_BYTES, AP_HAL::Util::MEM_FAST);
        if (mission_heatmap == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "GLIDE: FATAL - Not enough memory!");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GLIDE: Heatmap allocated successfully.");
        }
    }
    init_io_thread();
    updraft_estimator.init();
}

// PRIMARY LOOPS (parameterless - run by AP scheduler, passing state to overloaded functions)
void MSoaringController::update_strategic_loop() {
    VehicleState state = get_current_state();
    if (!msoar_enable || !state.is_armed) return;
    
    update_strategic_loop(state);
}

void MSoaringController::update_tactical_loop() {
    if (!msoar_enable) return;
    
    _vario.update(_ahrs.get_roll());
    
    VehicleState state = get_current_state();
    
    if (!state.is_armed || state.current_loc.is_zero()) return; 
    
    update_tactical_loop(state);
}

void MSoaringController::update_strategic_loop(const VehicleState &state) {

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
            
            if (max_local_x > 0 && max_local_y > 0) { // sanity check swath larger than cell - not on the ground
                
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
    
    // Updraft estimator update
    
    Vector2f wind_xy(state.wind.x, state.wind.y);
    updraft_estimator.trigger_update(state.current_loc, state.alt_m, state.ground_vel_m_s, wind_xy);
    has_global_thermal = updraft_estimator.get_best_global_updraft(state.current_loc, global_thermal_loc, global_thermal_strength, state.time_us);
}

void MSoaringController::update_tactical_loop(const VehicleState &state) {
    
    if (last_tactical_update_us == 0) {
        last_tactical_update_us = state.time_us;
        return; 
    }
    float dt = (state.time_us - last_tactical_update_us) * 1.0e-6f;
     
    update_thermals(state, dt);
    SoaringAction action = calculate_optimal_action(state);  

    if (action.is_valid) {
        if (_log_enable) {
            Log_Write_Soaring(action);
            log_estimators(state.time_us);
        }
        last_action = action; 
    }
    
    last_tactical_update_us = state.time_us;
    

}


// MDP Functions (inc input fetch)
// Get A/C state information - decouple hardware to allow unit testing of soaring controller
MSoaringController::VehicleState MSoaringController::get_current_state() {
    VehicleState state{};
    update_perf_table();   
    
    _ahrs.get_relative_position_D_home(state.alt_m);
    state.alt_m *= -1.0f;
    
    if (!_ahrs.airspeed_TAS(state.tas_m_s) || state.tas_m_s < _aparm.airspeed_min) {
        state.tas_m_s = _aparm.airspeed_cruise; 
    }
    
    const Matrix3f &rot = _ahrs.get_rotation_body_to_ned();
    Vector3f forward = rot * Vector3f(1,0,0);
    state.heading_true_rad = atan2f(forward.y, forward.x);
    state.wind = _ahrs.wind_estimate();
    if (!_ahrs.get_location(state.current_loc)) {
        state.current_loc.zero();
    }
    
    float consumed_mah = 0.0f;
    if (!AP::battery().consumed_mah(consumed_mah, 0)) {
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
    
    float throttle_pct = 0.0f;
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
        throttle_pct = MAX(0.0f, SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
    }

    // removed for redevelopment, not wokring well - TODO: Variometer that works while under power
    //float expected_motor_climb = 0.0f;
    
    /* if (throttle_pct > 0.0f) {
        uint8_t num_steps = ARRAY_SIZE(perf_table);
        float expected_vz = perf_table[num_steps - 1].vz_still_air; 
        
        // Interpolate expected climb based on the performance table
        for (uint8_t i = 1; i < num_steps; i++) {
            if (throttle_pct <= perf_table[i].throttle_pct) {
                float t0 = perf_table[i-1].throttle_pct;
                float t1 = perf_table[i].throttle_pct;
                float v0 = perf_table[i-1].vz_still_air;
                float v1 = perf_table[i].vz_still_air;
                
                float fraction = 0.0f;
                if ((t1 - t0) > 0.0f) { // <--- Safe math check
                    fraction = (throttle_pct - t0) / (t1 - t0);
                }
                expected_vz = v0 + fraction * (v1 - v0);
                break;
            }
        }
        
        // Extrapolate if throttle is higher than the max table value (e.g., > 85)
        if (throttle_pct > perf_table[num_steps - 1].throttle_pct) {
            float t0 = perf_table[num_steps-2].throttle_pct;
            float t1 = perf_table[num_steps-1].throttle_pct;
            float v0 = perf_table[num_steps-2].vz_still_air;
            float v1 = perf_table[num_steps-1].vz_still_air;
            
            float fraction = 0.0f;
            if ((t1 - t0) > 0.0f) { 
                fraction = (throttle_pct - t0) / (t1 - t0);
            }
            expected_vz = v0 + fraction * (v1 - v0);
        }

        // the motor's specific contribution to the reading is the powered climb rate - unpowered sink rate
        expected_motor_climb = expected_vz - perf_table[0].vz_still_air;
    }

    // Set airmass rate: Vario reading minus the expected motor contribution
    state.airmass_rate_m_s = _vario.get_displayed_value() - expected_motor_climb; */
    
    if (throttle_pct > 0.0f) {
        state.airmass_rate_m_s = 0.0f;
    } else {
        state.airmass_rate_m_s = _vario.get_displayed_value();
    }
    
    #if HAL_GCS_ENABLED
    static uint32_t last_airmass_msg_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_airmass_msg_ms >= 1000) {
        last_airmass_msg_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "GLIDE: Air %.2f m/s", (double)state.airmass_rate_m_s);
    }
    #endif

    if (!_ahrs.get_relative_position_NE_home(state.pos_ne_m)) {
        state.pos_ne_m.zero(); // Fallback AHRS not ready
    }
    state.ground_vel_m_s = _ahrs.groundspeed_vector();
    
    return state;
}

// Update thermal belief
void MSoaringController::update_thermals(const VehicleState &state, float dt) {
    
    updraft_estimator.push_observation(state.current_loc, state.alt_m, state.airmass_rate_m_s, state.time_us);
    
}

// determine reward functions and select optimum action
MSoaringController::SoaringAction MSoaringController::calculate_optimal_action(const VehicleState &state) {
    SoaringAction best_action = {};
    best_action.score_total = -FLT_MAX;
    best_action.is_valid = false;
    
    const float k_safe = msoar_k_safe.get();
    const float k_greed = msoar_k_greed.get(); 
    const float epsilon = 0.05f;
    const float buffer = msoar_alt_buf.get();
    const float k_thr_hyst = msoar_k_thr_hyst.get(); 
    const int8_t max_bank = msoar_max_bank.get();


    for (int bank = -max_bank; bank <= max_bank; bank += 5) {
        
        Location pred_loc_2d = predict_position_future(state, (float)bank, 5.0f);
        
        float thermal_lift = predict_thermal_lift(state, pred_loc_2d);
        float mission_density = get_local_density_score(pred_loc_2d);
        
        // global pull if no nearby mission density
        if (mission_density < 0.01f && has_nearest_target) {
            float current_dist = state.current_loc.get_distance(nearest_target);
            float pred_dist = pred_loc_2d.get_distance(nearest_target);
            mission_density = (nearest_target_score / 15.0f * (current_dist - pred_dist) / MAX(current_dist, 10.0f)) * msoar_m_glb_mis.get();
        }
        
        // global pull if no nearby updraft nearby
        float global_updraft_rew = 0.0f;
        if (thermal_lift < 0.25f && has_global_thermal) {
            float current_dist = state.current_loc.get_distance(global_thermal_loc);
            float pred_dist = pred_loc_2d.get_distance(global_thermal_loc);
            global_updraft_rew = (global_thermal_strength * (current_dist - pred_dist) / MAX(current_dist, 10.0f)) * msoar_m_glb_upd.get();

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
            r_mis *= msoar_m_mis_tot.get();

            float r_greed = 0.0f;
            float thr_hyst = 0.0f;
            
            float glide_sink = perf_table[0].vz_still_air * load_sq;
            if (step.throttle_pct > 0 && (thermal_lift + glide_sink) > 0.5f) {
                r_greed = step.power_amps * k_greed;
            }
            thr_hyst = k_thr_hyst * fabsf(step.throttle_pct - last_action.throttle_pct);
            
            float cost = step.power_amps + r_greed + thr_hyst;
            float r_eng = (lambda_lagrange + epsilon) * (net_climb + global_updraft_rew - cost);
            r_eng *= msoar_m_eng_tot.get();

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
            
            // general hysteresis factor + for doing the same thing again
            if (fabsf((float)bank - last_action.bank_angle) < 0.5f && step.throttle_pct == last_action.throttle_pct) {
                total += 0.001f;
            }
            
            // Motor start penalty
            if (last_action.throttle_pct == 0 && step.throttle_pct > 0) {
                total -= msoar_pen_mot.get();
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
       
    float expected_lift = updraft_estimator.get_lift_prediction(pred_loc, state.alt_m, state.time_us);
    
    // Variance-Driven Exploration reward
    // float exploration_reward = updraft_estimator.get_variance(pred_loc, state.alt_m) * (state.battery_remaining_mah / 1000.0f);
    // return expected_lift + exploration_reward;
    
    return expected_lift;
}


// MISSION MAP FUNCTIONS
float MSoaringController::get_local_density_score(const Location &loc) {
    if (!mission_heatmap) return 0.0f;
    if (!mission_sem.take_nonblocking()) return 0.0f;

    int16_t cx, cy;
    if (!get_grid_coords_from_loc(loc, cx, cy)) {
        mission_sem.give();
        return 0.0f;
    }

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
    
#define CHECK_HEALTHY(cond, msg) if (!(cond)) { GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "SOAR: Unhealthy - " msg); return false; }

    CHECK_HEALTHY(mission_loaded, "Mission not loaded");
    CHECK_HEALTHY(mission_heatmap != nullptr, "Heatmap null");
    CHECK_HEALTHY(mis_time_s != 0, "Time is zero");
    CHECK_HEALTHY(tgt_alt.get() > 0.0f, "TGT_ALT invalid");
    CHECK_HEALTHY(grid_res_m > 0.0f, "Grid res invalid");
    CHECK_HEALTHY(heatmap_origin.is_zero() == false, "Origin is zero");
    CHECK_HEALTHY(mis_alt_min_m < mis_alt_max_m, "Alt limits invalid");
    CHECK_HEALTHY(_ahrs.healthy(), "AHRS not healthy");
    CHECK_HEALTHY(updraft_estimator.is_initialised(), "Estimator failed to boot");

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
        return 0.0f; // Fly straight if no valid action yet
    }
    // Convert degrees to centi-degrees for the Attitude Controller
    return last_action.bank_angle * 100.0f; 
}

int8_t MSoaringController::get_target_throttle_pct() const {
    if (!last_action.is_valid) return 0;
    return last_action.throttle_pct;
}

void MSoaringController::update_perf_table() {
    perf_table[0] = { 0, msoar_p0_amp.get(), msoar_p0_vz.get() };
    perf_table[1] = { msoar_p1_thr.get(), msoar_p1_amp.get(), msoar_p1_vz.get() };
    perf_table[2] = { msoar_p2_thr.get(), msoar_p2_amp.get(), msoar_p2_vz.get() };
    perf_table[3] = { msoar_p3_thr.get(), msoar_p3_amp.get(), msoar_p3_vz.get() };
    perf_table[4] = { msoar_p4_thr.get(), msoar_p4_amp.get(), msoar_p4_vz.get() };
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
    hal.scheduler->delay(5000); 
    
    while (true) {
        hal.scheduler->delay(1000);
        
        
        if (mission_load_requested && !mission_loaded) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: No Mission Loaded");
            if (mission_sem.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Trying Mission Load");
                load_mission();
                // ONLY stop requesting if we successfully loaded it
                if (mission_loaded) { 
                    mission_load_requested = false;
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GLIDE: Mission Loaded");
                }
                mission_sem.give();
            }
        }
    }
}

void MSoaringController::load_mission() {
    if (mission_loaded || mission_heatmap == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GLIDE: Mission Heatmap nullptr!");
    return;
    }

    const char* fname = "/glide.bin";
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
    ssize_t payload_size = (grid_width_m * grid_height_m) / 2;
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

    const char* fname = "/live_glide.bin";
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
    
    ssize_t payload_size = (grid_width_m * grid_height_m) / 2; 
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
        "MSOR",                         
        "TimeUS,Bank,Thr,Tot,Mis,Eng,Saf,Lam", 
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
        csv_file = fopen("libraries/AP_MissionSoaring/flight_data.csv", "w");
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


#endif
