#include "mode.h"
#include "Plane.h"

/*
  SOAR Mode - Project GLIDE
  Soaring controller for Roll and Throttle. 
  TECS controls Pitch to maintain airspeed.
*/
#if HAL_MISSIONSOARING_ENABLED
bool ModeSoar::_enter()
{

    if (!plane.mission_soaring.is_active() || !plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: Controller unhealthy");
        return false; // Reject entering the mode
    }
    
    plane.mission_soaring.set_logging_enabled(true);
    plane.mission_soaring.updraft_estimator.set_logging_enabled(true);

    return true;
}

void ModeSoar::update()
{
    // If soaring controller unhealthy, reject the mode.
    if (!plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING FAILSAFE");
        
        // 0 = RTL, 1 = FBWA
        uint8_t fs_action = plane.mission_soaring.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 0) ? Mode::Number::RTL : Mode::Number::FLY_BY_WIRE_A;
        plane.set_mode_by_number(safe_mode, ModeReason::FAILSAFE);
        return; 
    }


    if (plane.mission_soaring.is_active()) {
        plane.nav_roll_cd = plane.mission_soaring.get_target_bank_angle_cd();
        plane.update_load_factor();
        
        int8_t target_throttle_pct = plane.mission_soaring.get_target_throttle_pct();
        plane.target_airspeed_cm = plane.mission_soaring.get_thermalling_target_airspeed() * 100.0f;
        plane.TECS_controller.set_gliding_requested_flag(target_throttle_pct == 0);
        plane.set_target_altitude_current();
        
        // tell TECS what throttle to use - used in combination with cage
        //plane.TECS_controller.set_throttle_min(target_throttle_pct * 0.01f, false); 
        //plane.TECS_controller.set_throttle_max(target_throttle_pct * 0.01f);
        
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_gcs_log_ms >= 5000) {
            _last_gcs_log_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Free RAM: %u bytes", (unsigned)hal.util->available_memory());
/*             GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
            "SOAR thr=%d%% spd_dem=%.1f ptch=%.1f",
            (int)target_throttle_pct,
            (double)(plane.target_airspeed_cm / 100.0f),   // what YOU set
            (double)plane.TECS_controller.get_pitch_demand() // what TECS wants
            ); */
        } 
        
        plane.mission_soaring.log_estimators(AP_HAL::micros());
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING DISABLED");
        uint8_t fs_action = plane.mission_soaring.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 0) ? Mode::Number::RTL : Mode::Number::FLY_BY_WIRE_A;
        plane.set_mode_by_number(safe_mode, ModeReason::FAILSAFE);

    }
}

void ModeSoar::run()
{
    plane.calc_nav_pitch();
    Mode::run();

}

void ModeSoar::_exit()
{
    plane.mission_soaring.set_logging_enabled(false);
    plane.mission_soaring.updraft_estimator.set_logging_enabled(false);
}

int8_t ModeSoar::throttle_cage_min() const
{
    return plane.mission_soaring.get_target_throttle_pct() * 0.98; // leave a small window so TECS slew limiter works
}

int8_t ModeSoar::throttle_cage_max() const
{
    return plane.mission_soaring.get_target_throttle_pct();
}

#endif