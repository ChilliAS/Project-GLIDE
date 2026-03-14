#include "mode.h"
#include "Plane.h"

/*
  SOAR Mode - Project GLIDE
  Soaring controller for Roll and Throttle. 
  TECS controls Pitch to maintain airspeed.
*/

bool ModeSoar::_enter()
{
#if HAL_MISSIONSOARING_ENABLED
    if (!plane.mission_soaring.is_active() || !plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: Controller unhealthy");
        return false; // Reject entering the mode
    }
#endif
    return true;
}

void ModeSoar::update()
{
#if HAL_MISSIONSOARING_ENABLED
    // If soaring controller unhealthy, reject the mode.
    if (!plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING FAILSAFE");
        
        // 0 = RTL, 1 = FBWA
        uint8_t fs_action = plane.mission_soaring.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 1) ? Mode::Number::FLY_BY_WIRE_A : Mode::Number::RTL;
        
        // Execute mode change and abort soaring
        plane.set_mode(safe_mode, ModeReason::FAILSAFE);
        return; 
    }


    if (plane.mission_soaring.is_active()) {
        plane.nav_roll_cd = plane.mission_soaring.get_target_bank_angle_cd();
        plane.update_load_factor();
        
        int8_t target_throttle_pct = plane.mission_soaring.get_target_throttle_pct();
        plane.target_airspeed_cm = plane.mission_soaring.get_thermalling_target_airspeed() * 100.0f;
        plane.TECS_controller.set_gliding_requested_flag(target_throttle_pct == 0);
        
        // set limits on TECS to enforce desired throttle. 
        plane.TECS_controller.set_throttle_min(target_throttle_pct * 0.01f, true); 
        plane.TECS_controller.set_throttle_max(target_throttle_pct * 0.01f);
        
        // TODO LOGGING
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING DISABLED");
        uint8_t fs_action = plane.mission_soaring.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 1) ? Mode::Number::FLY_BY_WIRE_A : Mode::Number::RTL;
        plane.set_mode(safe_mode, ModeReason::FAILSAFE);

    }
#endif
}

void ModeSoar::run()
{

    Mode::run();

}

