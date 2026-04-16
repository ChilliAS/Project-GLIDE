#include "mode.h"
#include "Plane.h"

/*
  FLY-BY-WIRE THERMAL (FBWT) Mode - Project GLIDE
  Inherits all logic from FBWA (Pilot controls bank/pitch).
  Passively runs soaring controller in "Shadow Mode".

*/
#if HAL_MISSIONSOARING_ENABLED
bool ModeFBWS::_enter()
{

    if (!plane.mission_soaring.is_active() || !plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: Controller unhealthy");
        return false; // Reject entering the mode
    }
    
    plane.mission_soaring.set_logging_enabled(true);
    plane.mission_soaring.updraft_estimator.set_logging_enabled(true);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: FBWS ENGAGED");
    return true;
}

void ModeFBWS::update()
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
    ModeFBWA::update();



    if (plane.mission_soaring.is_active()) {
        //float shadow_bank_cd = plane.mission_soaring.get_target_bank_angle_cd();
        //int8_t shadow_throttle_pct = plane.mission_soaring.get_target_throttle_pct();
        plane.mission_soaring.log_estimators(AP_HAL::micros());
    }

}

void ModeFBWS::run()
{
    ModeFBWA::run();
}

void ModeFBWS::_exit()
{
    plane.mission_soaring.set_logging_enabled(false);
    plane.mission_soaring.updraft_estimator.set_logging_enabled(false);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: FBWS EXIT");
}


#endif