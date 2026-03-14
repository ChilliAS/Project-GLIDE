#include "mode.h"
#include "Plane.h"

/*
  FLY-BY-WIRE THERMAL (FBWT) Mode - Project GLIDE
  Inherits all logic from FBWA (Pilot controls bank/pitch).
  Passively runs soaring controller in "Shadow Mode".

*/
bool ModeFBWS::_enter()
{
#if HAL_MISSIONSOARING_ENABLED
    if (!plane.mission_soaring.is_active() || !plane.mission_soaring.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SOAR: Controller unhealthy");
        return false; // Reject entering the mode
    }
#endif
    return true;
}

void ModeFBWS::update()
{
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
    ModeFBWA::update();

#if HAL_MISSIONSOARING_ENABLED

    if (plane.mission_soaring.is_active()) {
        float shadow_bank_cd = plane.mission_soaring.get_target_bank_angle_cd();
        int8_t shadow_throttle_pct = plane.mission_soaring.get_target_throttle_pct();
        // TODO LOGGING
    }
#endif
}

void ModeFBWS::run()
{
    ModeFBWA::run();
}

