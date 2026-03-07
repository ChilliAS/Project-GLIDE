#include "mode.h"
#include "Plane.h"

/*
  THERMAL Mode - Project GLIDE
  The gives the Project GLIDE Soaring Controller full autonomous control of Roll and Throttle Commands
  ArduPilot's TECS handles Pitch to maintain a safe airspeed.
*/
void ModeThermal::run()
{
    // THERMAL MODE FAILSAFE
    // If soaring controller unhealthy, reject the mode.
    if (!plane.soaring_controller.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING FAILSAFE");
        
        // 0 = RTL, 1 = FBWA
        uint8_t fs_action = plane.soaring_controller.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 1) ? Mode::Number::FLY_BY_WIRE_A : Mode::Number::RTL;
        
        // Execute mode change and abort soaring
        plane.set_mode(safe_mode, ModeReason::FAILSAFE);
        return; 
    }

    // Update soaring controller
    plane.soaring_controller.update();

    // Bank angle command in centidegrees
    plane.nav_roll_cd = (int32_t)plane.soaring_controller.get_target_bank_angle_cd();

    // Pass the 0-100% throttle command
    int8_t desired_throttle_pct = plane.soaring_controller.get_target_throttle_pct();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, desired_throttle_pct);

    // TECS for pitch
    plane.update_flight_stage();
    plane.calc_nav_pitch();

}

void ModeThermal::update()
{
    // run() handles the logic 

}
bool ModeThermal::_enter()
{
    // Return true to allow the mode change
    return true; 
}
bool ModeThermal::exit_heading_aligned() const
{
    return false;
}