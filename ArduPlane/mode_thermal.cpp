#include "mode.h"
#include "Plane.h"

/*
  THERMAL Mode - Project GLIDE
  Soaring controller for Roll and Throttle. 
  TECS controls Pitch to maintain airspeed.
*/

bool ModeThermal::_enter()
{
	// soar enable check
	if (!plane.g2.soaring_controller.is_active()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GLIDE: SOAR Not enabled");
        return false;
    }
	// health check
    if (!plane.g2.soaring_controller.is_healthy()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "GLIDE: Unhealthy, cannot enter mode");
        return false;
    }
    
    // Reset timers
    plane.g2.soaring_controller.init_cruising();

    return true;
}

void ModeThermal::update()
{
    // Failsafe
    if (!plane.g2.soaring_controller.is_healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "SOARING FAILSAFE TRIGGERED");
        uint8_t fs_action = plane.g2.soaring_controller.get_failsafe_action();
        Mode::Number safe_mode = (fs_action == 1) ? Mode::Number::FLY_BY_WIRE_A : Mode::Number::RTL;
        plane.set_mode(safe_mode, ModeReason::FAILSAFE);
        return; 
    }


    plane.g2.soaring_controller.update();

    // Roll Target (centidegrees)
    plane.nav_roll_cd = (int32_t)plane.g2.soaring_controller.get_target_bank_angle_cd();

    // Set Airspeed Target (cm/s)
    plane.target_airspeed_cm = plane.g2.soaring_controller.get_thermalling_target_airspeed() * 100.0f;

    // TECS calculate Pitch
    plane.update_flight_stage();
    plane.calc_nav_pitch();
	
	int8_t desired_throttle_pct = plane.g2.soaring_controller.get_target_throttle_pct();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, desired_throttle_pct);
    
}


void ModeThermal::run()
{
    Mode::run();
	
}

bool ModeThermal::exit_heading_aligned() const
{
    return false;
}