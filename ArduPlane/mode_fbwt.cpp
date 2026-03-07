#include "mode.h"
#include "Plane.h"

/*
  FLY-BY-WIRE THERMAL (FBWT) Mode - Project GLIDE
  Inherits all logic from FBWA (Pilot controls bank/pitch).
  Passively runs soaring controller in "Shadow Mode".

*/

void ModeFBWT::run()
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
	
	//Run FBWA Logic
    ModeFBWA::run();

    // Update soaring controller
    plane.soaring_controller.update();


}

