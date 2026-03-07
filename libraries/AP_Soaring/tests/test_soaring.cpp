#include <AP_gtest.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(SoaringTest, RunDecoupledCSV) {
    AP_AHRS ahrs;
    AP_FixedWing aparm;
	uint8_t dummy_tecs_mem[sizeof(AP_TECS)] = {0};
    AP_TECS& tecs = *reinterpret_cast<AP_TECS*>(dummy_tecs_mem);
	
    SoaringController controller(tecs, aparm);
	controller.load_heatmap();

    // LOAD FLIGHT DATA
    std::cout << "\n[ INFO ] Looking for flight_data.csv...\n";
    std::ifstream file("libraries/AP_Soaring/tests/flight_data.csv");
    
    if (!file.is_open()) {
        FAIL() << "CRITICAL: Could not open flight_data.csv!";
    }

	std::cout << "\n--- WAF TEST ---\n";    
    std::cout << std::left 
              << std::setw(10) << "Time(ms)"
              << std::setw(10) << "Lambda"
              << std::setw(12) << "Bank(deg)"
              << std::setw(8)  << "Thr(%)"
              << std::setw(12) << "TotScore"
              << std::setw(12) << "MisScore"
              << std::setw(12) << "EngScore"
              << std::setw(12) << "SafScore" << "\n";
    std::cout << std::string(88, '-') << "\n";
	
    std::string line;
    std::getline(file, line); // Skip CSV header

    uint32_t last_strat_ms = 0;

    // RUN SIMULATION
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        SoaringController::VehicleState state;
        
        if (std::getline(ss, val, ',')) state.time_ms = std::stoul(val);
        if (std::getline(ss, val, ',')) state.alt_m = std::stof(val);
        if (std::getline(ss, val, ',')) state.tas_m_s = std::stof(val);
        if (std::getline(ss, val, ',')) state.yaw_rad = std::stof(val);
        if (std::getline(ss, val, ',')) state.wind.x = std::stof(val);
        if (std::getline(ss, val, ',')) state.wind.y = std::stof(val);
        if (std::getline(ss, val, ',')) state.current_loc.lat = std::stoi(val);
        if (std::getline(ss, val, ',')) state.current_loc.lng = std::stoi(val);
        if (std::getline(ss, val, ',')) state.battery_remaining_mah = std::stof(val);
        
        // Mock the hardware variables
        state.wind.z = 0.0f;
        state.current_loc.alt = 0.0f;
        state.is_armed = true; 
        state.dist_to_home_m = 1000.0f; // 1km away
        state.cruise_spd_m_s = 12.0f;

        // Run Strategic Loop
        if (state.time_ms - last_strat_ms >= 1000 || last_strat_ms == 0) {
            controller.update_strategic_loop(state);
            last_strat_ms = state.time_ms;
        }

        // Run Tactical Loop
        SoaringController::SoaringAction action = controller.calculate_optimal_action(state);
        
        // Output telemetry table
		std::cout << std::left << std::fixed
                  << std::setw(10) << state.time_ms
                  << std::setw(10) << std::setprecision(4) << action.current_lambda
                  << std::setw(12) << std::setprecision(1) << action.bank_angle
                  << std::setw(8)  << (int)action.throttle_pct
                  << std::setw(12) << std::setprecision(4) << action.score_total
                  << std::setw(12) << std::setprecision(4) << action.score_mission
                  << std::setw(12) << std::setprecision(4) << action.score_energy
                  << std::setw(12) << std::setprecision(4) << action.score_safety
                  << "\n";
    }
}

AP_GTEST_MAIN()