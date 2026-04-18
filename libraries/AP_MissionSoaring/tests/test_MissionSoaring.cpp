#include <AP_gtest.h>
#include <AP_MissionSoaring/AP_MissionSoaring.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(SoaringTest, RunDecoupledCSV) {
    AP_Filesystem fs;
    GCS_Dummy gcs_dummy;
        
    AP_AHRS* ahrs = new AP_AHRS();
    AP_FixedWing* aparm = new AP_FixedWing();
	
    MSoaringController controller(*ahrs, *aparm);
	controller.load_mission();

    // LOAD FLIGHT DATA
    std::cout << "\n[ INFO ] Looking for flight_data.csv...\n";
    std::ifstream file("libraries/AP_MissionSoaring/tests/flight_data.csv");
    
    if (!file.is_open()) {
        FAIL() << "CRITICAL: Could not open flight_data.csv!";
    }
	
	// Create output file
	std::cout << "[ INFO ] Creating test_output.csv...\n";
    std::ofstream out_file("libraries/AP_MissionSoaring/tests/test_output.csv");
    if (!out_file.is_open()) {
        FAIL() << "CRITICAL: Could not create test_output.csv!";
    }
	
	out_file << "Time(ms),Lambda,Bank(deg),Thr(%),TotScore,MisScore,EngScore,SafScore\n";
	
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

    uint32_t last_strat_us = 0;

    // RUN SIMULATION
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        MSoaringController::VehicleState state;
        
        if (std::getline(ss, val, ',')) state.time_us = std::stoul(val) * 1000ULL;
        if (std::getline(ss, val, ',')) state.alt_m = std::stof(val);
        if (std::getline(ss, val, ',')) state.tas_m_s = std::stof(val);
        if (std::getline(ss, val, ',')) state.heading_true_rad = std::stof(val) * (M_PI / 180.0f);
        if (std::getline(ss, val, ',')) state.wind.x = std::stof(val);
        if (std::getline(ss, val, ',')) state.wind.y = std::stof(val);
        if (std::getline(ss, val, ',')) state.current_loc.lat = std::stoi(val);
        if (std::getline(ss, val, ',')) state.current_loc.lng = std::stoi(val);
        if (std::getline(ss, val, ',')) state.battery_remaining_mah = std::stof(val);
        
        state.wind.z = 0.0f;
        state.current_loc.alt = 0.0f;
        state.is_armed = true; 
        state.dist_to_home_m = 1000.0f; // 1km away
        state.cruise_spd_m_s = 12.0f;
        state.airmass_rate_m_s = 0.0f; 
        state.pos_ne_m = Vector2f(0.0f, 0.0f);

        // Run Strategic Loop
        if (state.time_us - last_strat_us >= 1000000ULL || last_strat_us == 0) {
            controller.update_strategic_loop(state);
            last_strat_us = state.time_us;
        }

        // Run Tactical Loop
        controller.update_tactical_loop(state);
        MSoaringController::SoaringAction action = controller.get_last_action();
        
        std::cout << std::left << std::fixed
                  << std::setw(12) << (state.time_us / 1000)
                  << std::setw(10) << std::setprecision(4) << action.current_lambda
                  << std::setw(12) << std::setprecision(1) << action.bank_angle
                  << std::setw(8)  << (int)action.throttle_pct
                  << std::setw(12) << std::setprecision(4) << action.score_total
                  << std::setw(12) << std::setprecision(4) << action.score_mission
                  << std::setw(12) << std::setprecision(4) << action.score_energy
                  << std::setw(12) << std::setprecision(4) << action.score_safety
                  << "\n";
				  
        out_file << std::fixed
                 << (state.time_us / 1000) << ","
                 << std::setprecision(4) << action.current_lambda << ","
                 << std::setprecision(1) << action.bank_angle << ","
                 << (int)action.throttle_pct << ","
                 << std::setprecision(4) << action.score_total << ","
                 << std::setprecision(4) << action.score_mission << ","
                 << std::setprecision(4) << action.score_energy << ","
                 << std::setprecision(4) << action.score_safety << "\n";
    
    }
	out_file.close();
}

AP_GTEST_MAIN()