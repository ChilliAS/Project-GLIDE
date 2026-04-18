#include <AP_gtest.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_MissionSoaring/UpdraftEstimator.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cstdlib>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Expected CSV input format:
//   0:  time_ms        (uint32)  — mission time in milliseconds
//   1:  alt_m          (float)   — barometric altitude, metres AMSL
//   2:  tas_m_s        (float)   — true airspeed, m/s
//   3:  yaw_deg        (float)   — heading, degrees clockwise from North
//   4:  wind_n         (float)   — wind North component, m/s
//   5:  wind_e         (float)   — wind East component, m/s
//   6:  lat_e7         (int32)   — latitude  × 1e7  (integer degrees)
//   7:  lng_e7         (int32)   — longitude × 1e7  (integer degrees)
//   8:  <skipped>                — reserved / sensor vertical speed
//   9:  wz             (float)   — estimated airmass vertical velocity, m/s

static constexpr uint8_t CSV_CAT_SLOTS = MAX_UPDRAFT_MEM;

TEST(UpdraftEstimatorTest, CSVPlaybackSim) {
    UpdraftEstimator estimator;
    estimator.init(false);

    // Open input file
    std::cout << "\n[ INFO ] Looking for updraft_data.csv...\n";
    std::ifstream in_file("libraries/AP_MissionSoaring/tests/updraft_data.csv");
    if (!in_file.is_open()) {
        FAIL() << "CRITICAL: Could not open updraft_data.csv! "
                  "Did you generate it from the HTML tool?";
    }

    // Create output file
    std::cout << "[ INFO ] Creating updraft_test_output.csv...\n";
    std::ofstream out_file("libraries/AP_MissionSoaring/tests/updraft_test_output.csv");
    if (!out_file.is_open()) {
        in_file.close();
        FAIL() << "CRITICAL: Could not create updraft_test_output.csv!";
    }

    out_file << "Time_ms,PlaneLat,PlaneLng,PlaneAlt,PlaneHdg,Wz,WindN,WindE";
    for (int i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
        out_file << ",C" << i << "_dN,C" << i << "_dE"
                 << ",C" << i << "_mean,C" << i << "_var";
    }


    for (int i = 0; i < CSV_CAT_SLOTS; i++) {
        out_file << ",Cat" << i << "_act"
                 << ",Cat" << i << "_typ"
                 << ",Cat" << i << "_w0"
                 << ",Cat" << i << "_radU"
                 << ",Cat" << i << "_radV"
                 << ",Cat" << i << "_hdg"
                 << ",Cat" << i << "_dN"
                 << ",Cat" << i << "_dE"
                 << ",Cat" << i << "_gateOpen";
    }
    out_file << "\n";

    // Parse
    std::string line;
    std::getline(in_file, line);

    uint32_t last_update_ms = 0;
    uint32_t prev_time_ms   = 0;
    bool     first_row      = true;

    Location origin_loc;
    bool origin_set = false;

    std::cout << "--- RUNNING PIPELINE ---\n";

    int row_number = 1;

    while (std::getline(in_file, line)) {
        row_number++;

        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string val;

        uint32_t time_ms = 0;
        float alt_m = 0.0f, tas_m_s = 0.0f, yaw_deg = 0.0f;
        float wind_n = 0.0f, wind_e = 0.0f;
        int32_t lat_e7 = 0, lng_e7 = 0;
        float wz = 0.0f;

        bool parse_error = false;

        auto parse_float = [&](const std::string& s) -> float {
            if (s.empty()) { parse_error = true; return 0.0f; }
            char* end;
            float res = std::strtof(s.c_str(), &end);
            if (end == s.c_str()) parse_error = true;
            return res;
        };

        auto parse_int = [&](const std::string& s) -> int32_t {
            if (s.empty()) { parse_error = true; return 0; }
            char* end;
            int32_t res = std::strtol(s.c_str(), &end, 10);
            if (end == s.c_str()) parse_error = true;
            return res;
        };

        auto parse_uint = [&](const std::string& s) -> uint32_t {
            if (s.empty()) { parse_error = true; return 0; }
            char* end;
            uint32_t res = std::strtoul(s.c_str(), &end, 10);
            if (end == s.c_str()) parse_error = true;
            return res;
        };

        if (std::getline(ss, val, ',')) time_ms  = parse_uint(val);
        if (std::getline(ss, val, ',')) alt_m    = parse_float(val);
        if (std::getline(ss, val, ',')) tas_m_s  = parse_float(val);
        if (std::getline(ss, val, ',')) yaw_deg  = parse_float(val);
        if (std::getline(ss, val, ',')) wind_n   = parse_float(val);
        if (std::getline(ss, val, ',')) wind_e   = parse_float(val);
        if (std::getline(ss, val, ',')) lat_e7   = parse_int(val);
        if (std::getline(ss, val, ',')) lng_e7   = parse_int(val);
        std::getline(ss, val, ',');
        if (std::getline(ss, val, ',')) wz       = parse_float(val);

        if (parse_error) {
            std::cerr << "[ WARN ] Parse error on row " << row_number
                      << " — skipping row.\n"
                      << "  Line: " << line << "\n";
            continue;
        }

        // time check
        if (!first_row && time_ms < prev_time_ms) {
            std::cerr << "[ WARN ] Row " << row_number
                      << ": timestamp went backwards ("
                      << time_ms << " < " << prev_time_ms
                      << ") — skipping row.\n";
            continue;
        }
        prev_time_ms = time_ms;
        first_row    = false;

        float yaw_rad = yaw_deg * DEG_TO_RAD;

        uint32_t time_us = static_cast<uint32_t>(
            static_cast<uint64_t>(time_ms) * 1000ULL);

        Vector2f wind_vel(wind_n, wind_e);

        Vector2f airspeed_vec(cosf(yaw_rad) * tas_m_s,
                              sinf(yaw_rad) * tas_m_s);
        Vector2f ground_vel = airspeed_vec + wind_vel;

        Location plane_loc;
        plane_loc.lat = lat_e7;
        plane_loc.lng = lng_e7;
        plane_loc.alt = static_cast<int32_t>(alt_m * 100.0f); // cm, as Location expects

        if (!origin_set) {
            origin_loc = plane_loc;
            origin_set = true;
        }

        estimator.push_observation(plane_loc, alt_m, wz, time_us);

        // Strategic update at 1 Hz
        if ((time_ms - last_update_ms) >= 1000U || last_update_ms == 0U) {

            estimator.trigger_update(plane_loc, alt_m, ground_vel, wind_vel);
            estimator.update_estimate(time_us);

            out_file << time_ms
                     << "," << std::fixed << std::setprecision(7)
                     << (plane_loc.lat * 1.0e-7)
                     << "," << (plane_loc.lng * 1.0e-7)
                     << "," << std::setprecision(3) << alt_m
                     << "," << yaw_deg
                     << "," << wz
                     << "," << wind_vel.x
                     << "," << wind_vel.y;

            int cell_idx = 0;
            for (int8_t x = -4; x <= 4; x++) {
                for (int8_t y = -4; y <= 4; y++) {
                    if (abs(x) == 4 && abs(y) == 4) {
                        continue;
                    }

                    float dN = static_cast<float>(x) * SGP_GRID_RES_M;
                    float dE = static_cast<float>(y) * SGP_GRID_RES_M;

                    Location query_loc = plane_loc;
                    query_loc.offset(dN, dE);

                    float mean = estimator.get_lift_prediction(query_loc, alt_m, time_us);
                    float var  = estimator.get_variance(query_loc, alt_m);

                    out_file << "," << std::setprecision(1) << dN
                             << "," << dE
                             << "," << std::setprecision(4) << mean
                             << "," << var;

                    cell_idx++;
                }
            }

            Vector2f plane_ne = origin_loc.get_distance_NE(plane_loc);

            for (uint8_t i = 0; i < CSV_CAT_SLOTS; i++) {
                const UpdraftObject& entry = estimator.get_catalog_entry(i);

                float rel_dN = entry.pos_north() - plane_ne.x;
                float rel_dE = entry.pos_east()  - plane_ne.y;

                out_file << "," << (entry.active ? 1 : 0)
                         << "," << static_cast<int>(entry.type.id)
                         << "," << std::setprecision(3) << entry.strength_w0()
                         << "," << entry.radius_u()
                         << "," << entry.radius_v()
                         << "," << std::setprecision(4) << entry.axis_heading
                         << "," << std::setprecision(2) << rel_dN
                         << "," << rel_dE
                         << "," << (entry.shape_gate_open ? 1 : 0);
            }

            out_file << "\n";
            out_file.flush();

            std::cout << "[ LOG ] " << std::fixed << std::setprecision(1)
                      << (time_ms / 1000.0f) << "s"
                      << " | Wz: " << std::setprecision(2) << wz << " m/s"
                      << " | Alt: " << std::setprecision(0) << alt_m << " m"
                      << "\n";

            last_update_ms = time_ms;
        }
    }

    in_file.close();
    out_file.close();

    std::cout << "[ SUCCESS ] updraft_test_output.csv generated.\n";
}

AP_GTEST_MAIN()