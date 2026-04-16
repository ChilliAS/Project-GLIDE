#include "SIM_Updraft.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

namespace SITL {

const AP_Param::GroupInfo UpdraftField::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Custom Updraft Field
    // @Description: 1 to load updraft_field.csv and inject custom lift
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, UpdraftField, enable, 0),

    AP_GROUPEND
};

void CustomUpdraft::init(const Location &loc, float w0, float r_u, float r_v, float heading, bool drift, uint32_t now_us) {
    start_loc = loc;
    strength_w0 = w0;       
    radius_u = r_u;        
    radius_v = r_v;         
    axis_heading_deg = heading; 
    advects = drift;          
    creation_time_us = now_us;
}

float CustomUpdraft::get_lift_at(const Location &query_loc, uint32_t now_us, float wind_n, float wind_e) const {
    if (strength_w0 < 0.1f) return 0.0f;
    
    Location core_loc = start_loc;
    if (advects) {
        float age_s = (now_us - creation_time_us) * 1.0e-6f;
        core_loc.offset(wind_n * age_s, wind_e * age_s);
    }

    Vector2f dist_ne = core_loc.get_distance_NE(query_loc);
    
    float heading_rad = radians(axis_heading_deg);
    float cos_t = cosf(heading_rad);
    float sin_t = sinf(heading_rad);

    float d_u = (dist_ne.x * cos_t) + (dist_ne.y * sin_t);
    float d_v = -(dist_ne.x * sin_t) + (dist_ne.y * cos_t);

    float r_u_sq = MAX(sq(radius_u), 1.0f);
    float r_v_sq = MAX(sq(radius_v), 1.0f);

    float exponent = -((d_u * d_u) / r_u_sq + (d_v * d_v) / r_v_sq);
    return strength_w0 * expf(exponent);
}

void UpdraftField::update(uint32_t now_us) {
    int8_t current_enable = enable.get();
    
    if (current_enable == 1 && _last_enable_state != 1) {
        load_from_csv("updraft_field.csv", now_us);
    }
    
    _last_enable_state = current_enable;
}

bool UpdraftField::load_from_csv(const char* filepath, uint32_t now_us) {
    FILE* file = fopen(filepath, "r");
    if (!file) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "[SIM_Updraft] Could not open %s", filepath);
        return false;
    }

    char line[256];
    num_updrafts = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[SIM_Updraft] Loading updraft field from %s...", filepath);

    while (fgets(line, sizeof(line), file) && num_updrafts < MAX_SIM_UPDRAFTS) {
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '#') continue;

        float lat_deg, lng_deg, w0, ru, rv, hdg;
        int advects_int;

        // Parse: Lat, Lng, Strength, RadU, RadV, Heading, Advects
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%d", 
                   &lat_deg, &lng_deg, &w0, &ru, &rv, &hdg, &advects_int) == 7) {
            
            Location loc;
            loc.lat = lat_deg * 1e7;
            loc.lng = lng_deg * 1e7;

            updrafts[num_updrafts].init(loc, w0, ru, rv, hdg, (advects_int == 1), now_us);
            num_updrafts++;
        }
    }

    fclose(file);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[SIM_Updraft] Successfully loaded %u updrafts.", num_updrafts);
    return true;
}

float UpdraftField::get_total_lift_at(const Location &query_loc, uint32_t now_us, float wind_n, float wind_e) const {
    float total_lift = 0.0f;
    for (uint8_t i = 0; i < num_updrafts; i++) {
        float lift = updrafts[i].get_lift_at(query_loc, now_us, wind_n, wind_e);
        if (lift > total_lift) {
            total_lift = lift; // Take the MAX lift of overlapping thermals
        }
    }
    
    static uint32_t last_gcs_msg_us = 0;
    if (now_us - last_gcs_msg_us >= 2000000UL) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[SIM_Updraft] True Lift: %.2f m/s", (double)total_lift);
        last_gcs_msg_us = now_us;
    }
    
    return total_lift;
}

} // end namespace SITL