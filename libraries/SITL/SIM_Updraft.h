#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

#define MAX_SIM_UPDRAFTS 50

namespace SITL {

class CustomUpdraft {
public:
    Location start_loc;
    float strength_w0;
    float radius_u;
    float radius_v;
    float axis_heading_deg;
    bool advects;
    uint32_t creation_time_us;

    void init(const Location &loc, float w0, float r_u, float r_v, float heading, bool drift, uint32_t now_us);
    float get_lift_at(const Location &query_loc, uint32_t now_us, float wind_n, float wind_e) const;
};

class UpdraftField {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 enable;

    // Field State
    CustomUpdraft updrafts[MAX_SIM_UPDRAFTS];
    uint8_t num_updrafts = 0;

    void update(uint32_t now_us);

    float get_total_lift_at(const Location &query_loc, uint32_t now_us, float wind_n, float wind_e) const;

private:
    int8_t _last_enable_state = -1;
    bool load_from_csv(const char* filepath, uint32_t now_us);
};

} // end namespace SITL