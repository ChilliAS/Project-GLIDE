#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

struct MSoaringVehicleState;

class ExtendedVariometer {
public:
    ExtendedVariometer();

    static const struct AP_Param::GroupInfo var_info[];

    float update(const MSoaringVehicleState &state, float dt);
    void reset();

    float get_confidence()   const { return confidence; }
    float get_raw_air_lift() const { return raw_air_lift; }
    float get_last_alpha()   const { return last_alpha; }
    float get_filtered_air_lift() const { return filtered_air_lift; }

private:
    // Parameters
    AP_Float mass_kg;
    AP_Float polar_K;
    AP_Float polar_B;
    AP_Float polar_CD0;
    AP_Float wing_area_m2;
    AP_Float prop_efficiency;
    AP_Float filter_tau_s;
    AP_Float conf_k_pitch;
    AP_Float conf_k_roll_rate;
    AP_Float conf_k_thrust;
    AP_Float te_gain;

    // Filter State
    bool  initialised = false;
    float last_alt_m = 0.0f;
    float last_tas_m_s = 0.0f;
    float filtered_air_lift = 0.0f;
    float confidence = 1.0f;
    float raw_air_lift = 0.0f;
    float last_alpha = 0.0f;
    float last_W_thrust = 0.0f;
};