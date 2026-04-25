#include "ExtendedVariometer.h"
#include "AP_MissionSoaring.h"

const AP_Param::GroupInfo ExtendedVariometer::var_info[] = {
    // @Param: AC_MASS
    // @DisplayName: Aircraft Mass
    // @Description: Aircraft all-up mass.
    // @Units: kg
    AP_GROUPINFO("AC_MASS", 1, ExtendedVariometer, mass_kg, 3.0f),

    // @Param: PLR_K
    // @DisplayName: Polar Drag K
    // @Description: Induced drag factor K.
    AP_GROUPINFO("PLR_K", 2, ExtendedVariometer, polar_K, 0.025f),

    // @Param: PLR_CD0
    // @DisplayName: Polar Drag CD0
    // @Description: Zero-lift drag coefficient CD0.
    AP_GROUPINFO("PLR_CD0", 3, ExtendedVariometer, polar_CD0, 0.025f),

    // @Param: PLR_B
    // @DisplayName: Polar Drag B
    // @Description: Linear drag coefficient.
    AP_GROUPINFO("PLR_B", 4, ExtendedVariometer, polar_B, 0.0f),

    // @Param: W_AREA
    // @DisplayName: Wing Area
    // @Description: Reference wing planform area.
    // @Units: m^2
    AP_GROUPINFO("W_AREA", 5, ExtendedVariometer, wing_area_m2, 0.45f),

    // @Param: P_EFF
    // @DisplayName: Propeller Efficiency
    // @Description: Propulsive efficiency of the motor and propeller system.
    AP_GROUPINFO("P_EFF", 6, ExtendedVariometer, prop_efficiency, 0.65f),

    // @Param: TAU
    // @DisplayName: Filter Time Constant
    // @Description: Low-pass filter time constant.
    // @Units: s
    AP_GROUPINFO("TAU", 7, ExtendedVariometer, filter_tau_s, 1.5f),

    // @Param: CONF_P
    // @DisplayName: Pitch Confidence Suppression
    // @Description: Gaussian confidence suppression factor for pitch rate.
    AP_GROUPINFO("CONF_P", 8, ExtendedVariometer, conf_k_pitch, 15.0f),

    // @Param: CONF_R
    // @DisplayName: Roll Confidence Suppression
    // @Description: Gaussian confidence suppression factor for roll rate.
    AP_GROUPINFO("CONF_R", 9, ExtendedVariometer, conf_k_roll_rate, 8.0f),

    // @Param: CONF_T
    // @DisplayName: Thrust Transient Suppression
    // @Description: Gaussian confidence suppression factor for rapid changes in motor thrust.
    AP_GROUPINFO("CONF_T", 10, ExtendedVariometer, conf_k_thrust, 5.0f),

    // @Param: TE_GAIN
    // @DisplayName: Total Energy Gain
    // @Description: Multiplier for kinetic energy compensation to tune out stick thermals.
    AP_GROUPINFO("TE_GAIN", 11, ExtendedVariometer, te_gain, 1.0f),

    AP_GROUPEND
};

ExtendedVariometer::ExtendedVariometer() {
    AP_Param::setup_object_defaults(this, var_info);
}

void ExtendedVariometer::reset() {
    initialised       = false;
    filtered_air_lift = 0.0f;
    confidence        = 1.0f;
    raw_air_lift      = 0.0f;
    last_alpha        = 0.0f;
    last_W_thrust     = 0.0f;
}

float ExtendedVariometer::update(const MSoaringVehicleState &state, float dt) {
    if (dt <= 0.0f) {
        return filtered_air_lift;
    }

    if (!initialised) {
        last_alt_m   = state.alt_m;
        last_tas_m_s = state.tas_m_s;
        initialised  = true;
        return 0.0f;
    }

    const float dh_dt = (state.alt_m - last_alt_m) / dt;
    const float dv_dt = (state.tas_m_s - last_tas_m_s) / dt;

    last_alt_m   = state.alt_m;
    last_tas_m_s = state.tas_m_s;

    const float TE_rate = dh_dt + te_gain.get() * ((state.tas_m_s * dv_dt) / GRAVITY_MSS);

    const float safe_roll = constrain_float(fabsf(state.roll_rad), 0.0f, 1.0472f);
    const float n         = 1.0f / MAX(cosf(safe_roll), 0.5f);

    const float tas_safe  = MAX(state.tas_m_s, 1.0f);
    const float q_dyn     = 0.5f * state.air_density * sq(tas_safe);
    const float weight_N  = MAX(mass_kg.get() * GRAVITY_MSS, 0.1f);
    const float qS        = MAX(q_dyn * wing_area_m2.get(), 0.01f);

    const float CL        = (weight_N * n) / qS;
    
    const float drag_N    = qS * (polar_CD0.get() + (polar_B.get() * CL) + (polar_K.get() * sq(CL)));
    const float W_sink    = (drag_N * tas_safe) / weight_N;

    float W_thrust = 0.0f;
    const float P_elec = state.batt_voltage * state.batt_current_a;
    if (P_elec > 1.0f) {
        W_thrust = P_elec * constrain_float(prop_efficiency.get(), 0.1f, 0.95f) / weight_N;
    }

    const float dW_thrust_dt = (W_thrust - last_W_thrust) / dt;
    last_W_thrust = W_thrust;

    // Apply Gaussian suppression for pitch, roll, and thrust transients
    confidence = expf(-conf_k_pitch.get() * sq(state.pitch_rate_rads))
               * expf(-conf_k_roll_rate.get() * sq(state.roll_rate_rads))
               * expf(-conf_k_thrust.get() * sq(dW_thrust_dt));

    raw_air_lift = TE_rate + W_sink - W_thrust;

    const float base_alpha = dt / (filter_tau_s.get() + dt);
    last_alpha = base_alpha * confidence;
    filtered_air_lift += last_alpha * (raw_air_lift - filtered_air_lift);

    return filtered_air_lift;
}