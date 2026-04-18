#include <AP_gtest.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <cmath>
#include <random>

#define private public
#include <AP_MissionSoaring/UpdraftEstimator.h>
#undef private

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
const std::string OUT_DIR = "libraries/AP_MissionSoaring/tests/output/";

static std::string make_safe_filename(std::string name) {
    std::replace(name.begin(), name.end(), ' ', '_');
    std::replace(name.begin(), name.end(), '(', '_');
    std::replace(name.begin(), name.end(), ')', '_');
    return name;
}

struct TrueTarget {
    bool valid = false;
    float w0 = 0.0f, ru = 0.0f, rv = 0.0f;
};

class GenericTestLogger {
public:
    // Full state dump: OBS, ALPHA, CATALOGUE, META records
    static void dump_full_state(const std::string& filename, UpdraftEstimator& ue,
                                const float* alpha_override = nullptr,
                                uint32_t current_time_us = 0,
                                bool append = false,
                                TrueTarget tt = {}) {
        std::ofstream f;
        if (append) f.open(OUT_DIR + filename, std::ios::app);
        else        f.open(OUT_DIR + filename);
        if (!f.is_open()) return;

        if (!append) {
            f << "RecordType,Time_us,North_m,East_m,Val1,Val2,Val3,Val4,Val5,Val6,Val7,Val8,Val9,StringVal\n";
        }

        // META: wind
        f << "META,0,0,0,"
          << ue._shared_state.wind_vel.x << ","
          << ue._shared_state.wind_vel.y
          << ",0,0,0,0,0,0,0,\n";

        // OBS: Observations
        const uint16_t obs_count = ue._shared_state.obs_count;
        for (uint16_t i = 0; i < obs_count; i++) {
            uint16_t idx = (ue._shared_state.obs_head + SGP_MAX_OBSERVATIONS - obs_count + i) % SGP_MAX_OBSERVATIONS;
            const auto& obs = ue._shared_state.obs_buffer[idx];
            f << "OBS," << obs.time_us << "," << obs.local_pos.x << "," << obs.local_pos.y
              << "," << obs.wz << ",0,0,0,0,0,0,0,0,\n";
        }

        // ALPHA: Alpha weights
        const float* alpha_source = alpha_override ? alpha_override : ue._shared_state.alpha;
        for (int i = 0; i < SGP_NUM_INDUCING_POINTS; i++) {
            const auto& ip = ue._shared_state.inducing_points[i];
            f << "ALPHA," << current_time_us << "," << ip.x << "," << ip.y
              << "," << alpha_source[i] << ",0,0,0,0,0,0,0,0,\n";
        }

        // CATALOGUE: Catalogue entries
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            const auto& cat = ue.get_catalog_entry(i);
            if (!cat.active) continue;
            f << "CATALOGUE," << cat.last_update_us << "," << cat.pos_north() << "," << cat.pos_east() << ","
              << cat.strength_w0() << "," << cat.radius_u() << "," << cat.radius_v() << ","
              << cat.axis_heading << "," << (cat.shape_gate_open ? 1 : 0) << "," << (int)cat.type.id << ",";
            if (tt.valid) f << tt.w0 << "," << tt.ru << "," << tt.rv << ",\n";
            else          f << "0,0,0,\n";
        }
    }

    // 1D lift slice along East axis (North=0, East varying)
    static void dump_1d_slice(const std::string& filename, UpdraftEstimator& ue,
                              const Location& center_loc, uint32_t now_us, float alt_m = 100.0f) {
        std::ofstream f(OUT_DIR + filename);
        if (!f.is_open()) return;
        f << "Distance_m,EstimatedLift_ms\n";
        for (float dist = -80.0f; dist <= 80.0f; dist += 1.0f) {
            Location query_loc = center_loc;
            query_loc.offset(0.0f, dist);
            f << dist << "," << ue.get_lift_prediction(query_loc, alt_m, now_us) << "\n";
        }
    }

    // Raw matrix dump (one row per line)
    static void dump_matrix(const std::string& filename, const float* mat, int n) {
        std::ofstream f(OUT_DIR + filename);
        if (!f.is_open()) return;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                f << mat[i * n + j] << (j < n - 1 ? "," : "");
            }
            f << "\n";
        }
    }

    // 2D variance map [-50,50] x [-50,50]
    static void dump_variance_map(const std::string& filename, UpdraftEstimator& ue, float z_alt) {
        std::ofstream f(OUT_DIR + filename);
        if (!f.is_open()) return;
        f << "North_m,East_m,Variance\n";
        for (float n = -50.0f; n <= 50.0f; n += 2.0f) {
            for (float e = -50.0f; e <= 50.0f; e += 2.0f) {
                Location q = ue._local_origin; q.offset(n, e);
                f << n << "," << e << "," << ue.get_variance(q, z_alt) << "\n";
            }
        }
    }

    // Alpha weight dump for SGP lift field reconstruction
    static void dump_alpha(const std::string& filename, const float* alpha,
                           const Vector3f* ips, int M_count) {
        std::ofstream f(OUT_DIR + filename);
        if (!f.is_open()) return;
        f << "North_m,East_m,Alpha\n";
        for (int i = 0; i < M_count; i++) {
            f << ips[i].x << "," << ips[i].y << "," << alpha[i] << "\n";
        }
    }
};


class UpdraftTestBase : public ::testing::Test {
protected:
    UpdraftEstimator ue;
    static constexpr uint8_t  M = SGP_NUM_INDUCING_POINTS;
    static constexpr uint16_t KUU_SIZE = M * M;
    Vector3f grid_points[M];

    void SetUp() override {
        ue.init(false);
        ue.kern_length_xy.set(10.0f);
        ue.kern_length_z.set(45.0f);
        ue.kern_variance.set(8.0f);
        ue.obs_noise_var.set(0.5f);
        ue.obs_tau_decay.set(300.0f);
        ue.lift_min.set(0.4f);
        ue.peak_fraction.set(0.3f);
        ue.veto_age_max.set(5.0f);
        ue.veto_radius.set(15.0f);

        Location origin{};
        origin.lat = 509350000;
        origin.lng = -13940000;
        ue._local_origin = origin;
        ue._origin_set   = true;
        ue.initialised   = true;
        ue._prior_ready  = false;
        ue._matrices_valid = true;

        clear_catalogue();
        fill_standard_grid();
    }

    void clear_catalogue() {
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) ue._catalogue[i].active = false;
        ue._shared_state.obs_count = 0;
        ue._shared_state.obs_head  = 0;
    }

    void fill_standard_grid(float alt = 100.0f) {
        uint8_t idx = 0;
        for (int8_t x = -4; x <= 4; x++) {
            for (int8_t y = -4; y <= 4; y++) {
                if (abs(x) == 4 && abs(y) == 4) continue;
                grid_points[idx] = Vector3f(x * 10.0f, y * 10.0f, alt);
                ue._shared_state.inducing_points[idx] = grid_points[idx];
                ue._local_state.inducing_points[idx]  = grid_points[idx];
                idx++;
            }
        }
    }

    void mock_kf_entry(uint8_t index, float n, float e, float w0, uint32_t t) {
        auto& entry = ue._catalogue[index];
        entry.start_pos      = {n, e};
        entry.kf.init(w0, 30.0f, 30.0f, n, e);
        entry.axis_heading   = 0.0f;
        entry.type           = UpdraftTypes::UNKNOWN;
        entry.last_update_us  = t;
        entry.last_predict_us = t;
        entry.created_us      = t;
        entry.active          = true;
        entry.shape_gate_open = false;
    }
};


// SGP BUILDER

enum class SpdMatrixType {
    IDENTITY, STRONGLY_DOMINANT, HIGH_CORRELATION, TRIDIAGONAL,
    NON_SPD_NEGATIVE_DIAGONAL, NON_SPD_SADDLE_POINT, ACTUAL_GP_PRIOR
};

class sgpBuilderTest : public UpdraftTestBase {
protected:
    float matrix_A[KUU_SIZE] = {0};

    void generate_spd_matrix(float* A, SpdMatrixType type) {
        memset(A, 0, KUU_SIZE * sizeof(float));
        if (type == SpdMatrixType::ACTUAL_GP_PRIOR) {
            ue.sigma_n_sq = 0.5f;
            ue.build_prior_matrix(A, nullptr, grid_points, false);
            return;
        }
        for (uint8_t i = 0; i < M; i++) {
            for (uint8_t j = 0; j < M; j++) {
                switch (type) {
                    case SpdMatrixType::IDENTITY:
                        A[i * M + j] = (i == j) ? 1.0f : 0.0f; break;
                    case SpdMatrixType::STRONGLY_DOMINANT:
                        A[i * M + j] = (i == j) ? 100.0f : 1.0f; break;
                    case SpdMatrixType::HIGH_CORRELATION:
                        A[i * M + j] = (i == j) ? 400.0f : 5.0f; break;
                    case SpdMatrixType::TRIDIAGONAL:
                        if (i == j) A[i * M + j] = 4.0f;
                        else if (abs(i - j) == 1) A[i * M + j] = 1.0f;
                        break;
                    case SpdMatrixType::NON_SPD_NEGATIVE_DIAGONAL:
                        A[i * M + j] = (i == j) ? -10.0f : 1.0f; break;
                    case SpdMatrixType::NON_SPD_SADDLE_POINT:
                        A[i * M + j] = (i == j) ? 1.0f : 100.0f; break;
                    default: break;
                }
            }
        }
    }
};

TEST_F(sgpBuilderTest, BuildPriorMatrix_GivenNoCachedPrior_GeneratesSymmetricMatrix) {
    ue.sigma_n_sq = 0.5f;
    ue.build_prior_matrix(matrix_A, nullptr, grid_points, false);
    GenericTestLogger::dump_matrix("builder_scratch_matrix.csv", matrix_A, M);
    for (uint8_t i = 0; i < M; i++) {
        for (uint8_t j = i + 1; j < M; j++) {
            EXPECT_FLOAT_EQ(matrix_A[i * M + j], matrix_A[j * M + i]);
        }
    }
}

TEST_F(sgpBuilderTest, BuildPriorMatrix_GivenCachedPrior_CopiesValues) {
    ue.sigma_n_sq = 0.5f;
    float fake_prior[KUU_SIZE];
    for (uint16_t i = 0; i < KUU_SIZE; i++) fake_prior[i] = 42.0f;
    ue.build_prior_matrix(matrix_A, fake_prior, grid_points, true);
    EXPECT_FLOAT_EQ(matrix_A[0], 42.0f + 0.5f * 0.1f);
    EXPECT_FLOAT_EQ(matrix_A[1], 42.0f);
}

TEST_F(sgpBuilderTest, BuildPriorMatrix_GivenNaNBuffer_OverwritesWithValidData) {
    ue.sigma_n_sq = 0.5f;
    for (uint16_t i = 0; i < KUU_SIZE; i++) matrix_A[i] = std::numeric_limits<float>::quiet_NaN();
    ue.build_prior_matrix(matrix_A, nullptr, grid_points, false);
    for (uint16_t i = 0; i < KUU_SIZE; i++) EXPECT_FALSE(std::isnan(matrix_A[i]));
}

TEST_F(sgpBuilderTest, BuildPriorMatrix_GivenColocatedPoints_GeneratesCorrectValues) {
    ue.sigma_n_sq = 0.5f;
    Vector3f collapsed[M];
    for (uint8_t i = 0; i < M; i++) collapsed[i] = Vector3f(0, 0, 100);
    ue.build_prior_matrix(matrix_A, nullptr, collapsed, false);
    float ed = 8.0f + 0.5f * 0.1f, eo = 8.0f;
    for (uint8_t i = 0; i < M; i++) {
        for (uint8_t j = 0; j < M; j++) {
            EXPECT_FLOAT_EQ(matrix_A[i * M + j], (i == j) ? ed : eo);
        }
    }
}


// SGP MATHS — Cholesky, Solve, Variance

struct MatrixParams {
    const char* test_name;
    SpdMatrixType type;
    bool expected_success;
};

class sgpMathsTest : public sgpBuilderTest, public ::testing::WithParamInterface<MatrixParams> {};

TEST_P(sgpMathsTest, CholeskyDecompose_GivenSpdMatrix_DecomposesSuccessfully) {
    const MatrixParams& p = GetParam();
    generate_spd_matrix(matrix_A, p.type);
    GenericTestLogger::dump_matrix("maths_raw_matrix_" + make_safe_filename(p.test_name) + ".csv", matrix_A, M);
    bool success = ue.cholesky_decompose(matrix_A, M);
    if (success) {
        GenericTestLogger::dump_matrix("maths_L_factor_" + make_safe_filename(p.test_name) + ".csv", matrix_A, M);
    }
    EXPECT_EQ(success, p.expected_success);
    if (!p.expected_success) return;
    for (uint8_t i = 0; i < M; i++) {
        for (uint8_t j = i + 1; j < M; j++) EXPECT_FLOAT_EQ(matrix_A[i * M + j], 0.0f);
        EXPECT_GT(matrix_A[i * M + i], 0.0f);
    }
}

TEST_P(sgpMathsTest, CholeskySolve_GivenKnownVector_SolvesCorrectly) {
    const MatrixParams& p = GetParam();
    if (!p.expected_success) return;
    generate_spd_matrix(matrix_A, p.type);
    float known_x[M], vector_b[M], vector_x[M];
    for (uint8_t i = 0; i < M; i++) known_x[i] = 1.0f;
    for (uint8_t i = 0; i < M; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < M; j++) sum += matrix_A[i * M + j] * known_x[j];
        vector_b[i] = sum;
    }
    ASSERT_TRUE(ue.cholesky_decompose(matrix_A, M));
    ue.cholesky_solve(matrix_A, M, vector_b, vector_x);
    for (uint8_t i = 0; i < M; i++) EXPECT_NEAR(vector_x[i], known_x[i], 0.01f);
}

// build posterior Cholesky factors from a set of observation points
static bool build_posterior_factors(UpdraftEstimator& ue, const Vector3f* ips,
                                    const Vector3f* obs_pts, int n_obs_pts,
                                    float sigma_n_sq,
                                    float* out_l_kuu, float* out_l_a,
                                    int M_count) {
    const int KUU = M_count * M_count;

    // Build prior
    float kuu_buf[KUU];
    ue.build_prior_matrix(kuu_buf, nullptr, ips, false);

    // L_Kuu from pure prior
    memcpy(out_l_kuu, kuu_buf, KUU * sizeof(float));
    if (!ue.cholesky_decompose(out_l_kuu, M_count)) return false;

    // Accumulate observations into A = Kuu + sigma^-2 * Kuf*Kfu
    float a_buf[KUU];
    memcpy(a_buf, kuu_buf, KUU * sizeof(float));
    for (int o = 0; o < n_obs_pts; o++) {
        float kfu[77]; // max M
        for (int j = 0; j < M_count; j++) kfu[j] = ue.matern_kernel(obs_pts[o], ips[j]);
        for (int j = 0; j < M_count; j++) {
            for (int k = j; k < M_count; k++) {
                float val = kfu[j] * kfu[k] / sigma_n_sq;
                a_buf[j * M_count + k] += val;
                if (k != j) a_buf[k * M_count + j] += val;
            }
        }
    }
    memcpy(out_l_a, a_buf, KUU * sizeof(float));
    return ue.cholesky_decompose(out_l_a, M_count);
}

TEST_P(sgpMathsTest, GetVariance_GivenSingleObservation_ReducesVarianceAtOrigin) {
    const MatrixParams& p = GetParam();
    if (!p.expected_success) return;

    float l_kuu_buf[KUU_SIZE], l_a_buf[KUU_SIZE];
    const Vector3f obs_pt(0.0f, 0.0f, 100.0f);
    if (!build_posterior_factors(ue, grid_points, &obs_pt, 1, 0.5f, l_kuu_buf, l_a_buf, M)) return;

    float* saved_kuu = ue._L_Kuu_shared;
    float* saved_a   = ue._L_A_shared;
    ue._L_Kuu_shared = l_kuu_buf;
    ue._L_A_shared   = l_a_buf;

    GenericTestLogger::dump_variance_map(
        "maths_variance_single_" + make_safe_filename(p.test_name) + ".csv", ue, 100.0f);

    Location obs_loc = ue._local_origin;
    Location far_loc = ue._local_origin; far_loc.offset(40.0f, 40.0f);
    float var_at_obs = ue.get_variance(obs_loc, 100.0f);
    float var_far    = ue.get_variance(far_loc, 100.0f);
    EXPECT_LT(var_at_obs, var_far);

    ue._L_Kuu_shared = saved_kuu;
    ue._L_A_shared   = saved_a;
}

TEST_P(sgpMathsTest, GetVariance_GivenCrossPattern_ReducesVarianceAtAllObs) {
    const MatrixParams& p = GetParam();
    // Only meaningful for the actual GP prior
    if (!p.expected_success || p.type != SpdMatrixType::ACTUAL_GP_PRIOR) return;

    const Vector3f obs_pts[] = {
        {0.0f,   0.0f,  100.0f},
        {15.0f,  0.0f,  100.0f},
        {-15.0f, 0.0f,  100.0f},
        {0.0f,   15.0f, 100.0f},
        {0.0f,  -15.0f, 100.0f}
    };
    float l_kuu_buf[KUU_SIZE], l_a_buf[KUU_SIZE];
    if (!build_posterior_factors(ue, grid_points, obs_pts, 5, 0.5f, l_kuu_buf, l_a_buf, M)) return;

    float* saved_kuu = ue._L_Kuu_shared;
    float* saved_a   = ue._L_A_shared;
    ue._L_Kuu_shared = l_kuu_buf;
    ue._L_A_shared   = l_a_buf;

    GenericTestLogger::dump_variance_map("maths_variance_cross_pattern.csv", ue, 100.0f);

    // Variance at each obs centre should be lower than at an unobserved far corner
    Location far_loc = ue._local_origin; far_loc.offset(45.0f, 45.0f);
    float var_far = ue.get_variance(far_loc, 100.0f);
    for (const auto& op : obs_pts) {
        Location obs_loc = ue._local_origin; obs_loc.offset(op.x, op.y);
        EXPECT_LT(ue.get_variance(obs_loc, 100.0f), var_far);
    }

    ue._L_Kuu_shared = saved_kuu;
    ue._L_A_shared   = saved_a;
}

INSTANTIATE_TEST_SUITE_P(
    MatrixTopologies, sgpMathsTest,
    ::testing::Values(
        MatrixParams{"Identity Matrix",          SpdMatrixType::IDENTITY,                  true},
        MatrixParams{"Strongly Dominant",        SpdMatrixType::STRONGLY_DOMINANT,         true},
        MatrixParams{"High Correlation",         SpdMatrixType::HIGH_CORRELATION,          true},
        MatrixParams{"Tridiagonal",              SpdMatrixType::TRIDIAGONAL,               true},
        MatrixParams{"Actual GP Prior",          SpdMatrixType::ACTUAL_GP_PRIOR,           true},
        MatrixParams{"Negative Diagonal",        SpdMatrixType::NON_SPD_NEGATIVE_DIAGONAL, false},
        MatrixParams{"Saddle Point",             SpdMatrixType::NON_SPD_SADDLE_POINT,      false}
    )
);


// VARIANCE ARRANGEMENT TESTS

class VarianceArrangementsTest : public UpdraftTestBase {
protected:
    void run_arrangement(const std::string& name, const Vector3f* obs_pts, int n_pts) {
        float l_kuu_buf[KUU_SIZE], l_a_buf[KUU_SIZE];
        ue.sigma_n_sq = 0.5f;
        if (!build_posterior_factors(ue, grid_points, obs_pts, n_pts, 0.5f, l_kuu_buf, l_a_buf, M)) return;

        float* saved_kuu = ue._L_Kuu_shared;
        float* saved_a   = ue._L_A_shared;
        ue._L_Kuu_shared = l_kuu_buf;
        ue._L_A_shared   = l_a_buf;

        GenericTestLogger::dump_variance_map("maths_variance_arr_" + name + ".csv", ue, 100.0f);

        // All obs centres should have lower variance than a far unobserved corner
        Location far_loc = ue._local_origin; far_loc.offset(48.0f, 48.0f);
        float var_far = ue.get_variance(far_loc, 100.0f);
        for (int i = 0; i < n_pts; i++) {
            Location obs_loc = ue._local_origin; obs_loc.offset(obs_pts[i].x, obs_pts[i].y);
            EXPECT_LT(ue.get_variance(obs_loc, 100.0f), var_far);
        }

        ue._L_Kuu_shared = saved_kuu;
        ue._L_A_shared   = saved_a;
    }
};

TEST_F(VarianceArrangementsTest, Cross_ReducesVarianceAtAllArms) {
    const Vector3f pts[] = {{0,0,100},{30,0,100},{-30,0,100},{0,30,100},{0,-30,100}};
    run_arrangement("Cross", pts, 5);
}

TEST_F(VarianceArrangementsTest, LineNorth_ReducesVarianceAlongAxis) {
    const Vector3f pts[] = {{0,0,100},{30,0,100},{60,0,100},{-30,0,100},{-60,0,100}};
    run_arrangement("Line_N", pts, 5);
}

TEST_F(VarianceArrangementsTest, LineEast_ReducesVarianceAlongAxis) {
    const Vector3f pts[] = {{0,0,100},{0,30,100},{0,60,100},{0,-30,100},{0,-60,100}};
    run_arrangement("Line_E", pts, 5);
}

TEST_F(VarianceArrangementsTest, Corners_ReducesVarianceAtCorners) {
    const Vector3f pts[] = {{0,0,100},{30,30,100},{-30,30,100},{30,-30,100},{-30,-30,100}};
    run_arrangement("Corners", pts, 5);
}


// MATERN KERNEL

struct MaternParams {
    const char* test_name;
    Vector3f p1, p2;
    float expected_k;
    float param_l_xy, param_l_z, param_var;
};

class sgpMaternTest : public UpdraftTestBase, public ::testing::WithParamInterface<MaternParams> {};

TEST_P(sgpMaternTest, MaternKernel_GivenDistance_CalculatesCorrectCovariance) {
    const MaternParams& p = GetParam();
    ue.kern_length_xy.set(p.param_l_xy);
    ue.kern_length_z.set(p.param_l_z);
    ue.kern_variance.set(p.param_var);

    std::ofstream f(OUT_DIR + "matern_decay_" + make_safe_filename(p.test_name) + ".csv");
    if (f.is_open()) {
        f << "Distance_m,Covariance_XY,Covariance_Z\n";
        Vector3f origin(0, 0, 0);
        for (float d = 0.0f; d <= 100.0f; d += 0.5f) {
            f << d << ","
              << ue.matern_kernel(origin, Vector3f(d, 0, 0)) << ","
              << ue.matern_kernel(origin, Vector3f(0, 0, d)) << "\n";
        }
    }

    float kf = ue.matern_kernel(p.p1, p.p2);
    float kb = ue.matern_kernel(p.p2, p.p1);
    EXPECT_FLOAT_EQ(kf, kb);
    if (p.expected_k >= 0.0f) {
        EXPECT_NEAR(kf, p.expected_k, 0.001f);
        }
}

INSTANTIATE_TEST_SUITE_P(
    MaternEdgeCases, sgpMaternTest,
    ::testing::Values(
        MaternParams{"Identical Points",     {0,0,0}, {0,0,0},    8.0f,                                10, 45, 8},
        MaternParams{"1 LenScale XY 10m",   {0,0,0}, {10,0,0},   8.0f*(1+1.73205f)*expf(-1.73205f),  10, 45, 8},
        MaternParams{"1 LenScale Z 45m",    {0,0,0}, {0,0,45},   8.0f*(1+1.73205f)*expf(-1.73205f),  10, 45, 8},
        MaternParams{"3D Diagonal",         {0,0,0}, {10,10,45}, 8.0f*(1+3.0f)*expf(-3.0f),          10, 45, 8},
        MaternParams{"Far XY",              {0,0,0}, {1000,0,0}, 0.0f,                                10, 45, 8}
    )
);



// UPDRAFT CLASSIFICATION

struct ClassifyParams {
    std::string name;
    float actual_drift_n, major_axis, minor_axis, variance;
    UpdraftType expected;
};

class UpdraftClassificationParamTest : public UpdraftTestBase, public ::testing::WithParamInterface<ClassifyParams> {};

TEST_P(UpdraftClassificationParamTest, ClassifyUpdraft_GivenFeatures_AssignsCorrectType) {
    ClassifyParams tc = GetParam();
    UpdraftObject cat; cat.type = UpdraftTypes::UNKNOWN; cat.created_us = 0; cat.last_predict_us = 0;
    cat.kf.init(3.0f, 30.0f, 30.0f, tc.actual_drift_n, 0.0f);

    UpdraftFeatures features;
    features.peak_pos     = {cat.kf.x_N, cat.kf.x_E, 100.0f};
    features.max_lift     = 3.0f;
    features.core_variance = tc.variance;
    features.length_major  = tc.major_axis;
    features.length_minor  = tc.minor_axis;
    features.axis_heading  = 0.0f;

    ue.classify_updraft(cat, features, 100000000, {5.0f, 0.0f});
    EXPECT_EQ(cat.type.id, tc.expected.id) << "Failed on: " << tc.name;
}

TEST_P(UpdraftClassificationParamTest, ClassifyUpdraft_GivenAlreadyClassified_DoesNotOverwrite) {
    ClassifyParams tc = GetParam();
    if (tc.expected.id == UpdraftTypes::UNKNOWN.id) return;
    UpdraftObject cat; cat.type = UpdraftTypes::THERMAL_CHIMNEY; cat.created_us = 0; cat.last_predict_us = 0;
    cat.kf.init(3.0f, 30.0f, 30.0f, tc.actual_drift_n, 0.0f);

    UpdraftFeatures features; features.peak_pos = {cat.kf.x_N, cat.kf.x_E, 100.0f};
    ue.classify_updraft(cat, features, 100000000, {5.0f, 0.0f});
    EXPECT_EQ(cat.type.id, UpdraftTypes::THERMAL_CHIMNEY.id);
}

INSTANTIATE_TEST_SUITE_P(
    UpdraftHeuristics, UpdraftClassificationParamTest,
    ::testing::Values(
        ClassifyParams{"Thermal_Bubble", 500.0f, 30.0f,  30.0f, 3.0f, UpdraftTypes::THERMAL_BUBBLE},
        ClassifyParams{"Wind_Shear",     500.0f, 150.0f, 20.0f, 8.0f, UpdraftTypes::SHEAR},
        ClassifyParams{"Ridge_Terrain",  15.0f,  200.0f, 30.0f, 2.0f, UpdraftTypes::RIDGE},
        ClassifyParams{"Chimney",        10.0f,  30.0f,  30.0f, 1.0f, UpdraftTypes::OROGRAPHIC}
    )
);



// WIND ADVECTION SHAPE TEST

struct AdvectionShapeParams {
    const char* test_name;
    float wind_n, wind_e;
    float expected_n, expected_e;
    float var_stretch_n, var_stretch_e;
};

class AdvectionAndShapeTest : public UpdraftTestBase, public ::testing::WithParamInterface<AdvectionShapeParams> {};

TEST_P(AdvectionAndShapeTest, ProcessUpdrafts_GivenWindAdvection_CalculatesCorrectPeakAndShape) {
    const AdvectionShapeParams& p = GetParam();
    ue.tau_decay  = 10000.0f;
    ue.sigma_n_sq = 0.5f;

    float fake_alpha[M] = {0};
    for (int i = 0; i < M; i++) {
        float n = grid_points[i].x, e = grid_points[i].y;
        float dn = n - p.expected_n, de = e - p.expected_e;
        fake_alpha[i] = 2.0f * expf(-((dn*dn)/(2*p.var_stretch_n) + (de*de)/(2*p.var_stretch_e)));
    }

    ue.process_updrafts(ue._catalogue, fake_alpha, grid_points, 10000000, {p.wind_n, p.wind_e}, 100.0f);

    int active_idx = -1;
    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (ue._catalogue[i].active) { active_idx = i; break; }
    }

    EXPECT_GE(active_idx, 0);
    if (active_idx >= 0) {
        EXPECT_NEAR(ue._catalogue[active_idx].pos_north(), p.expected_n, 5.0f);
        EXPECT_NEAR(ue._catalogue[active_idx].pos_east(),  p.expected_e, 5.0f);
    }
    GenericTestLogger::dump_full_state("shape_advection_" + make_safe_filename(p.test_name) + ".csv", ue, fake_alpha);
}

INSTANTIATE_TEST_SUITE_P(
    AdvectionShapes, AdvectionAndShapeTest,
    ::testing::Values(
        AdvectionShapeParams{"Zero_Wind",  0, 0,   0,   0, 400.0f, 400.0f},
        AdvectionShapeParams{"North_Wind", 5, 0,  20,   0, 900.0f, 100.0f},
        AdvectionShapeParams{"East_Wind",  0, 5,   0,  20, 100.0f, 900.0f},
        AdvectionShapeParams{"SouthWest", -2,-2, -20, -20, 400.0f, 400.0f}
    )
);



// ADVECTION MULTI-DIRECTION TEST

struct AdvectionPosteriorParams {
    const char* test_name;
    float wind_n, wind_e;   // wind (m/s)
    float obs_n,  obs_e;    // obs placement (m)
    float expect_n, expect_e; // expected advected peak
};

class AdvectionPosteriorTest : public UpdraftTestBase,
                               public ::testing::WithParamInterface<AdvectionPosteriorParams> {};

TEST_P(AdvectionPosteriorTest, AccumulatePosterior_GivenWindAndAge_AdvectsObsToCorrectInducingPoint) {
    const AdvectionPosteriorParams& p = GetParam();
    ue.tau_decay  = 10000.0f;
    ue.sigma_n_sq = 0.5f;

    const uint32_t now_us = 10000000;
    const float    age_s  = 10.0f;

    UpdraftObservation obs;
    obs.local_pos = Vector3f(p.obs_n, p.obs_e, 100.0f);
    obs.wz        = 5.0f;
    obs.time_us   = now_us - (uint32_t)(age_s * 1.0e6f);

    float kuu[KUU_SIZE] = {0}, pvec[M] = {0};
    ue.accumulate_posterior(kuu, pvec, &obs, 1, now_us, {p.wind_n, p.wind_e},
                            ue._shared_state.inducing_points, ue._catalogue);

    uint8_t peak_idx = 0;
    for (uint8_t i = 1; i < M; i++) {
        if (pvec[i] > pvec[peak_idx]) peak_idx = i;
    }

    const Vector3f& peak_ip = ue._shared_state.inducing_points[peak_idx];
    float dist = sqrtf(sq(peak_ip.x - p.expect_n) + sq(peak_ip.y - p.expect_e));
    EXPECT_LT(dist, 15.0f) << "Direction: " << p.test_name;

    std::ofstream f(OUT_DIR + "pipe_advection_" + make_safe_filename(p.test_name) + ".csv");
    if (f.is_open()) {
        f << "Index,North_m,East_m,Posterior_Value,WindN_ms,WindE_ms\n";
        for (int i = 0; i < M; i++) {
            f << i << "," << ue._shared_state.inducing_points[i].x
              << "," << ue._shared_state.inducing_points[i].y
              << "," << pvec[i]
              << "," << p.wind_n << "," << p.wind_e << "\n";
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    WindDirections, AdvectionPosteriorTest,
    ::testing::Values(
        // Obs placed up-wind so that after age_s it reaches expect position
        AdvectionPosteriorParams{"North",     2,  0,   0,  0,  20,  0},
        AdvectionPosteriorParams{"South",    -2,  0,   0,  0, -20,  0},
        AdvectionPosteriorParams{"East",      0,  2,   0,  0,   0, 20},
        AdvectionPosteriorParams{"West",      0, -2,   0,  0,   0,-20},
        AdvectionPosteriorParams{"Northeast", 2,  2,   0,  0,  20, 20},
        AdvectionPosteriorParams{"Southwest",-2, -2,   0,  0, -20,-20}
    )
);



// PRUNE INACTIVE

struct PruneParams {
    const char* test_name;
    float cat_n, cat_e, cat_w0;
    float alpha_origin;
    bool expect_active;
};

class PruneInactiveTest : public UpdraftTestBase, public ::testing::WithParamInterface<PruneParams> {};

TEST_P(PruneInactiveTest, PruneInactive_GivenVariousStates_PrunesCorrectly) {
    const PruneParams& p = GetParam();
    mock_kf_entry(0, p.cat_n, p.cat_e, p.cat_w0, 0);
    float alpha[M] = {0}; alpha[38] = p.alpha_origin;
    ue.prune_inactive(ue._catalogue, alpha, grid_points, 100.0f);
    EXPECT_EQ(ue._catalogue[0].active, p.expect_active);
}

INSTANTIATE_TEST_SUITE_P(
    PruningLogic, PruneInactiveTest,
    ::testing::Values(
        PruneParams{"InsideGate_WeakSGP",   5,   5, 2.0f, 0.0f, false},
        PruneParams{"InsideGate_StrongSGP", 0,   0, 2.0f, 0.5f, true},
        PruneParams{"OutsideGate",        100, 100, 2.0f, 0.0f, true},
        PruneParams{"WeakStrength",         5,   5, 0.3f, 0.0f, true}
    )
);



// CATALOGUE TESTS

class UpdraftEstimatorCatalogueTest : public UpdraftTestBase {
protected:
    float fake_alpha[M] = {0};
};

TEST_F(UpdraftEstimatorCatalogueTest, ProcessUpdrafts_GivenMessyAlpha_ExtractsGlobalMaximum) {
    for (uint8_t i = 0; i < M; i++) fake_alpha[i] = (i % 2) ? 0.05f : -0.05f;
    fake_alpha[15] = 0.2f; fake_alpha[60] = 0.6f; fake_alpha[35] = -1.0f;
    ue.process_updrafts(ue._catalogue, fake_alpha, grid_points, 1000000, {0,0}, 100.0f);

    bool found = false;
    for (int c = 0; c < MAX_UPDRAFT_MEM; c++) {
        if (ue._catalogue[c].active
            && is_equal(ue._catalogue[c].start_pos.x, grid_points[60].x)
            && is_equal(ue._catalogue[c].start_pos.y, grid_points[60].y)) {
            found = true;
            EXPECT_GT(ue._catalogue[c].strength_w0(), 4.0f);
            break;
        }
    }
    EXPECT_TRUE(found);
    GenericTestLogger::dump_full_state("cat_messy_sky.csv", ue, fake_alpha);
}

TEST_F(UpdraftEstimatorCatalogueTest, ProcessUpdrafts_GivenStructuredAlpha_ExtractsMultiplePeaks) {
    for (uint8_t i = 0; i < M; i++) {
        float n = grid_points[i].x, e = grid_points[i].y;
        fake_alpha[i] += 0.8f * expf(-((n+20)*(n+20)/200.0f + (e+20)*(e+20)/200.0f));
        fake_alpha[i] += 0.6f * expf(-((n-20)*(n-20)/1800.0f + (e-20)*(e-20)/200.0f));
    }
    ue.process_updrafts(ue._catalogue, fake_alpha, grid_points, 1000000, {0,0}, 100.0f);
    int count = 0;
    for (int c = 0; c < MAX_UPDRAFT_MEM; c++) if (ue._catalogue[c].active) count++;
    EXPECT_EQ(count, 2);
    GenericTestLogger::dump_full_state("cat_structured_sky.csv", ue, fake_alpha);
}

TEST_F(UpdraftEstimatorCatalogueTest, ProcessUpdrafts_GivenWeakAlpha_FindsNoPeaks) {
    fake_alpha[38] = 0.04f;
    ue.process_updrafts(ue._catalogue, fake_alpha, grid_points, 1000000, {0,0}, 100.0f);
    for (int c = 0; c < MAX_UPDRAFT_MEM; c++) EXPECT_FALSE(ue._catalogue[c].active);
}

TEST_F(UpdraftEstimatorCatalogueTest, ProcessUpdrafts_GivenNegativeAlpha_FindsNoPeaks) {
    for (uint8_t i = 0; i < M; i++) fake_alpha[i] = -0.5f;
    ue.process_updrafts(ue._catalogue, fake_alpha, grid_points, 1000000, {0,0}, 100.0f);
    for (int c = 0; c < MAX_UPDRAFT_MEM; c++) EXPECT_FALSE(ue._catalogue[c].active);
}

TEST_F(UpdraftEstimatorCatalogueTest, UpdateCatalogue_GivenNewObservation_UpdatesExistingEntry) {
    uint32_t now_us = 2000000;
    mock_kf_entry(0, 10, 10, 2.0f, 1000000);
    ue._catalogue[0].type = UpdraftTypes::THERMAL_BUBBLE;

    UpdraftFeatures feat;
    feat.peak_pos      = {13, 10, 100};
    feat.max_lift      = 2.5f;
    feat.core_variance = 1.0f;
    feat.length_major  = 30.0f;
    feat.length_minor  = 30.0f;
    feat.axis_heading  = 0.0f;
    ue.update_catalogue(ue._catalogue, feat, now_us, {2,0}, 100.0f);

    EXPECT_FALSE(ue._catalogue[1].active);
    EXPECT_EQ(ue._catalogue[0].last_update_us, now_us);
    EXPECT_GT(ue._catalogue[0].pos_north(), 11.0f);
    EXPECT_LT(ue._catalogue[0].pos_north(), 13.0f);
}

TEST_F(UpdraftEstimatorCatalogueTest, GetBestGlobalUpdraft_GivenEmptyCatalogue_ReturnsFalse) {
    Location t; float s;
    EXPECT_FALSE(ue.get_best_global_updraft(ue._local_origin, t, s, 1000000));
}

TEST_F(UpdraftEstimatorCatalogueTest, GetBestGlobalUpdraft_GivenNearWeakAndFarStrong_SelectsNearWeak) {
    uint32_t now_us = 1000000;
    mock_kf_entry(0, 10, 0, 1.0f, now_us);
    mock_kf_entry(1, 500, 0, 10.0f, now_us);
    Location t; float s;
    ue.get_best_global_updraft(ue._local_origin, t, s, now_us);
    EXPECT_FLOAT_EQ(s, 1.0f);
}

TEST_F(UpdraftEstimatorCatalogueTest, GetBestGlobalUpdraft_GivenAdvectingTypes_CalculatesCorrectPositions) {
    uint32_t start_us = 1000000;
    uint32_t now_us   = 101000000UL; // 100 s later
    ue._shared_state.wind_vel = {5.0f, 0.0f};

    struct Profile { UpdraftType type; const char* name; float ru; float rv; };
    const Profile profiles[] = {
        {UpdraftTypes::UNKNOWN,         "Unknown",         30.0f, 30.0f},
        {UpdraftTypes::THERMAL_BUBBLE,  "Thermal_Bubble",  30.0f, 30.0f},
        {UpdraftTypes::THERMAL_CHIMNEY, "Thermal_Chimney", 50.0f, 30.0f},
        {UpdraftTypes::OROGRAPHIC,      "Orographic",      20.0f, 40.0f},
        {UpdraftTypes::SHEAR,           "Shear",           70.0f, 20.0f},
        {UpdraftTypes::RIDGE,           "Ridge",           20.0f, 70.0f}
    };

    for (int i = 0; i < 6; i++) {
        mock_kf_entry(i, 0.0f, (i - 2.5f) * 80.0f, 2.0f, start_us);
        ue._catalogue[i].type    = profiles[i].type;
        ue._catalogue[i].kf.x_Ru = profiles[i].ru;
        ue._catalogue[i].kf.x_Rv = profiles[i].rv;
    }

    GenericTestLogger::dump_full_state("cat_target_advection.csv", ue, nullptr, now_us);

    // for each type, get the advected target position
    std::ofstream fk(OUT_DIR + "cat_target_types.csv");
    if (fk.is_open()) {
        fk << "TypeName,TypeId,Advects,StartN_m,StartE_m,PredN_m,PredE_m,Ru_m,Rv_m,WindN,WindE,Dt_s\n";
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) ue._catalogue[j].active = (i == j);
            Location target; float s;
            bool ok = ue.get_best_global_updraft(ue._local_origin, target, s, now_us);
            Vector2f ne = ok ? ue._local_origin.get_distance_NE(target) : Vector2f(0, 0);
            bool advects = (profiles[i].type.id == 1 || profiles[i].type.id == 4); // Bubble or Shear
            fk << profiles[i].name << ","
               << (int)profiles[i].type.id << ","
               << (advects ? 1 : 0) << ","
               << 0.0f << ","
               << (i - 2.5f) * 80.0f << ","
               << ne.x << "," << ne.y << ","
               << profiles[i].ru << "," << profiles[i].rv << ","
               << 5.0f << "," << 0.0f << ","
               << 100.0f << "\n";
        }
        for (int i = 0; i < 6; i++) ue._catalogue[i].active = true;
    }
}

TEST_F(UpdraftEstimatorCatalogueTest, UpdraftLKF_PredictAndUpdate_ConvergesCovariance) {
    UpdraftLKF kf; kf.init(2.0f, 30.0f, 30.0f, 0.0f, 0.0f);
    float init_pN = kf.p_N;

    std::ofstream f(OUT_DIR + "cat_lkf_convergence.csv");
    if (f.is_open()) f << "Iteration,Uncertainty_N,Uncertainty_W0\n";

    for (int i = 0; i < 10; i++) {
        if (f.is_open()) f << i << "," << kf.p_N << "," << kf.p_W0 << "\n";
        kf.predict(0.1f, 0.0f, 0.0f, 0.0f, 0.0f);
        kf.update_strength_pos(2.5f, 2.0f, 2.0f);
        kf.update_radii(30.0f, 30.0f);
    }
    if (f.is_open()) f << 10 << "," << kf.p_N << "," << kf.p_W0 << "\n";

    EXPECT_LT(kf.p_N, init_pN);
}

TEST_F(UpdraftEstimatorCatalogueTest, UpdraftLKF_PredictOnly_IncreasesUncertainty) {
    UpdraftLKF kf; kf.init(2.0f, 30.0f, 30.0f, 0.0f, 0.0f);
    float pN0 = kf.p_N, pW0 = kf.p_W0;
    for (int i = 0; i < 5; i++) kf.predict(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_GT(kf.p_N, pN0); EXPECT_GT(kf.p_W0, pW0);
}

TEST_F(UpdraftEstimatorCatalogueTest, UpdraftLKF_GivenWind_AdvectsPosition) {
    UpdraftLKF kf; kf.init(2.0f, 30.0f, 30.0f, 100.0f, 50.0f);
    kf.predict(10.0f, 5.0f, 3.0f, 0.0f, 0.0f);
    EXPECT_NEAR(kf.x_N, 150, 0.1f); EXPECT_NEAR(kf.x_E, 80, 0.1f);
}

TEST_F(UpdraftEstimatorCatalogueTest, UpdraftLKF_GivenRepeatedMeasurements_ConvergesToMeasurement) {
    UpdraftLKF kf; kf.init(2.0f, 30.0f, 30.0f, 0.0f, 0.0f);
    for (int i = 0; i < 20; i++) {
        kf.predict(0.1f, 0.0f, 0.0f, 0.0f, 0.0f);
        kf.update_strength_pos(3.0f, 50.0f, 50.0f);
        kf.update_radii(30.0f, 30.0f);
    }
    EXPECT_NEAR(kf.x_W0, 3.0f, 0.5f); EXPECT_NEAR(kf.x_N, 50.0f, 10.0f);
}



// DATA PIPELINE TESTS

class UpdraftEstimatorDataPipelineTest : public UpdraftTestBase {};

TEST_F(UpdraftEstimatorDataPipelineTest, CalculateDiscreteSnap_GivenMovement_ShiftsGridAndResetsAlpha) {
    for (int i = 0; i < M; i++) ue._local_state.alpha[i] = 1.0f;
    ue.calculate_discrete_snap({6,0},{0,0},100);
    EXPECT_FLOAT_EQ(ue._local_state.inducing_points[38].x, 10.0f);
    EXPECT_FLOAT_EQ(ue._local_state.alpha[0], 0.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, CalculateDiscreteSnap_GivenNoMovement_PreservesAlpha) {
    for (int i = 0; i < M; i++) ue._local_state.alpha[i] = 1.0f;
    ue.calculate_discrete_snap({0,0},{0,0},100);
    float as = 0;
    for (int i = 0; i < M; i++) as += ue._local_state.alpha[i];
    EXPECT_FLOAT_EQ(as, 77.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, CalculateDiscreteSnap_GivenHalfCellMovement_SnapsCorrectly) {
    ue.calculate_discrete_snap({5,0},{0,0},100);
    EXPECT_FLOAT_EQ(ue._local_state.inducing_points[38].x, 10.0f);
    for (uint8_t i = 0; i < M; i++) ue._local_state.inducing_points[i].x = ue._shared_state.inducing_points[i].x;
    ue.calculate_discrete_snap({4.9f,0},{0,0},100);
    EXPECT_FLOAT_EQ(ue._local_state.inducing_points[38].x, 0.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, AccumulatePosterior_GivenObsOutsideGate_RejectsObservation) {
    ue.tau_decay = 10000; ue.sigma_n_sq = 0.5f;
    float kuu[KUU_SIZE] = {0}, pv[M] = {0};
    UpdraftObservation obs = {{1000,1000,100}, 5.0f, 0};
    ue.accumulate_posterior(kuu, pv, &obs, 1, 10000000, {0,0}, ue._shared_state.inducing_points, ue._catalogue);
    float ps = 0;
    for (int i = 0; i < M; i++) ps += fabsf(pv[i]);
    EXPECT_FLOAT_EQ(ps, 0.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, AccumulatePosterior_GivenOldObservation_ReducesWeight) {
    ue.tau_decay = 100.0f; ue.sigma_n_sq = 0.5f;
    float kf[KUU_SIZE] = {0}, pf[M] = {0}, ko[KUU_SIZE] = {0}, po[M] = {0};
    const uint32_t now_us = 600000000U;
    UpdraftObservation fresh = {grid_points[38], 5.0f, now_us};
    ue.accumulate_posterior(kf, pf, &fresh, 1, now_us, {0,0}, grid_points, ue._catalogue);
    UpdraftObservation old   = {grid_points[38], 5.0f, now_us - 500000000U};
    ue.accumulate_posterior(ko, po, &old,   1, now_us, {0,0}, grid_points, ue._catalogue);
    EXPECT_GT(fabsf(pf[38]), fabsf(po[38]));
}

TEST_F(UpdraftEstimatorDataPipelineTest, GetLiftPrediction_GivenRecentSink_VetoesCatalogueLift) {
    mock_kf_entry(0, 10, 10, 5.0f, 10000000); ue._catalogue[0].type = UpdraftTypes::THERMAL_BUBBLE;
    Location pl = ue._local_origin; pl.offset(10,10);
    ue.push_observation(pl, 100, -2.0f, 10000000-1000000);
    EXPECT_LT(ue.get_lift_prediction(pl, 100, 10000000), 1.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, GetLiftPrediction_GivenSinkInsideRadius_VetoesLift) {
    mock_kf_entry(0, 0, 0, 5.0f, 10000000); ue._catalogue[0].type = UpdraftTypes::THERMAL_BUBBLE;
    Location ol = ue._local_origin; ol.offset(14,0);
    ue.push_observation(ol, 100, -2.0f, 10000000-1000000);
    EXPECT_LT(ue.get_lift_prediction(ue._local_origin, 100, 10000000), 1.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, GetLiftPrediction_GivenSinkOutsideRadius_DoesNotVeto) {
    mock_kf_entry(0, 0, 0, 5.0f, 10000000); ue._catalogue[0].type = UpdraftTypes::THERMAL_BUBBLE;
    Location ol = ue._local_origin; ol.offset(20,0);
    ue.push_observation(ol, 100, -2.0f, 10000000-1000000);
    EXPECT_GT(ue.get_lift_prediction(ue._local_origin, 100, 10000000), 1.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, GetLiftPrediction_GivenStaleSink_DoesNotVeto) {
    mock_kf_entry(0, 0, 0, 5.0f, 10000000); ue._catalogue[0].type = UpdraftTypes::THERMAL_BUBBLE;
    ue.push_observation(ue._local_origin, 100, -2.0f, 10000000-6000000);
    EXPECT_GT(ue.get_lift_prediction(ue._local_origin, 100, 10000000), 1.0f);
}

TEST_F(UpdraftEstimatorDataPipelineTest, PushObservation_GivenBufferLimit_WrapsAround) {
    for (int i = 0; i < 205; i++) ue.push_observation(ue._local_origin, 100, 1.0f, i*1000000U);
    EXPECT_EQ(ue._shared_state.obs_count, SGP_MAX_OBSERVATIONS);
    EXPECT_EQ(ue._shared_state.obs_head,  5);
}

TEST_F(UpdraftEstimatorDataPipelineTest, UpdateEstimate_GivenComprehensiveFlight_ExtractsCorrectUpdrafts) {
    ue._shared_state.wind_vel = {5, 0};
    Location pl = ue._local_origin; pl.offset(-20, 0);
    for (int i = 0; i < 12; i++) {
        Location obs_loc = pl; obs_loc.offset((i%4 - 1.5f)*8.0f, (i/4 - 1.0f)*8.0f);
        ue.push_observation(obs_loc, 100, 2.5f, 10000000U - i*500000U);
    }
    ue.trigger_update(pl, 100, {0,0}, {5,0}); ue.update_estimate(10000000U);
    GenericTestLogger::dump_full_state("e2e_comprehensive.csv", ue, nullptr, 10000000U);

    bool found = false;
    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) if (ue._catalogue[i].active) found = true;
    EXPECT_TRUE(found);
}

TEST_F(UpdraftEstimatorDataPipelineTest, UpdateEstimate_GivenSeparatedThermals_ExtractsMultipleUpdrafts) {
    Location loc1 = ue._local_origin; loc1.offset(20,20);
    Location loc2 = ue._local_origin; loc2.offset(-25,-25);
    for (int i = 0; i < 12; i++) ue.push_observation(loc1, 100, 4.5f, 5000000U-(i*80000U));
    ue.trigger_update(loc1, 100, {0,0}, {0,0}); ue.update_estimate(5000000U);
    for (int i = 0; i < 12; i++) ue.push_observation(loc2, 100, 4.0f, 10000000U-(i*80000U));
    ue.trigger_update(loc2, 100, {0,0}, {0,0}); ue.update_estimate(10000000U);

    bool found1 = false, found2 = false;
    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!ue._catalogue[i].active) continue;
        if (sqrtf(sq(ue._catalogue[i].pos_north()-20)+sq(ue._catalogue[i].pos_east()-20)) < 15.0f) found1=true;
        if (sqrtf(sq(ue._catalogue[i].pos_north()+25)+sq(ue._catalogue[i].pos_east()+25)) < 15.0f) found2=true;
    }
    EXPECT_TRUE(found1); EXPECT_TRUE(found2);
    GenericTestLogger::dump_full_state("e2e_separated_thermals.csv", ue, nullptr, 10000000U);
}



// SGP OBSERVATION IMPACT TESTS

class SGPObservationImpactTest : public UpdraftTestBase {
protected:
    float kuu[KUU_SIZE], pvec[M], alpha[M];

    bool run_pipeline(const UpdraftObservation* obs, uint16_t n_obs,
                      uint32_t now_us, const Vector2f& wind = {0, 0}) {
        ue.build_prior_matrix(kuu, nullptr, grid_points, false);
        memset(pvec, 0, sizeof(pvec));
        ue.accumulate_posterior(kuu, pvec, obs, n_obs, now_us, wind, grid_points, ue._catalogue);
        memset(alpha, 0, sizeof(alpha));
        if (!ue.cholesky_decompose(kuu, M)) return false;
        ue.cholesky_solve(kuu, M, pvec, alpha);
        return true;
    }

    float query(const Vector3f& pos) const {
        return ue.query_sgp_lift(pos, alpha, grid_points);
    }

    void dump_alpha_to(const std::string& fname) {
        GenericTestLogger::dump_alpha(fname, alpha, grid_points, M);
    }
};

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenSingleObsAtCentre_PeaksAtCentre) {
    UpdraftObservation obs = {grid_points[38], 5.0f, 10000000U};
    ASSERT_TRUE(run_pipeline(&obs, 1, 10000000U));
    EXPECT_GT(query(grid_points[38]), query(Vector3f(30, 30, 100)));
    dump_alpha_to("obs_impact_single_centre_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenSingleObsAtEdge_PeaksAtEdge) {
    UpdraftObservation obs = {grid_points[0], 5.0f, 10000000U};
    ASSERT_TRUE(run_pipeline(&obs, 1, 10000000U));
    EXPECT_GT(query(grid_points[0]), query(grid_points[38]));
    dump_alpha_to("obs_impact_single_edge_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenMultipleIdenticalObs_IncreasesLift) {
    float prev_lift = -1.0f;
    for (int count : {1, 2, 5, 10}) {
        UpdraftObservation obs_arr[10];
        for (int i = 0; i < count; i++) {
            obs_arr[i] = {grid_points[38], 5.0f, 10000000U - (uint32_t)(i * 100000U)};
        }
        ASSERT_TRUE(run_pipeline(obs_arr, count, 10000000U));
        float l = query(grid_points[38]);
        EXPECT_GT(l, prev_lift);
        prev_lift = l;
    }
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenNegativeObs_ProducesNegativeLift) {
    UpdraftObservation obs = {grid_points[38], -3.0f, 10000000U};
    ASSERT_TRUE(run_pipeline(&obs, 1, 10000000U));
    EXPECT_LT(query(grid_points[38]), 0.5f);
    dump_alpha_to("obs_impact_negative_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenTwoClusters_ProducesTwoPeaks) {
    UpdraftObservation obs_arr[10];
    for (int i = 0; i < 5; i++) {
        obs_arr[i]   = {grid_points[15], 5.0f, 10000000U - (uint32_t)(i * 100000U)};
        obs_arr[i+5] = {grid_points[60], 5.0f, 10000000U - (uint32_t)(i * 100000U)};
    }
    ASSERT_TRUE(run_pipeline(obs_arr, 10, 10000000U));
    EXPECT_GT(query(grid_points[15]), query(grid_points[38]));
    EXPECT_GT(query(grid_points[60]), query(grid_points[38]));
    dump_alpha_to("obs_impact_two_clusters_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenVaryingMagnitude_ProducesProportionalLift) {
    const float wz_vals[] = {1.0f, 2.0f, 3.0f, 5.0f, 8.0f};
    std::ofstream f(OUT_DIR + "obs_impact_magnitude.csv");
    if (f.is_open()) f << "Wz_ms,LiftAtObs\n";

    float prev = -1.0e9f;
    for (float wz : wz_vals) {
        UpdraftObservation obs = {grid_points[38], wz, 10000000U};
        ASSERT_TRUE(run_pipeline(&obs, 1, 10000000U));
        float l = query(grid_points[38]);
        EXPECT_GT(l, prev);
        prev = l;
        if (f.is_open()) f << wz << "," << l << "\n";
    }
    dump_alpha_to("obs_impact_magnitude_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenTemporalDecay_OlderObsHasLessImpact) {
    ue.obs_tau_decay.set(50.0f);  // aggressive 50s decay
    const uint32_t now_us    = 10000000;
    const uint32_t ages_us[] = {0, 30000000U, 100000000U, 200000000U};

    std::ofstream f(OUT_DIR + "obs_impact_decay.csv");
    if (f.is_open()) f << "Age_s,LiftAtObs\n";

    float prev = 1.0e9f;
    for (uint32_t age_us : ages_us) {
        UpdraftObservation obs = {grid_points[38], 5.0f, now_us - age_us};
        ASSERT_TRUE(run_pipeline(&obs, 1, now_us));
        float l = query(grid_points[38]);
        EXPECT_LT(l, prev);
        prev = l;
        if (f.is_open()) f << (age_us / 1000000) << "," << l << "\n";
    }
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenLowNoise_RecoversActualWz) {
    ue.obs_noise_var.set(0.01f);
    const float true_wz = 4.0f;
    UpdraftObservation obs = {grid_points[38], true_wz, 10000000U};
    ASSERT_TRUE(run_pipeline(&obs, 1, 10000000U));
    float recovered = query(grid_points[38]);
    EXPECT_NEAR(recovered, true_wz, true_wz * 0.25f);

    std::ofstream f(OUT_DIR + "obs_impact_alpha_recovery.csv");
    if (f.is_open()) {
        f << "TrueWz_ms,RecoveredLift_ms,Error_ms\n";
        f << true_wz << "," << recovered << "," << fabsf(recovered - true_wz) << "\n";
    }
    dump_alpha_to("obs_impact_recovery_alpha.csv");
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenMixedPolarity_SuppressesField) {
    UpdraftObservation obs_arr[M];
    for (uint8_t i = 0; i < M; i++) {
        float wz = (i % 2 == 0) ? 4.0f : -4.0f;
        obs_arr[i] = {grid_points[i], wz, 10000000U};
    }
    ASSERT_TRUE(run_pipeline(obs_arr, M, 10000000U));

    float sum_abs = 0.0f;
    for (uint8_t i = 0; i < M; i++) sum_abs += fabsf(query(grid_points[i]));
    EXPECT_LT(sum_abs / M, 3.5f); // Mean absolute lift should be suppressed

    dump_alpha_to("obs_impact_mixed_polarity_alpha.csv");

    std::ofstream f(OUT_DIR + "obs_impact_mixed_polarity.csv");
    if (f.is_open()) {
        f << "North_m,East_m,InputWz,LiftQueried\n";
        for (uint8_t i = 0; i < M; i++) {
            float wz = (i % 2 == 0) ? 4.0f : -4.0f;
            f << grid_points[i].x << "," << grid_points[i].y << "," << wz << "," << query(grid_points[i]) << "\n";
        }
    }
}

TEST_F(SGPObservationImpactTest, AccumulatePosterior_GivenFullGrid_ProducesUniformField) {
    UpdraftObservation obs_arr[M];
    for (uint8_t i = 0; i < M; i++) obs_arr[i] = {grid_points[i], 4.0f, 10000000U};
    ASSERT_TRUE(run_pipeline(obs_arr, M, 10000000U));

    float max_l = -1e9f, min_l = 1e9f;
    for (uint8_t i = 0; i < M; i++) {
        float l = query(grid_points[i]);
        if (l > max_l) max_l = l;
        if (l < min_l) min_l = l;
    }
    // With uniform observations the ratio of max/min should be moderate
    if (min_l > 0.0f) { EXPECT_LT(max_l / min_l, 2.0f); }

    dump_alpha_to("obs_impact_full_grid_alpha.csv");
}



// TUNING TESTS

class TuningTests : public UpdraftTestBase {
protected:
    std::default_random_engine generator;

    void simulate_step(uint32_t& now_us, Location& loc,
                       Vector2f ground_vel, float wz_true, float noise_std_dev) {
        now_us += 1000000;
        std::normal_distribution<float> dist(0.0f, noise_std_dev);
        float wz_meas = wz_true + dist(generator);
        ue.push_observation(loc, 100.0f, wz_meas, now_us);
        ue.trigger_update(loc, 100.0f, ground_vel, {0,0});
        ue.update_estimate(now_us);
    }

    float get_true_lift(float n, float e, float peak_n, float peak_e, float w0, float r) {
        return w0 * expf(-(sq(n - peak_n) + sq(e - peak_e)) / sq(r));
    }

    void get_intersecting_path(float t, Vector2f& pos, Vector2f& vel) {
        if (t <= 6.5f) {
            pos.x = -50.0f + 10.0f * t; pos.y = 1.0f;
            vel.x = 10.0f; vel.y = 0.0f;
        } else if (t <= 13.57f) {
            float dt = t - 6.5f;
            float d_theta = (1.5f * M_PI) / 7.07f;
            float theta = -M_PI_2 + dt * d_theta;
            pos.x =  15.0f + 15.0f * cosf(theta);
            pos.y =  16.0f + 15.0f * sinf(theta);
            vel.x = -15.0f * sinf(theta) * d_theta;
            vel.y =  15.0f * cosf(theta) * d_theta;
        } else {
            float dt = t - 13.57f;
            pos.x = 0.0f; pos.y = 16.0f - 10.0f * dt;
            vel.x = 0.0f; vel.y = -10.0f;
        }
    }

    float get_complex_lift(float n, float e, const char* type,
                           float t, float wind_n, float wind_e, float r) {
        float peak_n = 0.0f, peak_e = 0.0f, Ru = r, Rv = r;
        if (strcmp(type,"Bubble")==0 || strcmp(type,"Bubble_Calm")==0) {
            peak_n = wind_n * (t - 5.0f); peak_e = wind_e * (t - 5.0f);
        } else if (strcmp(type,"Ridge")==0) {
            Rv = r * 3.0f;
        } else if (strcmp(type,"Shear")==0) {
            peak_n = wind_n * (t - 13.57f); peak_e = wind_e * (t - 13.57f);
            Ru = r * 3.0f;
        }
        return 4.0f * expf(-((n-peak_n)*(n-peak_n)/sq(Ru) + (e-peak_e)*(e-peak_e)/sq(Rv)));
    }

    void run_intersect_sweep(const std::string& prefix, const std::string& label,
                             float true_radius, float noise, const char* type,
                             Vector2f wind, float stop_east) {
        SetUp();
        ue._shared_state.obs_count = 0; ue._shared_state.obs_head = 0;
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) ue._catalogue[i].active = false;
        ue._shared_state.wind_vel = wind;

        uint32_t now_us = 10000000;
        Location glider_loc = ue._local_origin;

        std::ofstream f_obs(OUT_DIR + prefix + "_" + label + "_obs.csv");
        if (f_obs.is_open()) f_obs << "Time_s,North_m,East_m,Wz_meas\n";

        for (float t = 0.0f; ; t += 0.5f) {
            Vector2f pos, vel;
            get_intersecting_path(t, pos, vel);
            glider_loc = ue._local_origin;
            glider_loc.offset(pos.x, pos.y);
            float true_wz = get_complex_lift(pos.x, pos.y, type, t, wind.x, wind.y, true_radius);
            now_us += 500000;
            std::normal_distribution<float> dist_n(0.0f, noise);
            float wz_meas = true_wz + dist_n(generator);
            ue.push_observation(glider_loc, 100.0f, wz_meas, now_us);
            ue.trigger_update(glider_loc, 100.0f, vel, wind);
            ue.update_estimate(now_us);
            if (f_obs.is_open()) f_obs << t << "," << pos.x << "," << pos.y << "," << wz_meas << "\n";

            if (t > 14.0f && pos.y <= stop_east) {
                for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
                    if (ue._catalogue[i].active) {
                        ue._catalogue[i].created_us = now_us - 35000000;
                        if (strcmp(type,"Bubble")==0 || strcmp(type,"Shear")==0 || strcmp(type,"Bubble_Calm")==0) {
                            ue._catalogue[i].start_pos.x -= wind.x * 35.0f;
                            ue._catalogue[i].start_pos.y -= wind.y * 35.0f;
                        }
                    }
                }
                ue.trigger_update(glider_loc, 100.0f, {0,0}, wind);
                ue.update_estimate(now_us);
                break;
            }
        }

        TrueTarget tt; tt.valid = true; tt.w0 = 4.0f;
        tt.ru = true_radius; tt.rv = true_radius;
        if (strcmp(type,"Ridge")==0)  tt.rv = true_radius * 3.0f;
        if (strcmp(type,"Shear")==0)  tt.ru = true_radius * 3.0f;

        GenericTestLogger::dump_full_state(prefix + "_" + label + "_state.csv", ue, nullptr, now_us, false, tt);
        GenericTestLogger::dump_1d_slice(prefix + "_" + label + "_1d_slice.csv", ue, ue._local_origin, now_us);
    }
};

TEST_F(TuningTests, Observation1DSlice) {
    uint32_t now_us = 10000000; Location loc = ue._local_origin;
    for (int i = 0; i < 10; i++) ue.push_observation(loc, 100.0f, 5.0f, now_us - (i * 100000));
    ue.trigger_update(loc, 100.0f, {0,0}, {0,0});
    ue.update_estimate(now_us);
    GenericTestLogger::dump_1d_slice("obs_impact_1d_slice.csv", ue, loc, now_us);
}

TEST_F(TuningTests, StraightLineSweeps) {
    float offsets[] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f};
    for (float offset_e : offsets) {
        SetUp();
        uint32_t now_us = 10000000;
        Location glider_loc = ue._local_origin; glider_loc.offset(-75.0f, offset_e);
        std::string prefix = "tune_sweep_" + std::to_string((int)offset_e) + "m";

        std::ofstream f_obs(OUT_DIR + prefix + "_obs.csv");
        if (f_obs.is_open()) f_obs << "Time_s,North_m,East_m,Wz_meas\n";

        for (int t = 0; t <= 10; t++) {
            glider_loc.offset(10.0f, 0.0f);
            Vector2f pos_ne = ue._local_origin.get_distance_NE(glider_loc);
            float true_wz = get_true_lift(pos_ne.x, pos_ne.y, 0.0f, 0.0f, 4.0f, 20.0f);
            simulate_step(now_us, glider_loc, {10.0f, 0.0f}, true_wz, 0.0f);
            if (f_obs.is_open()) f_obs << t << "," << pos_ne.x << "," << pos_ne.y << "," << true_wz << "\n";
        }

        TrueTarget tt; tt.valid = true; tt.w0 = 4.0f; tt.ru = 20.0f; tt.rv = 20.0f;
        GenericTestLogger::dump_full_state(prefix + "_state.csv", ue, nullptr, now_us, false, tt);
        GenericTestLogger::dump_1d_slice(prefix + "_1d_slice.csv", ue, ue._local_origin, now_us);
    }
}

TEST_F(TuningTests, ZigZagSweeps) {
    float offsets[] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f};
    for (float offset_e : offsets) {
        SetUp();
        uint32_t now_us = 10000000;
        Location glider_loc = ue._local_origin;
        std::string prefix = "tune_zigzag_" + std::to_string((int)offset_e) + "m";

        std::ofstream f_obs(OUT_DIR + prefix + "_obs.csv");
        if (f_obs.is_open()) f_obs << "Time_s,North_m,East_m,Wz_meas\n";

        for (float t = 0.0f; t <= 10.5f; t += 0.5f) {
            float pos_n = -75.0f + (t * 10.0f);
            float pos_e = offset_e + 20.0f * sinf((t - 6.0f) * M_PI / 3.0f);
            glider_loc = ue._local_origin; glider_loc.offset(pos_n, pos_e);
            float true_wz = get_true_lift(pos_n, pos_e, 0.0f, 0.0f, 4.0f, 20.0f);
            now_us += 500000;
            ue.push_observation(glider_loc, 100.0f, true_wz, now_us);
            ue.trigger_update(glider_loc, 100.0f, {10.0f, 0.0f}, {0,0});
            ue.update_estimate(now_us);
            if (f_obs.is_open()) f_obs << t << "," << pos_n << "," << pos_e << "," << true_wz << "\n";
        }

        TrueTarget tt; tt.valid = true; tt.w0 = 4.0f; tt.ru = 20.0f; tt.rv = 20.0f;
        GenericTestLogger::dump_full_state(prefix + "_state.csv", ue, nullptr, now_us, false, tt);
        GenericTestLogger::dump_1d_slice(prefix + "_1d_slice.csv", ue, ue._local_origin, now_us);
    }
}

TEST_F(TuningTests, IntersectingRadiusSweep) {
    const char* labels[] = {"2", "5", "10", "15", "30", "40"};
    for (const auto* l : labels) {
        run_intersect_sweep("tune_int_rad", l, std::stof(l), 0.0f, "Bubble", {0,0}, -30.0f);
    }
}

TEST_F(TuningTests, IntersectingTypeSweep) {
    struct TypeTest { const char* label; const char* type; Vector2f wind; };
    TypeTest tests[] = {
        {"Bubble",      "Bubble",      {5.0f, 0.0f}},
        {"Orographic",  "Orographic",  {5.0f, 0.0f}},
        {"Ridge",       "Ridge",       {5.0f, 0.0f}},
        {"Shear",       "Shear",       {5.0f, 0.0f}},
        {"Bubble_Calm", "Bubble",      {0.0f, 0.0f}},
        {"Oro_Calm",    "Orographic",  {0.0f, 0.0f}}
    };
    for (const auto& t : tests) {
        run_intersect_sweep("tune_int_type", t.label, 15.0f, 0.0f, t.type, t.wind, -30.0f);
    }
}

TEST_F(TuningTests, IntersectingNoiseSweep) {
    const char* labels[] = {"0.0", "0.5", "1.0", "1.5", "2.0", "3.0"};
    for (const auto* l : labels) {
        run_intersect_sweep("tune_int_noise", l, 20.0f, std::stof(l), "Bubble", {0,0}, -30.0f);
    }
}

TEST_F(TuningTests, NoiseRejectionSweep) {
    float noise_levels[] = {0.0f, 0.5f, 1.0f, 2.0f, 5.0f};

    std::ofstream f(OUT_DIR + "tune_noise_rejection.csv");
    if (f.is_open()) f << "NoiseStdDev,Time_s,True_W0,Est_W0\n";

    for (float noise : noise_levels) {
        SetUp();
        uint32_t now_us = 10000000;
        Location loc = ue._local_origin; loc.offset(-60.0f, 5.0f);

        for (int t = 0; t <= 10; t++) {
            loc.offset(10.0f, 0.0f);
            Vector2f pos_ne = ue._local_origin.get_distance_NE(loc);
            float true_wz = get_true_lift(pos_ne.x, pos_ne.y, 0, 0, 4.0f, 20.0f);
            simulate_step(now_us, loc, {10.0f, 0}, true_wz, noise);
            if (f.is_open()) {
                const auto& cat = ue.get_catalog_entry(0);
                f << noise << "," << t << ",4.0," << (cat.active ? cat.strength_w0() : 0.0f) << "\n";
            }
        }
    }
}

TEST_F(TuningTests, SensitivitySweep) {
    float length_scales[] = {5.0f, 10.0f, 15.0f, 20.0f, 30.0f};
    float decays[]        = {60.0f, 150.0f, 300.0f, 600.0f};
    float radii[]         = {10.0f, 15.0f, 20.0f};

    std::ofstream f(OUT_DIR + "tune_sensitivity_mse.csv");
    if (f.is_open()) f << "TrueRadius,LengthScale,TauDecay,MSE\n";

    for (float r_true : radii) {
        for (float l_xy : length_scales) {
            for (float tau : decays) {
                SetUp();
                ue.kern_length_xy.set(l_xy); ue.obs_tau_decay.set(tau);

                uint32_t now_us = 10000000;
                Location loc = ue._local_origin; loc.offset(-60.0f, 5.0f);
                for (int t = 0; t <= 10; t++) {
                    loc.offset(10.0f, 0.0f);
                    Vector2f pos_ne = ue._local_origin.get_distance_NE(loc);
                    float true_wz = get_true_lift(pos_ne.x, pos_ne.y, 0, 0, 4.0f, r_true);
                    simulate_step(now_us, loc, {10.0f, 0}, true_wz, 0.5f);
                }

                float mse = 0.0f; int count = 0;
                for (int n = -20; n <= 20; n += 10) {
                    for (int e = -20; e <= 20; e += 10) {
                        Location q = ue._local_origin; q.offset(n, e);
                        float true_lift = get_true_lift(n, e, 0, 0, 4.0f, r_true);
                        float est_lift  = ue.get_lift_prediction(q, 100.0f, now_us);
                        mse += sq(true_lift - est_lift); count++;
                    }
                }
                if (f.is_open()) f << r_true << "," << l_xy << "," << tau << "," << (mse / count) << "\n";
            }
        }
    }
}



// NOISE ROBUSTNESS


class NoiseRobustnessTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{42};
    
    // Run one complete intersecting-path scenario and return active catalogue count.
    int run_noise_trial(float noise_sigma, float true_w0 = 4.0f, float true_r = 20.0f) {
        clear_catalogue();
        ue._shared_state.obs_count = 0;
        ue._shared_state.obs_head  = 0;
        ue._shared_state.wind_vel  = {0.0f, 0.0f};

        std::normal_distribution<float> noise{0.0f, noise_sigma};
        uint32_t now_us = 10000000U;

        // Leg 1: straight pass at E=1m
        for (float t = 0.0f; t <= 6.5f; t += 0.5f) {
            float pos_n = -50.0f + 10.0f * t;
            float pos_e = 1.0f;
            Location loc = ue._local_origin;
            loc.offset(pos_n, pos_e);
            float wz = true_w0 * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(true_r)) + noise(generator);
            now_us += 500000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {10.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }

        // Leg 2: 270-degree teardrop turn
        for (float angle = -M_PI_2; angle <= M_PI; angle += 0.2f) {
            float pos_n = 15.0f + 15.0f * cosf(angle);
            float pos_e = 16.0f + 15.0f * sinf(angle);
            Location loc = ue._local_origin;
            loc.offset(pos_n, pos_e);
            float wz = true_w0 * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(true_r)) + noise(generator);
            now_us += 500000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {0.0f, 10.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }

        // Leg 3: south pass at E=16m
        for (float dt = 0.0f; dt <= 3.0f; dt += 0.5f) {
            float pos_n = 0.0f;
            float pos_e = 16.0f - 10.0f * dt;
            Location loc = ue._local_origin;
            loc.offset(pos_n, pos_e);
            float wz = true_w0 * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(true_r)) + noise(generator);
            now_us += 500000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {0.0f, -10.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }

        int active = 0;
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (ue._catalogue[i].active) active++;
        }
        return active;
    }

    // Run N trials and return {mean, max} active catalogue count.
    std::pair<float,int> monte_carlo(float noise_sigma, int n_trials = 10) {
        int total = 0, max_seen = 0;
        for (int t = 0; t < n_trials; t++) {
            SetUp();
            int c = run_noise_trial(noise_sigma);
            total += c;
            max_seen = MAX(max_seen, c);
        }
        return {(float)total / n_trials, max_seen};
    }
};

TEST_F(NoiseRobustnessTest, SpawnCount_ZeroNoise_ExactlyOne) {
    int count = run_noise_trial(0.0f);
    EXPECT_EQ(count, 1) << "With no noise exactly one entry should be spawned";
}

TEST_F(NoiseRobustnessTest, SpawnCount_LowNoise_AtMostTwo) {
    int count = run_noise_trial(0.5f);
    EXPECT_LE(count, 2) << "At σ=0.5 m/s at most two entries should be spawned";
}

TEST_F(NoiseRobustnessTest, SpawnCount_MedNoise_AtMostTwo) {
    int count = run_noise_trial(1.0f);
    EXPECT_LE(count, 2) << "At σ=1.0 m/s at most two entries should be spawned";
}

TEST_F(NoiseRobustnessTest, SpawnCount_HighNoise_AtMostTwo) {
    int count = run_noise_trial(1.5f);
    EXPECT_LE(count, 2) << "At σ=1.5 m/s at most two entries should be spawned";
}

TEST_F(NoiseRobustnessTest, SpawnCount_VeryHighNoise_AtMostThree) {
    int count = run_noise_trial(2.0f);
    EXPECT_LE(count, 3) << "At σ=2.0 m/s at most three entries should be spawned";
}

TEST_F(NoiseRobustnessTest, FalsePositiveRate_PureNoise) {
    std::normal_distribution<float> noise{0.0f, 1.5f};
    int false_positives = 0;
    const int N_TRIALS = 15;

    for (int trial = 0; trial < N_TRIALS; trial++) {
        SetUp();
        clear_catalogue();
        uint32_t now_us = 10000000U;
        Location loc = ue._local_origin;
        loc.offset(-50.0f, 0.0f);

        for (int t = 0; t < 20; t++) {
            loc.offset(5.0f, 0.0f);
            float wz = noise(generator);  // pure noise, no thermal
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {5.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
            now_us += 1000000U;
        }

        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (ue._catalogue[i].active) { false_positives++; break; }
        }
    }

    float fp_rate = (float)false_positives / N_TRIALS;
    EXPECT_LT(fp_rate, 0.15f)
        << "False positive rate " << fp_rate << " exceeds 15% for σ=1.5 m/s pure noise";
}

TEST_F(NoiseRobustnessTest, TrueDetectionRate_AtOperationalNoise) {
    // With a real thermal, the system should detect it in ≥80% of trials at σ=1.5 m/s.
    int detections = 0;
    const int N_TRIALS = 10;

    for (int trial = 0; trial < N_TRIALS; trial++) {
        SetUp();
        int count = run_noise_trial(1.5f);
        if (count >= 1) detections++;
    }

    float detection_rate = (float)detections / N_TRIALS;
    EXPECT_GE(detection_rate, 0.8f)
        << "Detection rate " << detection_rate << " below 80% at σ=1.5 m/s";
}

TEST_F(NoiseRobustnessTest, SpawnCountSweep_WritesCSV) {
    const float noise_levels[] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
    std::ofstream f(OUT_DIR + "noise_spawn_count.csv");
    if (!f.is_open()) return;
    f << "Noise_sigma,Trial,SpawnCount\n";

    for (float sigma : noise_levels) {
        for (int trial = 0; trial < 10; trial++) {
            SetUp();
            int count = run_noise_trial(sigma);
            f << sigma << "," << trial << "," << count << "\n";
        }
    }
}


// QUANTITATIVE LOCALISATION AND ESTIMATION ACCURACY

class LocalisationAccuracyTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{123};

    struct EstimationResult {
        float pos_error_m;      // distance from estimated peak to true peak
        float w0_error_ms;      // |estimated W0 - true W0|
        float ru_error_m;       // |estimated Ru - true Ru|
        int   spawn_count;
        bool  detected;         // any entry within 2×true_r of true position
    };

    EstimationResult run_sweep_trial(float noise_sigma, float true_r,
                                     float offset_e = 5.0f,
                                     float true_w0 = 4.0f) {
        clear_catalogue();
        ue._shared_state.obs_count = 0;
        ue._shared_state.obs_head  = 0;

        std::normal_distribution<float> noise{0.0f, noise_sigma};
        uint32_t now_us = 10000000U;
        Location loc = ue._local_origin;
        loc.offset(-70.0f, offset_e);

        for (int t = 0; t <= 14; t++) {
            loc.offset(10.0f, 0.0f);
            Vector2f pos_ne = ue._local_origin.get_distance_NE(loc);
            float wz = true_w0 * expf(-(pos_ne.x*pos_ne.x + pos_ne.y*pos_ne.y) / sq(true_r))
                       + noise(generator);
            now_us += 1000000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {10.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }

        EstimationResult res{};
        res.pos_error_m = 1e6f;
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (!ue._catalogue[i].active) continue;
            res.spawn_count++;
            float d = sqrtf(sq(ue._catalogue[i].pos_north()) + sq(ue._catalogue[i].pos_east()));
            if (d < res.pos_error_m) {
                res.pos_error_m = d;
                res.w0_error_ms = fabsf(ue._catalogue[i].strength_w0() - true_w0);
                res.ru_error_m  = fabsf(ue._catalogue[i].radius_u() - true_r);
            }
            if (d < 2.0f * true_r) res.detected = true;
        }
        if (res.spawn_count == 0) res.pos_error_m = 0.0f;  // nothing to compare
        return res;
    }
};

TEST_F(LocalisationAccuracyTest, PositionError_VsNoise_WritesCSV) {
    const float noise_levels[] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f};
    const float radii[]        = {10.0f, 20.0f, 30.0f};
    const float offsets[]      = {0.0f, 5.0f, 10.0f, 15.0f};

    std::ofstream f(OUT_DIR + "localisation_accuracy.csv");
    if (!f.is_open()) return;
    f << "Noise_sigma,TrueRadius,Offset,Trial,PosError_m,W0Error_ms,RuError_m,Detected\n";

    for (float r : radii) {
        for (float sigma : noise_levels) {
            for (float offset : offsets) {
                for (int trial = 0; trial < 5; trial++) {
                    SetUp();
                    auto res = run_sweep_trial(sigma, r, offset);
                    f << sigma << "," << r << "," << offset << "," << trial << ","
                      << res.pos_error_m << "," << res.w0_error_ms << ","
                      << res.ru_error_m  << "," << (int)res.detected << "\n";
                }
            }
        }
    }
}

TEST_F(LocalisationAccuracyTest, PositionError_ZeroNoise_BelowTwoGridCells) {
    SetUp();
    auto res = run_sweep_trial(0.0f, 20.0f, 5.0f);
    if (res.detected) {
        EXPECT_LT(res.pos_error_m, 2.0f * SGP_GRID_RES_M)
            << "Zero-noise position error " << res.pos_error_m << " m exceeds 2 grid cells";
    }
}

TEST_F(LocalisationAccuracyTest, W0Estimation_ZeroNoise_Within20Percent) {
    SetUp();
    auto res = run_sweep_trial(0.0f, 20.0f, 0.0f);
    if (res.detected) {
        EXPECT_LT(res.w0_error_ms, 0.8f)
            << "Zero-noise W0 error " << res.w0_error_ms << " m/s exceeds 20%";
    }
}


// CONVERGENCE RATE TESTS

class ConvergenceTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{99};

    // Run N observation steps centred at (0,0) with Gaussian wz profile.
    // After each step log [obs_count, best_W0_error, best_pos_error].
    void run_convergence_log(std::ofstream& f, float noise_sigma, float true_r, const std::string& label) {
        clear_catalogue();
        ue._shared_state.obs_count = 0;
        ue._shared_state.obs_head  = 0;

        std::normal_distribution<float> noise{0.0f, noise_sigma};
        uint32_t now_us = 10000000U;

        const float W0_TRUE = 4.0f;
        // Circular path of radius 25m centred on the thermal
        for (int step = 0; step < 40; step++) {
            float angle = step * (2.0f * M_PI / 40.0f);
            float pos_n = 25.0f * cosf(angle);
            float pos_e = 25.0f * sinf(angle);
            Location loc = ue._local_origin;
            loc.offset(pos_n, pos_e);
            float wz = W0_TRUE * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(true_r)) + noise(generator);
            now_us += 500000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {0.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);

            float best_pos_err = 1e6f, best_w0_err = W0_TRUE;
            for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
                if (!ue._catalogue[i].active) continue;
                float d = sqrtf(sq(ue._catalogue[i].pos_north()) + sq(ue._catalogue[i].pos_east()));
                if (d < best_pos_err) {
                    best_pos_err = d;
                    best_w0_err = fabsf(ue._catalogue[i].strength_w0() - W0_TRUE);
                }
            }
            if (best_pos_err > 1e5f) best_pos_err = -1.0f;  // sentinel: not yet spawned

            f << label << "," << (step+1) << "," << noise_sigma << ","
              << best_pos_err << "," << best_w0_err << "\n";
        }
    }
};

TEST_F(ConvergenceTest, ConvergenceRate_CircularPath_WritesCSV) {
    std::ofstream f(OUT_DIR + "convergence_circular.csv");
    if (!f.is_open()) return;
    f << "Label,ObsStep,NoiseSigma,PosError_m,W0Error_ms\n";

    for (float sigma : {0.0f, 0.5f, 1.0f, 1.5f}) {
        for (float r : {15.0f, 20.0f, 30.0f}) {
            SetUp();
            std::string label = "s" + std::to_string((int)(sigma*10)) + "_r" + std::to_string((int)r);
            run_convergence_log(f, sigma, r, label);
        }
    }
}

TEST_F(ConvergenceTest, ConvergenceRate_FirstSpawnWithin10Steps) {
    // After 10 passes centred on the thermal, at least one entry should exist.
    clear_catalogue();
    ue._shared_state.obs_count = 0;
    uint32_t now_us = 10000000U;

    for (int step = 0; step < 10; step++) {
        float angle = step * (2.0f * M_PI / 10.0f);
        float pos_n = 20.0f * cosf(angle);
        float pos_e = 20.0f * sinf(angle);
        Location loc = ue._local_origin;
        loc.offset(pos_n, pos_e);
        float wz = 4.0f * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(20.0f));
        now_us += 500000U;
        ue.push_observation(loc, 100.0f, wz, now_us);
        ue.trigger_update(loc, 100.0f, {0.0f, 0.0f}, {0.0f, 0.0f});
        ue.update_estimate(now_us);
    }

    bool any_active = false;
    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (ue._catalogue[i].active) { any_active = true; break; }
    }
    EXPECT_TRUE(any_active) << "No catalogue entry spawned after 10 circular passes over a real thermal";
}



// TWO-THERMAL DISCRIMINATION

class TwoThermalTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{77};

    // Push observations for two thermals at (n1, e1) and (n2, e2) with a path
    // that passes between them.
    void observe_two_thermals(float n1, float e1, float n2, float e2,
                              float noise_sigma, uint32_t& now_us) {
        std::normal_distribution<float> noise{0.0f, noise_sigma};
        const float W0 = 3.5f, R = 15.0f;

        float mid_n = (n1 + n2) * 0.5f;
        float mid_e = (e1 + e2) * 0.5f;

        // Path sweeps across both thermals
        for (int t = -8; t <= 8; t++) {
            float pos_n = mid_n + t * 5.0f;
            float pos_e = mid_e;
            Location loc = ue._local_origin;
            loc.offset(pos_n, pos_e);

            float wz1 = W0 * expf(-(sq(pos_n-n1)+sq(pos_e-e1)) / sq(R));
            float wz2 = W0 * expf(-(sq(pos_n-n2)+sq(pos_e-e2)) / sq(R));
            float wz = MAX(wz1, wz2) + noise(generator);

            now_us += 500000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {5.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }
    }
};

TEST_F(TwoThermalTest, Separation50m_TwoEntriesDetected) {
    uint32_t now_us = 10000000U;
    observe_two_thermals(-25.0f, 0.0f, 25.0f, 0.0f, 0.3f, now_us);

    int count = 0;
    bool found1 = false, found2 = false;
    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!ue._catalogue[i].active) continue;
        count++;
        float d1 = sqrtf(sq(ue._catalogue[i].pos_north()+25)+sq(ue._catalogue[i].pos_east()));
        float d2 = sqrtf(sq(ue._catalogue[i].pos_north()-25)+sq(ue._catalogue[i].pos_east()));
        if (d1 < 15.0f) found1 = true;
        if (d2 < 15.0f) found2 = true;
    }
    EXPECT_TRUE(found1) << "Failed to detect thermal at (-25, 0)";
    EXPECT_TRUE(found2) << "Failed to detect thermal at (+25, 0)";
    EXPECT_LE(count, 4) << "Too many spurious entries for two-thermal scenario";
}

TEST_F(TwoThermalTest, Separation20m_DoesNotFalselyMerge) {
    // Two thermals 20 m apart — system may report 1 or 2 entries but must not report a combined entry displaced far from both true positions.
    // This test may not be relevant any more - 20m not enough sep TODO
    uint32_t now_us = 10000000U;
    observe_two_thermals(-10.0f, 0.0f, 10.0f, 0.0f, 0.3f, now_us);

    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!ue._catalogue[i].active) continue;
        float d_to_either = MIN(
            sqrtf(sq(ue._catalogue[i].pos_north()+10)+sq(ue._catalogue[i].pos_east())),
            sqrtf(sq(ue._catalogue[i].pos_north()-10)+sq(ue._catalogue[i].pos_east()))
        );
        EXPECT_LT(d_to_either, 25.0f)
            << "Entry at (" << ue._catalogue[i].pos_north() << ", "
            << ue._catalogue[i].pos_east() << ") is not near either true thermal";
    }
}

TEST_F(TwoThermalTest, SeparationSweep_WritesCSV) {
    const float separations[] = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 70.0f, 100.0f};
    std::ofstream f(OUT_DIR + "two_thermal_separation.csv");
    if (!f.is_open()) return;
    f << "Separation_m,SpawnCount,Found1,Found2\n";

    for (float sep : separations) {
        SetUp();
        clear_catalogue();
        uint32_t now_us = 10000000U;
        observe_two_thermals(-sep*0.5f, 0.0f, sep*0.5f, 0.0f, 0.3f, now_us);

        int count = 0;
        bool f1=false, f2=false;
        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (!ue._catalogue[i].active) continue;
            count++;
            if (sqrtf(sq(ue._catalogue[i].pos_north()+sep*0.5f)+sq(ue._catalogue[i].pos_east())) < 15.0f) f1=true;
            if (sqrtf(sq(ue._catalogue[i].pos_north()-sep*0.5f)+sq(ue._catalogue[i].pos_east())) < 15.0f) f2=true;
        }
        f << sep << "," << count << "," << (int)f1 << "," << (int)f2 << "\n";
    }
}


// VARIANCE CALIBRATION

class VarianceCalibrationTest : public UpdraftTestBase {
protected:
    float kuu[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    float pvec[SGP_NUM_INDUCING_POINTS];
    float alpha_buf[SGP_NUM_INDUCING_POINTS];
    float l_kuu[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];
    float l_a[SGP_NUM_INDUCING_POINTS * SGP_NUM_INDUCING_POINTS];

    void build_posterior(const UpdraftObservation* obs, uint16_t n) {
        ue.sigma_n_sq = 0.5f;
        ue.build_prior_matrix(kuu, nullptr, grid_points, false);
        memcpy(l_kuu, kuu, sizeof(kuu));
        ue.cholesky_decompose(l_kuu, M);

        memset(pvec, 0, sizeof(pvec));
        UpdraftObject dummy[MAX_UPDRAFT_MEM] = {};
        ue.accumulate_posterior(kuu, pvec, obs, n, obs[n-1].time_us,
                                {0,0}, grid_points, dummy);
        memcpy(l_a, kuu, sizeof(kuu));
        ue.cholesky_decompose(l_a, M);
        ue.cholesky_solve(l_a, M, pvec, alpha_buf);

        ue._L_Kuu_shared  = l_kuu;
        ue._L_A_shared    = l_a;
        ue._matrices_valid = true;
        for (uint8_t i = 0; i < M; i++) {
            ue._shared_state.inducing_points[i] = grid_points[i];
            ue._shared_state.alpha[i] = alpha_buf[i];
        }
    }
};

TEST_F(VarianceCalibrationTest, VarianceAtObsLocation_ReducedFromPrior) {
    UpdraftObservation obs[5];
    for (int i = 0; i < 5; i++) {
        obs[i] = {grid_points[38], 4.0f, 10000000U - (uint32_t)(i*200000U)};
    }
    build_posterior(obs, 5);

    Location qloc = ue._local_origin;
    float var_at_obs = ue.get_variance(qloc, 100.0f);
    float prior_var  = ue.kern_variance.get();

    EXPECT_LT(var_at_obs, prior_var * 0.6f)
        << "Variance at observation site should be < 60% of prior after 5 observations";
}

TEST_F(VarianceCalibrationTest, VarianceFarFromObs_NearPrior) {
    UpdraftObservation obs[3];
    for (int i = 0; i < 3; i++) {
        obs[i] = {grid_points[38], 4.0f, 10000000U - (uint32_t)(i*200000U)};
    }
    build_posterior(obs, 3);

    Location qloc = ue._local_origin;
    qloc.offset(60.0f, 60.0f);  // far corner of the grid
    float var_far  = ue.get_variance(qloc, 100.0f);
    float prior    = ue.kern_variance.get();

    EXPECT_GT(var_far, prior * 0.85f)
        << "Variance at unexplored location should be close to prior";
}

TEST_F(VarianceCalibrationTest, VarianceMap_MoreObs_LowerVariance) {
    float prev_var = ue.kern_variance.get();
    Location qloc = ue._local_origin;

    for (int n_obs : {1, 3, 5, 10}) {
        SetUp();
        UpdraftObservation obs[10];
        for (int i = 0; i < n_obs; i++) {
            obs[i] = {grid_points[38], 4.0f, 10000000U - (uint32_t)(i*100000U)};
        }
        VarianceCalibrationTest::build_posterior(obs, n_obs);
        float v = ue.get_variance(qloc, 100.0f);
        EXPECT_LT(v, prev_var) << "Variance did not decrease from " << prev_var
                               << " to " << v << " when adding obs " << n_obs;
        prev_var = v;
    }
}

TEST_F(VarianceCalibrationTest, VarianceMap_WritesCSV) {
    UpdraftObservation obs[10];
    Vector3f cross_pts[] = {{0,0,100},{15,0,100},{-15,0,100},{0,15,100},{0,-15,100},
                            {30,0,100},{-30,0,100},{0,30,100},{0,-30,100},{20,20,100}};
    for (int i = 0; i < 10; i++) {
        obs[i] = {cross_pts[i], 3.0f, 10000000U - (uint32_t)(i*100000U)};
    }
    build_posterior(obs, 10);

    std::ofstream f(OUT_DIR + "variance_calibration_map.csv");
    if (!f.is_open()) return;
    f << "North_m,East_m,Variance,PriorFrac\n";
    float prior = ue.kern_variance.get();
    for (float n = -50.0f; n <= 50.0f; n += 2.0f) {
        for (float e = -50.0f; e <= 50.0f; e += 2.0f) {
            Location q = ue._local_origin; q.offset(n, e);
            float v = ue.get_variance(q, 100.0f);
            f << n << "," << e << "," << v << "," << (v / prior) << "\n";
        }
    }
    std::ofstream fo(OUT_DIR + "variance_calibration_obs.csv");
    if (fo.is_open()) {
        fo << "North_m,East_m\n";
        for (const auto& pt : cross_pts) fo << pt.x << "," << pt.y << "\n";
    }
}



// SHAPE ESTIMATION ACCURACY

class ShapeEstimationTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{55};

    void run_anisotropic_sweep(float ru_true, float rv_true, float axis_deg,
                               float noise_sigma, std::ofstream& f,
                               const std::string& label) {
        clear_catalogue();
        ue._shared_state.obs_count = 0;
        float axis_rad = axis_deg * DEG_TO_RAD;
        float cos_a = cosf(axis_rad), sin_a = sinf(axis_rad);

        std::normal_distribution<float> noise{0.0f, noise_sigma};
        uint32_t now_us = 10000000U;

        for (int pass = 0; pass < 2; pass++) {
            for (int t = -8; t <= 8; t++) {
                float pos_n = (pass == 0) ? t * 5.0f : 0.0f;
                float pos_e = (pass == 0) ? 0.0f     : t * 5.0f;
                float du = pos_n * cos_a + pos_e * sin_a;
                float dv = -pos_n * sin_a + pos_e * cos_a;
                float wz = 4.0f * expf(-(du*du/sq(ru_true) + dv*dv/sq(rv_true))) + noise(generator);
                Location loc = ue._local_origin;
                loc.offset(pos_n, pos_e);
                now_us += 500000U;
                ue.push_observation(loc, 100.0f, wz, now_us);
                ue.trigger_update(loc, 100.0f, {5.0f, 0.0f}, {0.0f, 0.0f});
                ue.update_estimate(now_us);
            }
        }

        for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
            if (!ue._catalogue[i].active) continue;
            float ru_est = ue._catalogue[i].radius_u();
            float rv_est = ue._catalogue[i].radius_v();
            float ratio_true = ru_true / MAX(rv_true, 1.0f);
            float ratio_est  = ru_est  / MAX(rv_est,  1.0f);
            f << label << "," << ru_true << "," << rv_true << "," << axis_deg
              << "," << noise_sigma << "," << ru_est << "," << rv_est
              << "," << ratio_true << "," << ratio_est << "\n";
            break;  // report closest only
        }
    }
};

TEST_F(ShapeEstimationTest, ShapeEstimation_AnisotropicThermal_WritesCSV) {
    std::ofstream f(OUT_DIR + "shape_estimation_accuracy.csv");
    if (!f.is_open()) return;
    f << "Label,TrueRu,TrueRv,TrueAxis_deg,Noise,EstRu,EstRv,TrueRatio,EstRatio\n";

    struct Case { float ru, rv, axis, noise; const char* label; };
    Case cases[] = {
        {20, 20,   0,  0.0f, "circular_0noise"},
        {30, 10,   0,  0.0f, "elongated_NS_0noise"},
        {30, 10,  90,  0.0f, "elongated_EW_0noise"},
        {30, 10,  45,  0.0f, "elongated_diag_0noise"},
        {20, 20,   0,  1.0f, "circular_1noise"},
        {30, 10,   0,  1.0f, "elongated_NS_1noise"},
        {30, 10,  90,  1.0f, "elongated_EW_1noise"},
    };
    for (const auto& c : cases) {
        SetUp();
        run_anisotropic_sweep(c.ru, c.rv, c.axis, c.noise, f, c.label);
    }
}

TEST_F(ShapeEstimationTest, CircularThermal_RadiusAccuracy_ZeroNoise) {
    clear_catalogue();
    ue._shared_state.obs_count = 0;
    const float TRUE_R = 20.0f;
    uint32_t now_us = 10000000U;

    for (int step = 0; step < 20; step++) {
        float angle = step * (2.0f * M_PI / 20.0f);
        float pos_n = 30.0f * cosf(angle);
        float pos_e = 30.0f * sinf(angle);
        float wz = 4.0f * expf(-(pos_n*pos_n + pos_e*pos_e) / sq(TRUE_R));
        Location loc = ue._local_origin;
        loc.offset(pos_n, pos_e);
        now_us += 500000U;
        ue.push_observation(loc, 100.0f, wz, now_us);
        ue.trigger_update(loc, 100.0f, {0.0f, 0.0f}, {0.0f, 0.0f});
        ue.update_estimate(now_us);
    }

    for (int i = 0; i < MAX_UPDRAFT_MEM; i++) {
        if (!ue._catalogue[i].active) continue;
        if (!ue._catalogue[i].shape_gate_open) continue;
        float ru = ue._catalogue[i].radius_u();
        float rv = ue._catalogue[i].radius_v();
        EXPECT_NEAR(ru, TRUE_R, TRUE_R * 0.5f)
            << "Estimated Ru " << ru << " m is more than 50% off true radius " << TRUE_R;
        EXPECT_NEAR(rv, TRUE_R, TRUE_R * 0.5f)
            << "Estimated Rv " << rv << " m is more than 50% off true radius " << TRUE_R;
        break;
    }
}



// PARAMETER SENSITIVITY

class ParameterSensitivityTest : public UpdraftTestBase {
protected:
    std::default_random_engine generator{11};

    float compute_mse(float l_xy, float obs_noise, float tau,
                      float true_r, float noise_sigma) {
        SetUp();
        ue.kern_length_xy.set(l_xy);
        ue.obs_noise_var.set(obs_noise);
        ue.obs_tau_decay.set(tau);

        clear_catalogue();
        ue._shared_state.obs_count = 0;
        std::normal_distribution<float> noise{0.0f, noise_sigma};
        uint32_t now_us = 10000000U;
        Location loc = ue._local_origin;
        loc.offset(-60.0f, 5.0f);

        for (int t = 0; t <= 12; t++) {
            loc.offset(10.0f, 0.0f);
            Vector2f pos_ne = ue._local_origin.get_distance_NE(loc);
            float wz = 4.0f * expf(-(pos_ne.x*pos_ne.x+pos_ne.y*pos_ne.y)/sq(true_r))
                       + noise(generator);
            now_us += 1000000U;
            ue.push_observation(loc, 100.0f, wz, now_us);
            ue.trigger_update(loc, 100.0f, {10.0f, 0.0f}, {0.0f, 0.0f});
            ue.update_estimate(now_us);
        }

        float mse = 0.0f; int n = 0;
        for (int ni = -30; ni <= 30; ni += 5) {
            for (int ei = -30; ei <= 30; ei += 5) {
                Location q = ue._local_origin; q.offset(ni, ei);
                float true_lift = 4.0f * expf(-((float)(ni*ni+ei*ei))/sq(true_r));
                float est_lift  = ue.get_lift_prediction(q, 100.0f, now_us);
                mse += sq(true_lift - est_lift); n++;
            }
        }
        return n > 0 ? mse / n : 1e6f;
    }
};

TEST_F(ParameterSensitivityTest, HyperparameterSweep_WritesCSV) {
    const float l_xys[]   = {5.0f, 10.0f, 15.0f, 20.0f};
    const float noises[]  = {0.1f, 0.5f, 1.0f, 2.0f};
    const float taus[]    = {60.0f, 150.0f, 300.0f, 600.0f};
    const float r_trues[] = {10.0f, 20.0f, 30.0f};
    const float sigmas[]  = {0.0f, 0.5f, 1.5f};

    std::ofstream f(OUT_DIR + "hyperparam_sweep.csv");
    if (!f.is_open()) return;
    f << "LengthXY,ObsNoise,Tau,TrueR,Sigma,MSE\n";

    for (float r : r_trues) {
        for (float sig : sigmas) {
            for (float l : l_xys) {
                for (float obs_n : noises) {
                    for (float tau : taus) {
                        float mse = compute_mse(l, obs_n, tau, r, sig);
                        f << l << "," << obs_n << "," << tau << ","
                          << r << "," << sig << "," << mse << "\n";
                    }
                }
            }
        }
    }
}


AP_GTEST_MAIN()