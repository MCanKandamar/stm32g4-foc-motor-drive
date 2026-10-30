// Clarke, inverse Clarke, Park and inverse Park tests.

#include "transforms.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace motor_control;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTol = 1e-5f;

float deg_to_rad(int deg)
{
    return static_cast<float>(deg) * kPi / 180.0f;
}

SinCos_t make_sin_cos(float theta)
{
    return {std::sin(theta), std::cos(theta)};
}

// Balanced 3-phase current with peak 'amplitude' and electrical angle 'theta'
Iabc_t balanced_current(float amplitude, float theta)
{
    return {
        amplitude * std::cos(theta),
        amplitude * std::cos(theta - 2.0f * kPi / 3.0f),
        amplitude * std::cos(theta + 2.0f * kPi / 3.0f),
    };
}

}  // namespace

// ---------------------------------------------------------------- Clarke

TEST(Clarke, PhaseAPeakIsOnAlphaAxis)
{
    const I_AlphaBeta_t ab = clarke({1.0f, -0.5f, -0.5f});
    EXPECT_NEAR(ab.ialpha, 1.0f, kTol);
    EXPECT_NEAR(ab.ibeta, 0.0f, kTol);
}

TEST(Clarke, NinetyDegreesIsOnBetaAxis)
{
    const I_AlphaBeta_t ab = clarke(balanced_current(1.0f, kPi / 2.0f));
    EXPECT_NEAR(ab.ialpha, 0.0f, kTol);
    EXPECT_NEAR(ab.ibeta, 1.0f, kTol);
}

TEST(Clarke, KeepsAmplitude)
{
    for (int deg = 0; deg < 360; deg += 15) {
        const I_AlphaBeta_t ab = clarke(balanced_current(2.5f, deg_to_rad(deg)));
        EXPECT_NEAR(std::hypot(ab.ialpha, ab.ibeta), 2.5f, kTol) << "aci = " << deg;
    }
}

// ---------------------------------------------------------------- Inverse Clarke

TEST(InverseClarke, VectorOnAlphaAxis)
{
    const Vabc_t v = inverseClarke({10.0f, 0.0f});
    EXPECT_NEAR(v.Va, 10.0f, kTol);
    EXPECT_NEAR(v.Vb, -5.0f, kTol);
    EXPECT_NEAR(v.Vc, -5.0f, kTol);
}

TEST(InverseClarke, PhasesSumToZero)
{
    for (int deg = 0; deg < 360; deg += 15) {
        const float theta = deg_to_rad(deg);
        const Vabc_t v = inverseClarke({7.0f * std::cos(theta), 7.0f * std::sin(theta)});
        EXPECT_NEAR(v.Va + v.Vb + v.Vc, 0.0f, 1e-4f) << "aci = " << deg;
    }
}

TEST(InverseClarke, RoundTripWithClarke)
{
    for (int deg = 0; deg < 360; deg += 15) {
        const Iabc_t i = balanced_current(3.0f, deg_to_rad(deg));
        const I_AlphaBeta_t ab = clarke(i);
        const Vabc_t back = inverseClarke({ab.ialpha, ab.ibeta});  // copy by hand from the current type to the voltage type
        EXPECT_NEAR(back.Va, i.ia, kTol) << "aci = " << deg;
        EXPECT_NEAR(back.Vb, i.ib, kTol) << "aci = " << deg;
        EXPECT_NEAR(back.Vc, i.ic, kTol) << "aci = " << deg;
    }
}

// ---------------------------------------------------------------- Park

TEST(Park, CurrentRotatingWithRotorIsDcOnDAxis)
{
    // The basic idea of FOC: a balanced 3-phase current, passed through Park with the rotor angle,
    // turns into constant (DC) d and q values.
    for (int deg = 0; deg < 360; deg += 15) {
        const float theta = deg_to_rad(deg);
        const Idq_fdb_t dq = park(clarke(balanced_current(3.0f, theta)), make_sin_cos(theta));
        EXPECT_NEAR(dq.id_fdb, 3.0f, kTol) << "aci = " << deg;
        EXPECT_NEAR(dq.iq_fdb, 0.0f, kTol) << "aci = " << deg;
    }
}

TEST(Park, CurrentNinetyDegreesAheadIsOnQAxis)
{
    // Current 90° ahead of the rotor -> all of it on the q axis (torque current)
    for (int deg = 0; deg < 360; deg += 15) {
        const float theta = deg_to_rad(deg);
        const Idq_fdb_t dq = park(clarke(balanced_current(2.0f, theta + kPi / 2.0f)), make_sin_cos(theta));
        EXPECT_NEAR(dq.id_fdb, 0.0f, kTol) << "aci = " << deg;
        EXPECT_NEAR(dq.iq_fdb, 2.0f, kTol) << "aci = " << deg;
    }
}

// ---------------------------------------------------------------- Inverse Park

TEST(InversePark, AtZeroAngleDAxisIsAlphaAxis)
{
    const V_AlphaBeta_t ab = inversePark({4.0f, 1.5f}, make_sin_cos(0.0f));
    EXPECT_NEAR(ab.Valpha, 4.0f, kTol);
    EXPECT_NEAR(ab.Vbeta, 1.5f, kTol);
}

TEST(InversePark, RoundTripWithPark)
{
    const Vdq_t input{1.2f, -0.7f};
    for (int deg = 0; deg < 360; deg += 15) {
        const SinCos_t sc = make_sin_cos(deg_to_rad(deg));
        const V_AlphaBeta_t ab = inversePark(input, sc);
        const Idq_fdb_t back = park({ab.Valpha, ab.Vbeta}, sc);  // copy by hand from the voltage type to the current type
        EXPECT_NEAR(back.id_fdb, input.Vd, kTol) << "aci = " << deg;
        EXPECT_NEAR(back.iq_fdb, input.Vq, kTol) << "aci = " << deg;
    }
}
