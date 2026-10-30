// Sliding-mode observer and back-EMF PLL tests, with the observer settings from the board.
//
// The plant is the same discrete R-L model the observer uses, with a known back-EMF e:
//   i[k+1] = F * i[k] + G * (v - e),  F = 1 - R/L * Ts,  G = Ts / L
// Inside the boundary layer the correction is z = (k/phi) * error, so at steady state
//   z / e = G*K / (1 - F + G*K),  K = k/phi

#include "observer.hpp"
#include "pll.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace motor_control;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kTs = 50e-6f;           // 20 kHz ISR
constexpr float kR = 0.20f;             // [Ω]
constexpr float kL = 0.30e-3f;          // [H]
constexpr float kGain = 20.0f;
constexpr float kPhi = 3.3f;

constexpr float kF = 1.0f - kR / kL * kTs;
constexpr float kG = kTs / kL;
constexpr float kK = kGain / kPhi;
constexpr float kSteadyGain = kG * kK / (1.0f - kF + kG * kK);

constexpr float kOmega = 1000.0f;       // [rad/s electrical], 2387 rpm with 4 pole pairs
constexpr float kEmfPeak = 6.0f;        // [V], λ = 6 mWb at 1000 rad/s

smo_config smo_settings()
{
    return {.phase_resistance_ohm = kR, .phase_inductance = kL, .sample_time = kTs, .k_gain = kGain, .phi = kPhi};
}

PiConfig_ pll_settings()     // 30 Hz, as on the board
{
    const float wn = kTwoPi * 30.0f;
    return {.Kp = 2.0f * 0.8f * wn, .Ki = wn * wn, .step_time = kTs, .out_max = 2000.0f, .out_min = -2000.0f};
}

Z_alphabeta_t back_emf(float theta, float peak)
{
    return {-peak * std::sin(theta), peak * std::cos(theta)};
}

// Shortest difference between two angles, in degrees (−180..+180)
float diff_deg(float a, float b)
{
    float d = std::fmod(a - b + 3.0f * kPi, kTwoPi);
    if (d < 0.0f) d += kTwoPi;
    return (d - kPi) * 180.0f / kPi;
}

float wrap(float a)
{
    return std::fmod(a, kTwoPi) + ((a < 0.0f) ? kTwoPi : 0.0f);
}

// One step of the plant
void plant_step(I_AlphaBeta_t& i, const V_AlphaBeta_t& v, const Z_alphabeta_t& e)
{
    i.ialpha = kF * i.ialpha + kG * (v.Valpha - e.z_alpha);
    i.ibeta  = kF * i.ibeta  + kG * (v.Vbeta  - e.z_beta);
}

}  // namespace

TEST(SmoObserver, ZeroInputsGiveZeroCorrection)
{
    smo_observer smo(smo_settings());

    const Z_alphabeta_t z = smo.update({0.0f, 0.0f}, {0.0f, 0.0f});
    EXPECT_FLOAT_EQ(z.z_alpha, 0.0f);
    EXPECT_FLOAT_EQ(z.z_beta, 0.0f);
}

TEST(SmoObserver, ReportsTheCurrentError)
{
    smo_observer smo(smo_settings());

    smo.update({2.0f, -1.0f}, {0.0f, 0.0f});
    EXPECT_FLOAT_EQ(smo.get_ialphabeta_error().i_alpha_error, -2.0f);
    EXPECT_FLOAT_EQ(smo.get_ialphabeta_error().i_beta_error, 1.0f);
}

TEST(SmoObserver, CorrectionIsLinearInsideTheBoundaryLayer)
{
    smo_observer smo(smo_settings());

    // error = -1 A, |error| < phi  ->  z = (20 / 3.3) * -1
    const Z_alphabeta_t z = smo.update({1.0f, 0.0f}, {0.0f, 0.0f});
    EXPECT_NEAR(z.z_alpha, -kK, 1e-4f);
}

TEST(SmoObserver, CorrectionSaturatesAtTheGain)
{
    smo_observer smo(smo_settings());

    const Z_alphabeta_t z = smo.update({100.0f, -100.0f}, {0.0f, 0.0f});
    EXPECT_FLOAT_EQ(z.z_alpha, -kGain);
    EXPECT_FLOAT_EQ(z.z_beta, kGain);
}

TEST(SmoObserver, ConvergesToConstantBackEmf)
{
    smo_observer smo(smo_settings());
    const Z_alphabeta_t e{5.0f, -3.0f};
    const V_AlphaBeta_t v{0.0f, 0.0f};

    I_AlphaBeta_t i{0.0f, 0.0f};
    Z_alphabeta_t z{};
    for (int k = 0; k < 2000; ++k) {
        z = smo.update(i, v);
        plant_step(i, v, e);
    }
    EXPECT_NEAR(z.z_alpha, e.z_alpha * kSteadyGain, 0.01f);
    EXPECT_NEAR(z.z_beta, e.z_beta * kSteadyGain, 0.01f);
}

TEST(SmoObserver, FollowsRotatingBackEmf)
{
    smo_observer smo(smo_settings());
    const V_AlphaBeta_t v{0.0f, 0.0f};

    I_AlphaBeta_t i{0.0f, 0.0f};
    Z_alphabeta_t z{};
    float theta = 0.0f;
    for (int k = 0; k < 4000; ++k) {
        const Z_alphabeta_t e = back_emf(theta, kEmfPeak);
        z = smo.update(i, v);
        plant_step(i, v, e);
        theta = wrap(theta + kOmega * kTs);
    }

    EXPECT_NEAR(std::hypot(z.z_alpha, z.z_beta), kEmfPeak * kSteadyGain, 0.1f);
}

TEST(EmfPll, LocksOnRotatingEmf)
{
    AnglePLL pll(pll_settings());
    pll.reset(0.0f, 0.0f);

    float theta = 0.0f;
    for (int k = 0; k < 8000; ++k) {
        pll.update_emf(back_emf(theta, kEmfPeak));
        theta = wrap(theta + kOmega * kTs);
    }

    // update_emf() moves its angle first, so the PLL sits one step behind 'theta' here
    EXPECT_NEAR(diff_deg(pll.get_angle(), theta), -kOmega * kTs * 180.0f / kPi, 1.0f);
    EXPECT_NEAR(pll.get_speed(), kOmega, 10.0f);
}

TEST(EmfPll, LocksInReverse)
{
    AnglePLL pll(pll_settings());
    pll.reset(0.0f, 0.0f);

    float theta = 0.0f;
    for (int k = 0; k < 8000; ++k) {
        pll.update_emf(back_emf(theta, kEmfPeak));
        theta = wrap(theta - kOmega * kTs);
    }

    EXPECT_NEAR(diff_deg(pll.get_angle(), theta), kOmega * kTs * 180.0f / kPi, 1.0f);
    EXPECT_NEAR(pll.get_speed(), -kOmega, 10.0f);
}

TEST(EmfPll, LockDoesNotDependOnEmfAmplitude)
{
    for (float peak : {0.5f, 20.0f}) {
        AnglePLL pll(pll_settings());
        pll.reset(0.0f, 0.0f);

        float theta = 0.0f;
        for (int k = 0; k < 8000; ++k) {
            pll.update_emf(back_emf(theta, peak));
            theta = wrap(theta + kOmega * kTs);
        }
        EXPECT_NEAR(pll.get_speed(), kOmega, 10.0f) << "peak = " << peak;
    }
}

TEST(EmfPll, ZeroEmfKeepsAngleAndSpeed)
{
    AnglePLL pll(pll_settings());
    pll.reset(1.0f, 0.0f);

    for (int k = 0; k < 100; ++k) {
        pll.update_emf({0.0f, 0.0f});
    }
    EXPECT_FLOAT_EQ(pll.get_angle(), 1.0f);
    EXPECT_FLOAT_EQ(pll.get_speed(), 0.0f);
}

TEST(SmoWithPll, FollowsTheRotorAngle)
{
    smo_observer smo(smo_settings());
    AnglePLL pll(pll_settings());
    pll.reset(0.0f, 0.0f);

    const V_AlphaBeta_t v{0.0f, 0.0f};
    I_AlphaBeta_t i{0.0f, 0.0f};
    float theta = 0.0f;
    for (int k = 0; k < 8000; ++k) {
        const Z_alphabeta_t e = back_emf(theta, kEmfPeak);
        pll.update_emf(smo.update(i, v));
        plant_step(i, v, e);
        theta = wrap(theta + kOmega * kTs);
    }

    // Measured lag: 5.6°, from the observer's delay at this speed
    EXPECT_NEAR(diff_deg(pll.get_angle(), theta), 0.0f, 8.0f);
    EXPECT_NEAR(pll.get_speed(), kOmega, 10.0f);
}
