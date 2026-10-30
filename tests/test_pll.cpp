// AnglePLL tests: lock at constant speed, 2π wrap, reverse direction, and a start with reset(θ, ω)
// without a settling transient.
//
// The measured angle is the real rotor angle at each step (no noise); the PLL angle is compared with it.

#include "pll.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace motor_control;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kTs = 50e-6f;                 // 20 kHz ISR
constexpr float kWn = kTwoPi * 30.0f;         // bandwidth 30 Hz
constexpr float kZeta = 0.8f;

PiConfig_ pll_config()
{
    return {.Kp = 2.0f * kZeta * kWn, .Ki = kWn * kWn, .step_time = kTs, .out_max = 5000.0f, .out_min = -5000.0f};
}

float wrap_0_2pi(float a)
{
    a = std::fmod(a, kTwoPi);
    return (a < 0.0f) ? a + kTwoPi : a;
}

// Shortest difference between two angles, in degrees (−180..+180)
float diff_deg(float a, float b)
{
    float d = std::fmod(a - b + 3.0f * kPi, kTwoPi) - kPi;
    return d * 180.0f / kPi;
}

// Rotor turns at 'omega'; feeds the PLL for 'steps' steps, returns the largest angle error
float run_max_error_deg(AnglePLL& pll, float theta0, float omega, int steps)
{
    float worst = 0.0f;
    for (int k = 1; k <= steps; ++k) {
        const float theta = wrap_0_2pi(theta0 + omega * kTs * static_cast<float>(k));
        pll.update(theta);
        worst = std::fmax(worst, std::fabs(diff_deg(pll.get_angle(), theta)));
    }
    return worst;
}

}  // namespace

TEST(AnglePLL, ResetVerilenAciylaBaslar)
{
    AnglePLL pll(pll_config());
    pll.reset(1.0f, 0.0f);
    EXPECT_FLOAT_EQ(pll.get_angle(), 1.0f);
}

TEST(AnglePLL, ResetSonrasiHizVerilenDegerdir)
{
    AnglePLL pll(pll_config());
    pll.reset(0.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pll.get_speed(), 1000.0f);
}

TEST(AnglePLL, SabitHizdaKilitlenir)
{
    // Start from rest, rotor turns at 1000 rad/s. After 0.2 s the angle and speed must have settled.
    AnglePLL pll(pll_config());
    pll.reset(0.0f, 0.0f);
    run_max_error_deg(pll, 0.0f, 1000.0f, 4000);

    const float theta = wrap_0_2pi(1000.0f * kTs * 4000.0f);
    EXPECT_NEAR(diff_deg(pll.get_angle(), theta), 0.0f, 0.5f);
    EXPECT_NEAR(pll.get_speed(), 1000.0f, 10.0f);
}

TEST(AnglePLL, TersYondeDeKilitlenir)
{
    AnglePLL pll(pll_config());
    pll.reset(0.0f, 0.0f);
    run_max_error_deg(pll, 0.0f, -1000.0f, 4000);

    const float theta = wrap_0_2pi(-1000.0f * kTs * 4000.0f);
    EXPECT_NEAR(diff_deg(pll.get_angle(), theta), 0.0f, 0.5f);
    EXPECT_NEAR(pll.get_speed(), -1000.0f, 10.0f);
}

TEST(AnglePLL, IkiPiGecisindeAciZiplamaz)
{
    // Turn a few times while locked: the angle is always in 0..2π and moves only ω·Ts per step.
    AnglePLL pll(pll_config());
    pll.reset(0.0f, 1000.0f);

    float prev = pll.get_angle();
    for (int k = 1; k <= 2000; ++k) {                       // ~16 electrical turns
        const float angle = pll.update(wrap_0_2pi(1000.0f * kTs * static_cast<float>(k)));
        ASSERT_GE(angle, 0.0f) << "adim " << k;
        ASSERT_LT(angle, kTwoPi) << "adim " << k;
        EXPECT_NEAR(diff_deg(angle, prev), 1000.0f * kTs * 180.0f / kPi, 0.5f) << "adim " << k;
        prev = angle;
    }
}

TEST(AnglePLL, DogruResetGecisSureciYaratmaz)
{
    // Start while the motor already turns at 1000 rad/s: if the speed is given, the error stays small from the start.
    AnglePLL pll(pll_config());
    pll.reset(2.0f, 1000.0f);
    EXPECT_LT(run_max_error_deg(pll, 2.0f, 1000.0f, 400), 1.0f);   // first 20 ms
}

TEST(AnglePLL, HizsizResetBuyukGecisSureciYaratir)
{
    // Same case with speed 0: the angle error goes above 90° until the PLL catches up (this is why we pass ω).
    AnglePLL pll(pll_config());
    pll.reset(2.0f, 0.0f);
    EXPECT_GT(run_max_error_deg(pll, 2.0f, 1000.0f, 400), 90.0f);
}
