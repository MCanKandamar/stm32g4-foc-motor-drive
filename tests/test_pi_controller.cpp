// PiController tests.
// Checked: P and I terms, output limits, anti-windup (conditional integration),
// noise robustness, reset and catching invalid settings.

#include "pi_controller.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-4f;

// Shared test settings: dt = 1 ms, output ±10 (e.g. volts)
PiConfig_ make_config(float kp, float ki)
{
    return {.Kp = kp, .Ki = ki, .step_time = 1e-3f, .out_max = 10.0f, .out_min = -10.0f};
}

}  // namespace

TEST(PiController, SadecePTerimi)
{
    PiController pi(make_config(2.0f, 0.0f));

    // error = 1.0 - 0.25 = 0.75  ->  output = 2 * 0.75
    EXPECT_NEAR(pi.update(1.0f, 0.25f), 1.5f, kTol);
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);
}

TEST(PiController, IntegralZamanlaBirikir)
{
    PiController pi(make_config(0.0f, 100.0f));

    // Each step: integral += Ki * error * dt = 100 * 1 * 0.001 = 0.1
    float output = 0.0f;
    for (int i = 0; i < 10; ++i) {
        output = pi.update(1.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 1.0f, kTol);
    EXPECT_NEAR(output, 1.0f, kTol);
}

TEST(PiController, CikisLimitleriAsilmaz)
{
    PiController pi(make_config(5.0f, 1000.0f));

    for (int i = 0; i < 1000; ++i) {
        const float output = pi.update(100.0f, 0.0f);
        EXPECT_LE(output, 10.0f);
    }
    EXPECT_FLOAT_EQ(pi.update(100.0f, 0.0f), 10.0f);

    for (int i = 0; i < 1000; ++i) {
        const float output = pi.update(-100.0f, 0.0f);
        EXPECT_GE(output, -10.0f);
    }
    EXPECT_FLOAT_EQ(pi.update(-100.0f, 0.0f), -10.0f);
}

TEST(PiController, PTerimiDoyurunca_IntegralBirikmez)
{
    // The P term alone saturates the output (1 * 100 > 10).
    // With conditional integration the integral must stay at 0 (no windup).
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 1000; ++i) {
        pi.update(100.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);

    // When the error turns negative (error = -1) the output must react at once:
    //   P = -1, integral in this step 0 + 100 * (-1) * 0.001 = -0.1  ->  output = -1.1
    // With windup the integral would be stuck near 10 and the output would be -1 + 9.9 = 8.9.
    EXPECT_NEAR(pi.update(0.0f, 1.0f), -1.1f, kTol);
}

TEST(PiController, GurultuOrnegiIntegraliBozmaz)
{
    // Scenario: the integral sits at ~8 under a steady load. One noisy sample
    // makes the P term large for a moment and saturates the output.
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 80; ++i) {
        pi.update(1.0f, 0.0f);  // +0.1 each step
    }
    const float integral_before = pi.getIntegral();
    ASSERT_NEAR(integral_before, 8.0f, 1e-3f);

    // Noise: error = 5  ->  P + I = 5 + 8 = 13 >= 10, error is positive  ->  integral is not added
    EXPECT_FLOAT_EQ(pi.update(5.0f, 0.0f), 10.0f);
    EXPECT_FLOAT_EQ(pi.getIntegral(), integral_before);

    // Noise is gone (error = 0): the output goes back to its old value at once
    EXPECT_NEAR(pi.update(0.0f, 0.0f), integral_before, kTol);
}

TEST(PiController, AltLimitteDeAntiWindupCalisir)
{
    PiController pi(make_config(1.0f, 100.0f));

    for (int i = 0; i < 1000; ++i) {
        pi.update(-100.0f, 0.0f);
    }
    EXPECT_NEAR(pi.getIntegral(), 0.0f, kTol);
    // When the error turns around: P = 1, integral +0.1 in this step  ->  output = 1.1
    EXPECT_NEAR(pi.update(0.0f, -1.0f), 1.1f, kTol);
}

TEST(PiController, ResetIntegraliSifirlar)
{
    PiController pi(make_config(0.0f, 100.0f));

    for (int i = 0; i < 20; ++i) {
        pi.update(1.0f, 0.0f);
    }
    ASSERT_GT(pi.getIntegral(), 1.0f);

    pi.reset();
    EXPECT_FLOAT_EQ(pi.getIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(pi.update(0.0f, 0.0f), 0.0f);
}

TEST(PiController, SetLimitsYeniLimitleriUygular)
{
    PiController pi(make_config(5.0f, 0.0f));
    pi.setLimits(-3.0f, 3.0f);

    EXPECT_FLOAT_EQ(pi.update(100.0f, 0.0f), 3.0f);
    EXPECT_FLOAT_EQ(pi.update(-100.0f, 0.0f), -3.0f);
}

TEST(PiController, LimitDaralincaIntegralDeDaralir)
{
    // Scenario: integral at 8 V, limits ±10 V. Vbus drops and the limit goes down to ±5 V
    // (current_control calls setLimits every cycle).
    PiController pi(make_config(1.0f, 100.0f));
    for (int i = 0; i < 80; ++i) {
        pi.update(1.0f, 0.0f);  // +0.1 each step  ->  integral ~8
    }
    ASSERT_NEAR(pi.getIntegral(), 8.0f, 1e-3f);

    pi.setLimits(-5.0f, 5.0f);
    EXPECT_FLOAT_EQ(pi.update(0.0f, 0.0f), 5.0f);

    // The integral must also drop to the new limit. If not, it stays at 8 and when the error turns around
    // the output is stuck at 5 V until the integral goes below 5 (30 steps) (windup).
    EXPECT_LE(pi.getIntegral(), 5.0f);

    // The error turned around (error = -1): P = -1, integral 5 - 0.1 = 4.9  ->  output is 3.9 at once
    EXPECT_NEAR(pi.update(0.0f, 1.0f), 3.9f, kTol);
}

// Death test: when built with invalid settings, the assert in the constructor must stop the program.
// (assert works only in Debug builds; in Release these tests are skipped.)
#ifndef NDEBUG
TEST(PiControllerDeathTest, AdimSuresiSifirsaDurur)
{
    PiConfig_ config = make_config(1.0f, 1.0f);
    config.step_time = 0.0f;
    EXPECT_DEATH(PiController{config}, "");
}

TEST(PiControllerDeathTest, LimitlerTersseDurur)
{
    PiConfig_ config = make_config(1.0f, 1.0f);
    config.out_max = -10.0f;
    config.out_min = 10.0f;
    EXPECT_DEATH(PiController{config}, "");
}
#endif
