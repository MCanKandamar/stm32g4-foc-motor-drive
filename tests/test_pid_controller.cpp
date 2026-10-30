// PidController tests. The speed loop uses this class through speed_control.
// Checked: P, I and D terms, output limits, anti-windup, reset and catching invalid settings.

#include "pi_controller.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-4f;

// Shared test settings: dt = 1 ms, output ±10
PidConfig_ make_config(float kp, float ki, float kd)
{
    return {.Kp = kp, .Ki = ki, .Kd = kd, .step_time = 1e-3f, .out_max = 10.0f, .out_min = -10.0f};
}

}  // namespace

TEST(PidController, ProportionalTermOnly)
{
    PidController pid(make_config(2.0f, 0.0f, 0.0f));

    // error = 1.0 - 0.25 = 0.75  ->  output = 2 * 0.75
    EXPECT_NEAR(pid.update(1.0f, 0.25f), 1.5f, kTol);
    EXPECT_NEAR(pid.getIntegral(), 0.0f, kTol);
}

TEST(PidController, IntegralAccumulatesOverTime)
{
    PidController pid(make_config(0.0f, 100.0f, 0.0f));

    // Each step: integral += Ki * error * dt = 100 * 1 * 0.001 = 0.1
    float output = 0.0f;
    for (int i = 0; i < 10; ++i) {
        output = pid.update(1.0f, 0.0f);
    }
    EXPECT_NEAR(pid.getIntegral(), 1.0f, kTol);
    EXPECT_NEAR(output, 1.0f, kTol);
}

TEST(PidController, DerivativeReactsToErrorChange)
{
    PidController pid(make_config(0.0f, 0.0f, 0.002f));

    // The error steps from 0 to 1 in 1 ms: derivative = 1000 /s, D term = 0.002 * 1000 = 2
    EXPECT_NEAR(pid.update(1.0f, 0.0f), 2.0f, kTol);
    EXPECT_NEAR(pid.getDerivative(), 1000.0f, 0.1f);

    // Same error again: the derivative is 0
    EXPECT_NEAR(pid.update(1.0f, 0.0f), 0.0f, kTol);
    EXPECT_NEAR(pid.getDerivative(), 0.0f, kTol);
}

TEST(PidController, OutputStaysWithinLimits)
{
    PidController pid(make_config(5.0f, 1000.0f, 0.0f));

    for (int i = 0; i < 1000; ++i) {
        EXPECT_LE(pid.update(100.0f, 0.0f), 10.0f);
    }
    EXPECT_FLOAT_EQ(pid.update(100.0f, 0.0f), 10.0f);

    for (int i = 0; i < 1000; ++i) {
        EXPECT_GE(pid.update(-100.0f, 0.0f), -10.0f);
    }
    EXPECT_FLOAT_EQ(pid.update(-100.0f, 0.0f), -10.0f);
}

TEST(PidController, NoIntegralWindupWhenPTermSaturates)
{
    // The P term alone saturates the output (1 * 100 > 10), so the integral must stay at 0.
    PidController pid(make_config(1.0f, 100.0f, 0.0f));

    for (int i = 0; i < 1000; ++i) {
        pid.update(100.0f, 0.0f);
    }
    EXPECT_NEAR(pid.getIntegral(), 0.0f, kTol);

    // error = -1: P = -1, integral in this step -0.1  ->  output = -1.1
    EXPECT_NEAR(pid.update(0.0f, 1.0f), -1.1f, kTol);
}

TEST(PidController, ResetClearsAllState)
{
    PidController pid(make_config(0.0f, 100.0f, 0.002f));

    for (int i = 0; i < 20; ++i) {
        pid.update(1.0f, 0.0f);
    }
    ASSERT_GT(pid.getIntegral(), 1.0f);

    pid.reset();
    EXPECT_FLOAT_EQ(pid.getIntegral(), 0.0f);
    EXPECT_FLOAT_EQ(pid.getDerivative(), 0.0f);

    // previous_error is 0 again, so the same error counts as a new step:
    // D = 0.002 * 1000 = 2, I = 0.1  ->  output = 2.1
    EXPECT_NEAR(pid.update(1.0f, 0.0f), 2.1f, kTol);
}

TEST(PidController, SetLimitsAppliesNewLimits)
{
    PidController pid(make_config(5.0f, 0.0f, 0.0f));
    pid.setLimits(-3.0f, 3.0f);

    EXPECT_FLOAT_EQ(pid.update(100.0f, 0.0f), 3.0f);
    EXPECT_FLOAT_EQ(pid.update(-100.0f, 0.0f), -3.0f);
}

TEST(PidController, SetIntegralSetsTheStartValue)
{
    PidController pid(make_config(0.0f, 0.0f, 0.0f));
    pid.setIntegral(2.5f);

    EXPECT_FLOAT_EQ(pid.getIntegral(), 2.5f);
    EXPECT_FLOAT_EQ(pid.update(0.0f, 0.0f), 2.5f);
}

// Death test: when built with invalid settings, the assert in the constructor must stop the program.
// (assert works only in Debug builds; in Release these tests are skipped.)
#ifndef NDEBUG
TEST(PidControllerDeathTest, ZeroStepTimeAborts)
{
    PidConfig_ config = make_config(1.0f, 1.0f, 0.0f);
    config.step_time = 0.0f;
    EXPECT_DEATH(PidController{config}, "");
}

TEST(PidControllerDeathTest, ReversedLimitsAbort)
{
    PidConfig_ config = make_config(1.0f, 1.0f, 0.0f);
    config.out_max = -10.0f;
    config.out_min = 10.0f;
    EXPECT_DEATH(PidController{config}, "");
}
#endif
