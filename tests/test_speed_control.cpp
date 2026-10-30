// speed_control tests: iq from the speed error, id stays 0, limits, and reset with a start value.

#include "speed_control.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-4f;

// Gains from the board: Kp [A/rpm], Ki [A/(rpm·s)], 1 kHz, ±9 A
PidConfig_ speed_config()
{
    return {.Kp = 0.0025f, .Ki = 0.02f, .Kd = 0.0f, .step_time = 0.001f, .out_max = 9.0f, .out_min = -9.0f};
}

}  // namespace

TEST(SpeedControl, IqFollowsSpeedError)
{
    speed_control speed(speed_config());

    // error = 200 rpm: P = 0.0025 * 200 = 0.5 A, I = 0.02 * 200 * 0.001 = 0.004 A
    const Idq_ref_t idq = speed.update(1000.0f, 800.0f);
    EXPECT_NEAR(idq.iq_ref, 0.504f, kTol);
}

TEST(SpeedControl, IdStaysZero)
{
    speed_control speed(speed_config());

    EXPECT_FLOAT_EQ(speed.update(1000.0f, 0.0f).id_ref, 0.0f);
    EXPECT_FLOAT_EQ(speed.update(-1000.0f, 0.0f).id_ref, 0.0f);
}

TEST(SpeedControl, IqStaysWithinLimits)
{
    speed_control speed(speed_config());

    EXPECT_FLOAT_EQ(speed.update(100000.0f, 0.0f).iq_ref, 9.0f);
    EXPECT_FLOAT_EQ(speed.update(-100000.0f, 0.0f).iq_ref, -9.0f);
}

TEST(SpeedControl, GetOutputKeepsTheLastValue)
{
    speed_control speed(speed_config());

    const Idq_ref_t last = speed.update(1000.0f, 800.0f);
    EXPECT_FLOAT_EQ(speed.get_output().iq_ref, last.iq_ref);
    EXPECT_FLOAT_EQ(speed.get_output().id_ref, last.id_ref);
}

TEST(SpeedControl, ResetStartsFromTheGivenIq)
{
    speed_control speed(speed_config());
    speed.update(1000.0f, 0.0f);

    speed.reset(1.5f);
    EXPECT_FLOAT_EQ(speed.get_output().iq_ref, 1.5f);
    EXPECT_FLOAT_EQ(speed.get_output().id_ref, 0.0f);

    // No speed error: the integral holds the start value, so there is no jump
    EXPECT_NEAR(speed.update(500.0f, 500.0f).iq_ref, 1.5f, kTol);
}

TEST(SpeedControl, DefaultResetGoesToZero)
{
    speed_control speed(speed_config());
    speed.update(1000.0f, 0.0f);

    speed.reset();
    EXPECT_FLOAT_EQ(speed.get_output().iq_ref, 0.0f);
    EXPECT_NEAR(speed.update(500.0f, 500.0f).iq_ref, 0.0f, kTol);
}
