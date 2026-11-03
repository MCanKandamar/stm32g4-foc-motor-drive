// regen_limit tests: braking current share from the DC bus voltage (27 V → 29 V band).

#include "regen_limit.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kStart = 27.0f;
constexpr float kStop  = 29.0f;

}  // namespace

TEST(RegenScale, FullBelowStart)
{
    EXPECT_FLOAT_EQ(regen_scale(24.0f, kStart, kStop), 1.0f);
    EXPECT_FLOAT_EQ(regen_scale(27.0f, kStart, kStop), 1.0f);
}

TEST(RegenScale, LinearInTheBand)
{
    EXPECT_FLOAT_EQ(regen_scale(28.0f, kStart, kStop), 0.5f);
    EXPECT_FLOAT_EQ(regen_scale(28.5f, kStart, kStop), 0.25f);
}

TEST(RegenScale, ZeroAboveStop)
{
    EXPECT_FLOAT_EQ(regen_scale(29.0f, kStart, kStop), 0.0f);
    EXPECT_FLOAT_EQ(regen_scale(35.0f, kStart, kStop), 0.0f);
}

TEST(LimitBrakingIq, ScalesBrakingInBothDirections)
{
    EXPECT_FLOAT_EQ(limit_braking_iq(-4.0f,  1000.0f, 0.5f), -2.0f);   // forward, braking
    EXPECT_FLOAT_EQ(limit_braking_iq( 4.0f, -1000.0f, 0.5f),  2.0f);   // reverse, braking
}

TEST(LimitBrakingIq, DrivingPassesUnchanged)
{
    EXPECT_FLOAT_EQ(limit_braking_iq( 4.0f,  1000.0f, 0.0f),  4.0f);
    EXPECT_FLOAT_EQ(limit_braking_iq(-4.0f, -1000.0f, 0.0f), -4.0f);
}

TEST(LimitBrakingIq, ZeroSpeedPassesUnchanged)
{
    // align and open loop run at speed feedback 0: nothing to limit
    EXPECT_FLOAT_EQ(limit_braking_iq(-2.0f, 0.0f, 0.0f), -2.0f);
}
