// math_utils tests: angle wrapping and the standard sin/cos helper.

#include "math_utils.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-5f;

}  // namespace

TEST(WrapTwoPi, KeepsAngleInsideTheRange)
{
    EXPECT_FLOAT_EQ(wrap_2pi(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(wrap_2pi(0.0f), 0.0f);
}

TEST(WrapTwoPi, WrapsAngleAboveTheRange)
{
    EXPECT_NEAR(wrap_2pi(7.0f), 7.0f - TWO_PI, kTol);
    EXPECT_NEAR(wrap_2pi(TWO_PI), 0.0f, kTol);
}

TEST(WrapTwoPi, WrapsNegativeAngle)
{
    EXPECT_NEAR(wrap_2pi(-1.0f), TWO_PI - 1.0f, kTol);
}

TEST(WrapPi, KeepsAngleInsideTheRange)
{
    EXPECT_FLOAT_EQ(wrap_pi(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(wrap_pi(-1.0f), -1.0f);
    EXPECT_FLOAT_EQ(wrap_pi(-PI), -PI);
}

TEST(WrapPi, WrapsAngleOutsideTheRange)
{
    EXPECT_NEAR(wrap_pi(4.0f), 4.0f - TWO_PI, kTol);
    EXPECT_NEAR(wrap_pi(-4.0f), TWO_PI - 4.0f, kTol);
    EXPECT_NEAR(wrap_pi(PI), -PI, kTol);    // the range is [-π, π)
}

TEST(SinCosStd, GivesSineAndCosine)
{
    // 30°: sin = 0.5, cos = 0.8660254
    const SinCos_t sc = sincos_std(PI / 6.0f);
    EXPECT_NEAR(sc.sin, 0.5f, kTol);
    EXPECT_NEAR(sc.cos, 0.8660254f, kTol);
}
