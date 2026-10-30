// OpenLoopAngle tests: the angle ramp used by the open-loop start and the Hall calibration.

#include "math_utils.hpp"
#include "startup.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-4f;
constexpr float kDt = 1e-3f;

}  // namespace

TEST(OpenLoopAngle, IntegratesTheSpeed)
{
    OpenLoopAngle angle;

    // 100 rad/s * 1 ms = 0.1 rad each step
    float theta = 0.0f;
    for (int i = 0; i < 10; ++i) {
        theta = angle.update(100.0f, kDt);
    }
    EXPECT_NEAR(theta, 1.0f, kTol);
}

TEST(OpenLoopAngle, WrapsAboveTwoPi)
{
    OpenLoopAngle angle;
    angle.reset(TWO_PI - 0.05f);

    EXPECT_NEAR(angle.update(100.0f, kDt), 0.05f, kTol);
}

TEST(OpenLoopAngle, WrapsBelowZero)
{
    OpenLoopAngle angle;
    angle.reset(0.05f);

    EXPECT_NEAR(angle.update(-100.0f, kDt), TWO_PI - 0.05f, kTol);
}

TEST(OpenLoopAngle, TravelledCountsBothDirections)
{
    OpenLoopAngle angle;

    for (int i = 0; i < 10; ++i) {
        angle.update(100.0f, kDt);
    }
    for (int i = 0; i < 10; ++i) {
        angle.update(-100.0f, kDt);
    }
    EXPECT_NEAR(angle.get_travelled(), 2.0f, kTol);
}

TEST(OpenLoopAngle, ResetSetsAngleAndClearsTravelled)
{
    OpenLoopAngle angle;
    angle.update(100.0f, kDt);

    angle.reset(1.0f);
    EXPECT_FLOAT_EQ(angle.get_travelled(), 0.0f);
    EXPECT_NEAR(angle.update(0.0f, kDt), 1.0f, kTol);
}

TEST(OpenLoopAngle, ClearTravelledKeepsTheAngle)
{
    OpenLoopAngle angle;
    angle.reset(1.0f);
    angle.update(100.0f, kDt);

    angle.clear_travelled();
    EXPECT_FLOAT_EQ(angle.get_travelled(), 0.0f);
    EXPECT_NEAR(angle.update(0.0f, kDt), 1.1f, kTol);
}
