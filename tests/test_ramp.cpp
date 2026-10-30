// Ramp tests.
// Checked: acceleration/deceleration slope, exact stop at the target, at a zero crossing
// first rate_down then rate_up, negative direction and reset.

#include "ramp.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-3f;
constexpr float kDt  = 1e-3f;      // 1 ms

// run n steps, return the last output
float run(Ramp& r, float target, float up, float down, int n)
{
    float out = r.get_output();
    for (int i = 0; i < n; ++i) {
        out = r.update(target, up, down, kDt);
    }
    return out;
}

}  // namespace

TEST(Ramp, AcceleratesWithRateUp)
{
    Ramp r;

    // 1000 rpm/s, 100 ms -> 100 rpm
    EXPECT_NEAR(run(r, 3000.0f, 1000.0f, 500.0f, 100), 100.0f, kTol);
}

TEST(Ramp, StopsExactlyAtTarget)
{
    Ramp r;

    // at 1000 rpm/s, 300 rpm is reached in 300 ms; extra steps do not go past the target
    run(r, 300.0f, 1000.0f, 1000.0f, 400);
    EXPECT_EQ(r.get_output(), 300.0f);
}

TEST(Ramp, DeceleratesWithRateDown)
{
    Ramp r;
    r.reset(1000.0f);

    // |output| shrinks: 500 rpm/s, 100 ms -> drops 50 rpm
    EXPECT_NEAR(run(r, 0.0f, 2000.0f, 500.0f, 100), 950.0f, kTol);
}

TEST(Ramp, CrossesZero)
{
    Ramp r;
    r.reset(100.0f);

    // +100 -> -100: first from 100 rpm to 0 at 1000 rpm/s in 100 ms,
    // then to -100 at 2000 rpm/s in 50 ms
    EXPECT_NEAR(run(r, -100.0f, 2000.0f, 1000.0f, 100), 0.0f, kTol);
    EXPECT_NEAR(run(r, -100.0f, 2000.0f, 1000.0f, 25), -50.0f, kTol);
    EXPECT_EQ(run(r, -100.0f, 2000.0f, 1000.0f, 30), -100.0f);
}

TEST(Ramp, AcceleratesInNegativeDirection)
{
    Ramp r;

    // from 0 to negative: |output| grows -> rate_up
    EXPECT_NEAR(run(r, -3000.0f, 1000.0f, 100.0f, 100), -100.0f, kTol);
}

TEST(Ramp, ResetSetsOutput)
{
    Ramp r;
    run(r, 500.0f, 1000.0f, 1000.0f, 100);

    r.reset(42.0f);
    EXPECT_EQ(r.get_output(), 42.0f);
    r.reset();
    EXPECT_EQ(r.get_output(), 0.0f);
}
