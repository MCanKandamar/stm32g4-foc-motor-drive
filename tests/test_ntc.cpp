// ntc tests: ADC reading to temperature for the board NTC (10k, B 3950, 10k to GND).

#include "ntc.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace motor_control;

namespace {

constexpr NtcParams kBoard = {10'000.0f, 3950.0f, 10'000.0f};

}  // namespace

TEST(Ntc, HalfScaleIs25C)
{
    // r_ntc = r_fixed = 10k → 25 °C
    EXPECT_NEAR(ntc_temp_c(2048, kBoard), 25.0f, 0.1f);
}

TEST(Ntc, ReadsKnownPoint)
{
    // 85 °C: r_ntc = 10k * exp(3950 * (1/358.15 - 1/298.15)) ≈ 1087 Ω → raw ≈ 3694
    EXPECT_NEAR(ntc_temp_c(3694, kBoard), 85.0f, 0.2f);
}

TEST(Ntc, HotterGivesHigherRaw)
{
    EXPECT_GT(ntc_temp_c(3000, kBoard), ntc_temp_c(2000, kBoard));
}

TEST(Ntc, OpenAndShortStayFinite)
{
    const float open    = ntc_temp_c(0, kBoard);
    const float shorted = ntc_temp_c(4095, kBoard);
    EXPECT_TRUE(std::isfinite(open));
    EXPECT_TRUE(std::isfinite(shorted));
    EXPECT_LT(open, -40.0f);      // reads cold: no false trip
    EXPECT_GT(shorted, 150.0f);   // reads hot: trips
}
