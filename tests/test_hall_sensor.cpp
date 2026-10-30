// HallSensor tests: sector order, direction, speed from edge periods, the angle between edges,
// timeout and edge calibration.
//
// Timer: 1 MHz, as on the board. One edge every 1000 counts = 60 electrical degrees per ms:
//   omega_e = (π/3) * 1 000 000 / 1000 = 1047.2 rad/s, with 4 pole pairs = 2500 rpm

#include "hall_sensor.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

using namespace motor_control;

namespace {

constexpr float kTol = 1e-3f;
constexpr float kFcnt = 1'000'000.0f;
constexpr uint32_t kPeriod = 1000;
constexpr float kOmega = PI_DIV_3 * kFcnt / kPeriod;

// Hall state of each sector, in the forward order
constexpr std::array<uint8_t, 6> kForward = {1, 5, 4, 6, 2, 3};

HallConfig config()
{
    return {.f_cnt_hz = kFcnt, .pole_pairs = 4};
}

// One Hall edge forward or backward; 'sector' follows the rotor
void step_forward(HallSensor& hall, int& sector, uint32_t period = kPeriod)
{
    sector = (sector + 1) % 6;
    hall.update(kForward[sector], period, 0);
}

void step_backward(HallSensor& hall, int& sector, uint32_t period = kPeriod)
{
    sector = (sector + 5) % 6;
    hall.update(kForward[sector], period, 0);
}

// One turn forward. At each edge the reference angle is 'edge * π/3 * sign + offset'.
void calibration_turn(HallSensor& hall, int& sector, float offset, float sign = 1.0f)
{
    for (int i = 0; i < 6; ++i) {
        step_forward(hall, sector);
        hall.recordCalibration(sign * sector * PI_DIV_3 + offset);
    }
}

}  // namespace

TEST(HallSensor, InitRejectsInvalidStates)
{
    HallSensor hall(config());

    EXPECT_FALSE(hall.init(0));
    EXPECT_FALSE(hall.init(7));
    EXPECT_TRUE(hall.init(1));
}

TEST(HallSensor, InitPutsTheAngleInTheMiddleOfTheSector)
{
    HallSensor hall(config());

    hall.init(kForward[0]);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI / 6.0f, kTol);     // 30°

    hall.init(kForward[5]);
    EXPECT_NEAR(hall.getElectricalAngleRad(), 11.0f * PI / 6.0f, kTol);     // 330°
}

TEST(HallSensor, AngleStaysStillUntilTheFirstEdge)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.update(kForward[0], 0, 1000);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI / 6.0f, kTol);
    EXPECT_EQ(hall.getDirection(), 0);
}

TEST(HallSensor, ForwardEdgeGivesForwardDirection)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    EXPECT_EQ(hall.getDirection(), 1);
}

TEST(HallSensor, BackwardEdgeGivesBackwardDirection)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_backward(hall, sector);
    EXPECT_EQ(hall.getDirection(), -1);
}

TEST(HallSensor, FirstEdgeAfterStopGivesNoSpeed)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    // The time before this edge is unknown, so its period cannot be used
    int sector = 0;
    step_forward(hall, sector);
    EXPECT_FLOAT_EQ(hall.getElectricalSpeedRads(), 0.0f);
}

TEST(HallSensor, SpeedComesFromTheEdgePeriod)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);

    EXPECT_NEAR(hall.getElectricalSpeedRads(), kOmega, 0.1f);
    EXPECT_NEAR(hall.getMechanicalSpeedRpm(), 2500.0f, 0.1f);
}

TEST(HallSensor, SpeedIsNegativeInReverse)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_backward(hall, sector);
    step_backward(hall, sector);

    EXPECT_NEAR(hall.getElectricalSpeedRads(), -kOmega, 0.1f);
    EXPECT_NEAR(hall.getMechanicalSpeedRpm(), -2500.0f, 0.1f);
}

TEST(HallSensor, SpeedIsTheAverageOfTheLast24Edges)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    for (int i = 0; i < 12; ++i) {
        step_forward(hall, sector, 1000);
    }
    for (int i = 0; i < 12; ++i) {
        step_forward(hall, sector, 2000);
    }
    // 24 edges = 8π rad in 36 000 counts = 36 ms  ->  698.13 rad/s
    EXPECT_NEAR(hall.getElectricalSpeedRads(), 698.13f, 0.1f);

    // 24 new edges push the old periods out of the buffer
    for (int i = 0; i < 24; ++i) {
        step_forward(hall, sector, 1000);
    }
    EXPECT_NEAR(hall.getElectricalSpeedRads(), kOmega, 0.1f);
}

TEST(HallSensor, DirectionChangeRestartsTheAverage)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    for (int i = 0; i < 10; ++i) {
        step_forward(hall, sector);
    }

    // Only the new period counts: (π/3) * 1e6 / 2000 = 523.6 rad/s backward
    step_backward(hall, sector, 2000);
    EXPECT_NEAR(hall.getElectricalSpeedRads(), -523.6f, 0.1f);
}

TEST(HallSensor, ForwardEdgeSetsTheLowerEdgeOfTheSector)
{
    HallSensor hall(config());
    hall.init(kForward[3]);

    int sector = 3;
    step_forward(hall, sector);
    EXPECT_NEAR(hall.getElectricalAngleRad(), 4.0f * PI_DIV_3, kTol);
}

TEST(HallSensor, BackwardEdgeSetsTheUpperEdgeOfTheSector)
{
    HallSensor hall(config());
    hall.init(kForward[3]);

    int sector = 3;
    step_backward(hall, sector);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI, kTol);    // upper edge of sector 2
}

TEST(HallSensor, AngleMovesBetweenEdges)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);     // sector 2, edge at 120°

    // 500 counts = half of a 60° sector: 120° + 30°
    hall.update(kForward[2], kPeriod, 500);
    EXPECT_NEAR(hall.getElectricalAngleRad(), 5.0f * PI / 6.0f, kTol);
}

TEST(HallSensor, AngleStopsAtTheSectorWidth)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);

    // The next edge is late: the angle waits at 120° + 60°
    hall.update(kForward[2], kPeriod, 5000);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI, kTol);
}

TEST(HallSensor, AngleWrapsAboveTwoPi)
{
    HallSensor hall(config());
    hall.init(kForward[3]);

    int sector = 3;
    step_forward(hall, sector);
    step_forward(hall, sector);     // sector 5, edge at 300°

    hall.update(kForward[5], kPeriod, 900);     // 300° + 54° = 354°
    EXPECT_NEAR(hall.getElectricalAngleRad(), 354.0f * PI / 180.0f, kTol);

    hall.update(kForward[5], kPeriod, 5000);    // limited to 360°, which wraps to 0°
    const float theta = hall.getElectricalAngleRad();
    EXPECT_GE(theta, 0.0f);
    EXPECT_LT(theta, TWO_PI);
}

TEST(HallSensor, TimeoutStopsAndCentresTheAngle)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);     // sector 2, moving

    // The limit is min(0.1 s = 100 000, 65535 / 5 * 4 = 52 428) counts
    hall.update(kForward[2], kPeriod, 60000);
    EXPECT_FLOAT_EQ(hall.getElectricalSpeedRads(), 0.0f);
    EXPECT_NEAR(hall.getElectricalAngleRad(), 5.0f * PI / 6.0f, kTol);  // 150°
}

TEST(HallSensor, NoTimeoutBelowTheLimit)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);

    hall.update(kForward[2], kPeriod, 50000);
    EXPECT_NEAR(hall.getElectricalSpeedRads(), kOmega, 0.1f);
}

TEST(HallSensor, FirstEdgeAfterTimeoutGivesNoSpeed)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    step_forward(hall, sector);
    step_forward(hall, sector);
    hall.update(kForward[2], kPeriod, 60000);

    step_forward(hall, sector);
    EXPECT_FLOAT_EQ(hall.getElectricalSpeedRads(), 0.0f);

    step_forward(hall, sector);
    EXPECT_NEAR(hall.getElectricalSpeedRads(), kOmega, 0.1f);
}

TEST(HallSensor, InvalidStatesAreSkipped)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.update(0, kPeriod, 0);
    hall.update(7, kPeriod, 0);
    EXPECT_EQ(hall.getDirection(), 0);

    // The last good sector is still 0, so this is a normal forward edge
    int sector = 0;
    step_forward(hall, sector);
    EXPECT_EQ(hall.getDirection(), 1);
}

TEST(HallSensor, SkippedSectorIsNotAnEdge)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.update(kForward[2], kPeriod, 0);   // two sectors at once: noise
    EXPECT_EQ(hall.getDirection(), 0);
}

TEST(HallSensor, ZeroPeriodIsIgnored)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.update(kForward[1], 0, 0);
    EXPECT_EQ(hall.getDirection(), 0);
}

TEST(HallSensor, CalibrationUpdatesTheEdgeTable)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.startCalibration();
    int sector = 0;
    calibration_turn(hall, sector, 0.1f);
    ASSERT_TRUE(hall.finishCalibration());

    // All edges are 0.1 rad later, so the middle of sector 0 is too
    hall.init(kForward[0]);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI / 6.0f + 0.1f, kTol);
}

TEST(HallSensor, CalibrationAveragesAcrossZero)
{
    HallSensor hall(config());
    hall.init(kForward[5]);

    // Edge 0 is seen at 350° and 10°: the vector mean is 0°, not 180°
    hall.startCalibration();
    int sector = 5;
    for (float edge_0 : {350.0f, 10.0f}) {
        for (int i = 0; i < 6; ++i) {
            step_forward(hall, sector);
            const float theta = (sector == 0) ? edge_0 * PI / 180.0f : sector * PI_DIV_3;
            hall.recordCalibration(theta);
        }
    }
    ASSERT_TRUE(hall.finishCalibration());

    hall.init(kForward[0]);
    EXPECT_NEAR(std::sin(hall.getElectricalAngleRad()), 0.5f, kTol);    // middle of sector 0 = 30°
}

TEST(HallSensor, CalibrationFailsWhenAnEdgeIsMissing)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.startCalibration();
    int sector = 0;
    for (int i = 0; i < 3; ++i) {
        step_forward(hall, sector);
        hall.recordCalibration(sector * PI_DIV_3);
    }
    EXPECT_FALSE(hall.finishCalibration());

    // The old table stays
    hall.init(kForward[0]);
    EXPECT_NEAR(hall.getElectricalAngleRad(), PI / 6.0f, kTol);
}

TEST(HallSensor, CalibrationFailsOnInvalidState)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.startCalibration();
    int sector = 0;
    calibration_turn(hall, sector, 0.0f);
    hall.update(7, kPeriod, 0);
    EXPECT_FALSE(hall.finishCalibration());
}

TEST(HallSensor, CalibrationFailsOnSkippedSector)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    hall.startCalibration();
    int sector = 0;
    calibration_turn(hall, sector, 0.0f);
    hall.update(kForward[(sector + 2) % 6], kPeriod, 0);
    EXPECT_FALSE(hall.finishCalibration());
}

TEST(HallSensor, CalibrationFailsWhenEdgesAreInReverseOrder)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    // Phase or Hall order swapped: the reference runs against the Hall order, sectors come out ~300°
    hall.startCalibration();
    int sector = 0;
    calibration_turn(hall, sector, 0.0f, -1.0f);
    EXPECT_FALSE(hall.finishCalibration());
}

TEST(HallSensor, RecordIsIgnoredWithoutCalibration)
{
    HallSensor hall(config());
    hall.init(kForward[0]);

    int sector = 0;
    calibration_turn(hall, sector, 0.5f);       // not started: nothing is recorded
    hall.startCalibration();
    EXPECT_FALSE(hall.finishCalibration());
}
