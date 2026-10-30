// CiA 402 tests: controlword decoding, the state transitions, what the drive asks the motor to do,
// the statusword and the fault error codes.

#include "cia402.hpp"

#include <gtest/gtest.h>

using namespace motor_control;

namespace {

constexpr uint16_t kShutdown = 0x0006U;
constexpr uint16_t kSwitchOn = 0x0007U;
constexpr uint16_t kEnable = 0x000FU;
constexpr uint16_t kDisableVoltage = 0x0000U;
constexpr uint16_t kQuickStop = 0x0002U;
constexpr uint16_t kFaultReset = 0x0080U;

void to_switch_on_disabled(Cia402& drive)
{
    drive.update(kDisableVoltage, MotorState::IDLE);
    ASSERT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

void to_switched_on(Cia402& drive)
{
    to_switch_on_disabled(drive);
    drive.update(kShutdown, MotorState::IDLE);
    drive.update(kSwitchOn, MotorState::IDLE);
    ASSERT_EQ(drive.getState(), Cia402State::SWITCHED_ON);
}

void to_operation_enabled(Cia402& drive)
{
    to_switched_on(drive);
    drive.update(kEnable, MotorState::IDLE);
    ASSERT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

void to_fault(Cia402& drive)
{
    drive.update(kDisableVoltage, MotorState::FAULT);
    ASSERT_EQ(drive.getState(), Cia402State::FAULT);
}

}  // namespace

TEST(Cia402Decode, StandardCommands)
{
    EXPECT_EQ(decode_controlword(kShutdown), Cia402Cmd::SHUTDOWN);
    EXPECT_EQ(decode_controlword(kSwitchOn), Cia402Cmd::SWITCH_ON);
    EXPECT_EQ(decode_controlword(kEnable), Cia402Cmd::ENABLE_OPERATION);
    EXPECT_EQ(decode_controlword(kDisableVoltage), Cia402Cmd::DISABLE_VOLTAGE);
    EXPECT_EQ(decode_controlword(kQuickStop), Cia402Cmd::QUICK_STOP);
}

TEST(Cia402Decode, DontCareBitsAreIgnored)
{
    EXPECT_EQ(decode_controlword(0x000DU), Cia402Cmd::DISABLE_VOLTAGE);     // 0xxx xx0x
    EXPECT_EQ(decode_controlword(0x000BU), Cia402Cmd::QUICK_STOP);          // 0xxx x01x
    EXPECT_EQ(decode_controlword(0x000EU), Cia402Cmd::SHUTDOWN);            // 0xxx x110
    EXPECT_EQ(decode_controlword(0x0107U), Cia402Cmd::SWITCH_ON);           // bit 8 is not in the table
    EXPECT_EQ(decode_controlword(0x010FU), Cia402Cmd::ENABLE_OPERATION);
}

TEST(Cia402Decode, FaultResetBitIsNoCommand)
{
    EXPECT_EQ(decode_controlword(kFaultReset), Cia402Cmd::NONE);
    EXPECT_EQ(decode_controlword(kFaultReset | kEnable), Cia402Cmd::NONE);
}

TEST(Cia402ErrorCode, EveryFaultHasItsCode)
{
    EXPECT_EQ(cia402_error_code(FaultCode::NONE), 0x0000U);
    EXPECT_EQ(cia402_error_code(FaultCode::DRIVER_FAULT), 0x5000U);
    EXPECT_EQ(cia402_error_code(FaultCode::MOTOR_OVER_CURRENT), 0x2310U);
    EXPECT_EQ(cia402_error_code(FaultCode::DRIVER_OVER_CURRENT), 0x2300U);
    EXPECT_EQ(cia402_error_code(FaultCode::OVER_VOLTAGE), 0x3210U);
    EXPECT_EQ(cia402_error_code(FaultCode::UNDER_VOLTAGE), 0x3220U);
    EXPECT_EQ(cia402_error_code(FaultCode::MOTOR_OVER_TEMP), 0x4310U);
    EXPECT_EQ(cia402_error_code(FaultCode::DRIVER_OVER_TEMP), 0x4210U);
    EXPECT_EQ(cia402_error_code(FaultCode::SENSOR_ERROR), 0x7300U);
    EXPECT_EQ(cia402_error_code(FaultCode::SENSORLESS_LOST), 0xFF01U);
}

TEST(Cia402, StartsNotReady)
{
    Cia402 drive;
    EXPECT_EQ(drive.getState(), Cia402State::NOT_READY_TO_SWITCH_ON);
    EXPECT_EQ(drive.statusword(true), 0x0000U);
}

TEST(Cia402, ControlwordIsIgnoredDuringInit)
{
    Cia402 drive;

    const Cia402Actions action = drive.update(kEnable, MotorState::INIT);
    EXPECT_EQ(drive.getState(), Cia402State::NOT_READY_TO_SWITCH_ON);
    EXPECT_FALSE(action.enter_operation);
    EXPECT_FALSE(action.offset_cal);
}

TEST(Cia402, GoesToSwitchOnDisabledAfterInit)
{
    Cia402 drive;
    drive.update(kDisableVoltage, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, StandardStartSequence)
{
    Cia402 drive;
    to_switch_on_disabled(drive);

    EXPECT_FALSE(drive.update(kShutdown, MotorState::IDLE).offset_cal);
    EXPECT_EQ(drive.getState(), Cia402State::READY_TO_SWITCH_ON);

    EXPECT_TRUE(drive.update(kSwitchOn, MotorState::IDLE).offset_cal);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCHED_ON);

    EXPECT_TRUE(drive.update(kEnable, MotorState::IDLE).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

TEST(Cia402, EnableInReadyGoesThroughSwitchedOn)
{
    Cia402 drive;
    to_switch_on_disabled(drive);
    drive.update(kShutdown, MotorState::IDLE);

    // 0x0F from Ready: one cycle in Switched On for the offset, then Operation Enabled
    EXPECT_TRUE(drive.update(kEnable, MotorState::IDLE).offset_cal);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCHED_ON);

    EXPECT_TRUE(drive.update(kEnable, MotorState::IDLE).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

TEST(Cia402, EnableWaitsForIdleMotor)
{
    Cia402 drive;
    to_switched_on(drive);

    EXPECT_FALSE(drive.update(kEnable, MotorState::CALIBRATING).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCHED_ON);
}

TEST(Cia402, EnableCannotSkipReadyToSwitchOn)
{
    Cia402 drive;
    to_switch_on_disabled(drive);

    EXPECT_FALSE(drive.update(kEnable, MotorState::IDLE).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, ReadyAndSwitchedOnGoBackOnDisableOrQuickStop)
{
    for (uint16_t command : {kDisableVoltage, kQuickStop}) {
        Cia402 ready;
        to_switch_on_disabled(ready);
        ready.update(kShutdown, MotorState::IDLE);
        ready.update(command, MotorState::IDLE);
        EXPECT_EQ(ready.getState(), Cia402State::SWITCH_ON_DISABLED);

        Cia402 switched_on;
        to_switched_on(switched_on);
        switched_on.update(command, MotorState::IDLE);
        EXPECT_EQ(switched_on.getState(), Cia402State::SWITCH_ON_DISABLED);
    }
}

TEST(Cia402, ShutdownInSwitchedOnGoesToReady)
{
    Cia402 drive;
    to_switched_on(drive);
    drive.update(kShutdown, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::READY_TO_SWITCH_ON);
}

TEST(Cia402, LeavingOperationStopsTheMotor)
{
    struct Case { uint16_t command; Cia402State next; };
    constexpr Case cases[] = {
        {kSwitchOn, Cia402State::SWITCHED_ON},
        {kShutdown, Cia402State::READY_TO_SWITCH_ON},
        {kDisableVoltage, Cia402State::SWITCH_ON_DISABLED},
        {kQuickStop, Cia402State::QUICK_STOP_ACTIVE},
    };

    for (const Case& c : cases) {
        Cia402 drive;
        to_operation_enabled(drive);
        drive.update(kEnable, MotorState::CLOSED_LOOP);

        EXPECT_TRUE(drive.update(c.command, MotorState::CLOSED_LOOP).leave_operation);
        EXPECT_EQ(drive.getState(), c.next);
    }
}

TEST(Cia402, HoldingEnableDoesNotStopTheMotor)
{
    Cia402 drive;
    to_operation_enabled(drive);

    EXPECT_FALSE(drive.update(kEnable, MotorState::CLOSED_LOOP).leave_operation);
    EXPECT_FALSE(drive.update(kEnable, MotorState::CLOSED_LOOP).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

TEST(Cia402, StaysEnabledWhileTheStartIsPending)
{
    Cia402 drive;
    to_operation_enabled(drive);

    // The Supervisor has not left IDLE yet: this is not a stop
    drive.update(kEnable, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

TEST(Cia402, UnexpectedStopDoesNotRestartByItself)
{
    Cia402 drive;
    to_operation_enabled(drive);
    drive.update(kEnable, MotorState::CLOSED_LOOP);

    // The motor dropped to IDLE while the master still sends 0x0F
    drive.update(kEnable, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);

    EXPECT_FALSE(drive.update(kEnable, MotorState::IDLE).enter_operation);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, SecondRunStartsWithAFreshRunFlag)
{
    Cia402 drive;
    to_operation_enabled(drive);
    drive.update(kEnable, MotorState::CLOSED_LOOP);
    drive.update(kSwitchOn, MotorState::CLOSED_LOOP);
    ASSERT_EQ(drive.getState(), Cia402State::SWITCHED_ON);

    drive.update(kEnable, MotorState::IDLE);
    ASSERT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);

    // IDLE right after the new start must not count as a stop
    drive.update(kEnable, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::OPERATION_ENABLED);
}

TEST(Cia402, QuickStopEndsWhenTheMotorStops)
{
    Cia402 drive;
    to_operation_enabled(drive);
    drive.update(kQuickStop, MotorState::CLOSED_LOOP);

    drive.update(kQuickStop, MotorState::CLOSED_LOOP);
    EXPECT_EQ(drive.getState(), Cia402State::QUICK_STOP_ACTIVE);

    drive.update(kQuickStop, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, DisableVoltageEndsQuickStop)
{
    Cia402 drive;
    to_operation_enabled(drive);
    drive.update(kQuickStop, MotorState::CLOSED_LOOP);

    drive.update(kDisableVoltage, MotorState::CLOSED_LOOP);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, MotorFaultWinsInEveryState)
{
    Cia402 enabled;
    to_operation_enabled(enabled);
    enabled.update(kEnable, MotorState::FAULT);
    EXPECT_EQ(enabled.getState(), Cia402State::FAULT);

    Cia402 starting;
    starting.update(kDisableVoltage, MotorState::FAULT);
    EXPECT_EQ(starting.getState(), Cia402State::FAULT);
}

TEST(Cia402, FaultResetActsOnTheRisingEdgeOnly)
{
    Cia402 drive;
    to_fault(drive);

    EXPECT_TRUE(drive.update(kFaultReset, MotorState::FAULT).reset_fault);
    EXPECT_FALSE(drive.update(kFaultReset, MotorState::FAULT).reset_fault);

    drive.update(kDisableVoltage, MotorState::FAULT);
    EXPECT_TRUE(drive.update(kFaultReset, MotorState::FAULT).reset_fault);
}

TEST(Cia402, ClearedFaultGoesToSwitchOnDisabled)
{
    Cia402 drive;
    to_fault(drive);

    drive.update(kFaultReset, MotorState::IDLE);
    EXPECT_EQ(drive.getState(), Cia402State::SWITCH_ON_DISABLED);
}

TEST(Cia402, FaultResetBackToInitIsNotReady)
{
    Cia402 drive;
    to_fault(drive);

    drive.update(kFaultReset, MotorState::INIT);
    EXPECT_EQ(drive.getState(), Cia402State::NOT_READY_TO_SWITCH_ON);
}

TEST(Cia402, StatuswordOfEachState)
{
    Cia402 drive;
    to_switch_on_disabled(drive);
    EXPECT_EQ(drive.statusword(true), 0x0250U);

    drive.update(kShutdown, MotorState::IDLE);
    EXPECT_EQ(drive.statusword(true), 0x0231U);

    drive.update(kSwitchOn, MotorState::IDLE);
    EXPECT_EQ(drive.statusword(true), 0x0233U);

    drive.update(kEnable, MotorState::IDLE);
    EXPECT_EQ(drive.statusword(true), 0x0237U);

    drive.update(kQuickStop, MotorState::IDLE);
    EXPECT_EQ(drive.statusword(true), 0x0217U);

    drive.update(kQuickStop, MotorState::FAULT);
    EXPECT_EQ(drive.statusword(true), 0x0218U);
}

TEST(Cia402, VoltageBitFollowsTheDcBus)
{
    Cia402 drive;
    to_switch_on_disabled(drive);

    EXPECT_EQ(drive.statusword(true) & 0x0010U, 0x0010U);
    EXPECT_EQ(drive.statusword(false) & 0x0010U, 0x0000U);
}
