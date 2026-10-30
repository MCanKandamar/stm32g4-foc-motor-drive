// StateMachine tests: INIT, the Hall and sensorless start paths, stop, faults and calibration.

#include "state_machine.hpp"

#include <gtest/gtest.h>

#include <array>

using namespace motor_control;

namespace {

void go_to_idle(StateMachine& sm)
{
    sm.event_handler(MotorEvent::INIT_DONE);
    ASSERT_EQ(sm.getState(), MotorState::IDLE);
}

void go_to_open_loop(StateMachine& sm)
{
    go_to_idle(sm);
    sm.event_handler(MotorEvent::START_SENSORLESS);
    sm.event_handler(MotorEvent::ALIGNMENT_DONE);
    ASSERT_EQ(sm.getState(), MotorState::OPEN_LOOP);
}

}  // namespace

TEST(StateMachine, StartsInInit)
{
    StateMachine sm;
    EXPECT_EQ(sm.getState(), MotorState::INIT);
}

TEST(StateMachine, InitDoneGoesToIdle)
{
    StateMachine sm;
    sm.event_handler(MotorEvent::INIT_DONE);
    EXPECT_EQ(sm.getState(), MotorState::IDLE);
}

TEST(StateMachine, StopIsIgnoredInInit)
{
    StateMachine sm;
    sm.event_handler(MotorEvent::STOP);
    EXPECT_EQ(sm.getState(), MotorState::INIT);
}

TEST(StateMachine, StartIsIgnoredInInit)
{
    StateMachine sm;
    sm.event_handler(MotorEvent::START_HALL);
    sm.event_handler(MotorEvent::START_SENSORLESS);
    EXPECT_EQ(sm.getState(), MotorState::INIT);
}

TEST(StateMachine, HallStartGoesStraightToClosedLoop)
{
    StateMachine sm;
    go_to_idle(sm);
    sm.event_handler(MotorEvent::START_HALL);
    EXPECT_EQ(sm.getState(), MotorState::CLOSED_LOOP);
}

TEST(StateMachine, SensorlessStartRunsEveryStep)
{
    StateMachine sm;
    go_to_idle(sm);

    sm.event_handler(MotorEvent::START_SENSORLESS);
    EXPECT_EQ(sm.getState(), MotorState::ALIGN);

    sm.event_handler(MotorEvent::ALIGNMENT_DONE);
    EXPECT_EQ(sm.getState(), MotorState::OPEN_LOOP);

    sm.event_handler(MotorEvent::CLOSED_LOOP_READY);
    EXPECT_EQ(sm.getState(), MotorState::CLOSED_LOOP);
}

TEST(StateMachine, SensorlessStepsCannotBeSkipped)
{
    StateMachine sm;
    go_to_idle(sm);
    sm.event_handler(MotorEvent::START_SENSORLESS);

    sm.event_handler(MotorEvent::CLOSED_LOOP_READY);    // ALIGN is not done yet
    EXPECT_EQ(sm.getState(), MotorState::ALIGN);
}

TEST(StateMachine, StopFromAnyRunningStateGoesToIdle)
{
    StateMachine in_align;
    go_to_idle(in_align);
    in_align.event_handler(MotorEvent::START_SENSORLESS);
    in_align.event_handler(MotorEvent::STOP);
    EXPECT_EQ(in_align.getState(), MotorState::IDLE);

    StateMachine in_open_loop;
    go_to_open_loop(in_open_loop);
    in_open_loop.event_handler(MotorEvent::STOP);
    EXPECT_EQ(in_open_loop.getState(), MotorState::IDLE);

    StateMachine in_closed_loop;
    go_to_idle(in_closed_loop);
    in_closed_loop.event_handler(MotorEvent::START_HALL);
    in_closed_loop.event_handler(MotorEvent::STOP);
    EXPECT_EQ(in_closed_loop.getState(), MotorState::IDLE);

    StateMachine in_calibration;
    go_to_idle(in_calibration);
    in_calibration.event_handler(MotorEvent::CALIBRATE);
    in_calibration.event_handler(MotorEvent::STOP);
    EXPECT_EQ(in_calibration.getState(), MotorState::IDLE);
}

TEST(StateMachine, FaultWinsInEveryState)
{
    StateMachine in_init;
    in_init.event_handler(MotorEvent::FAULT_DETECTED);
    EXPECT_EQ(in_init.getState(), MotorState::FAULT);

    StateMachine in_idle;
    go_to_idle(in_idle);
    in_idle.event_handler(MotorEvent::FAULT_DETECTED);
    EXPECT_EQ(in_idle.getState(), MotorState::FAULT);

    StateMachine in_open_loop;
    go_to_open_loop(in_open_loop);
    in_open_loop.event_handler(MotorEvent::FAULT_DETECTED);
    EXPECT_EQ(in_open_loop.getState(), MotorState::FAULT);
}

TEST(StateMachine, OnlyResetLeavesFault)
{
    StateMachine sm;
    go_to_idle(sm);
    sm.event_handler(MotorEvent::FAULT_DETECTED);

    constexpr std::array others = {MotorEvent::INIT_DONE,      MotorEvent::START_HALL,
                                   MotorEvent::START_SENSORLESS, MotorEvent::ALIGNMENT_DONE,
                                   MotorEvent::CLOSED_LOOP_READY, MotorEvent::STOP,
                                   MotorEvent::CALIBRATE,      MotorEvent::CALIBRATION_DONE};
    for (MotorEvent event : others) {
        sm.event_handler(event);
        EXPECT_EQ(sm.getState(), MotorState::FAULT);
    }
}

TEST(StateMachine, ResetFaultGoesToIdleAfterInit)
{
    StateMachine sm;
    go_to_idle(sm);
    sm.event_handler(MotorEvent::FAULT_DETECTED);
    sm.event_handler(MotorEvent::RESET_FAULT);
    EXPECT_EQ(sm.getState(), MotorState::IDLE);
}

TEST(StateMachine, ResetFaultGoesBackToInitWhenInitNeverFinished)
{
    StateMachine sm;
    sm.event_handler(MotorEvent::FAULT_DETECTED);
    sm.event_handler(MotorEvent::RESET_FAULT);
    EXPECT_EQ(sm.getState(), MotorState::INIT);
}

TEST(StateMachine, CalibrationReturnsToIdle)
{
    StateMachine sm;
    go_to_idle(sm);

    sm.event_handler(MotorEvent::CALIBRATE);
    EXPECT_EQ(sm.getState(), MotorState::CALIBRATING);

    sm.event_handler(MotorEvent::CALIBRATION_DONE);
    EXPECT_EQ(sm.getState(), MotorState::IDLE);
}
