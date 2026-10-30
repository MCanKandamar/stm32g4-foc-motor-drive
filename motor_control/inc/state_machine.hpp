#pragma once
#include <cstdint>

namespace motor_control
{
    enum class MotorState : uint8_t
    {
        INIT,               // power-up: device setup and current offset, then IDLE
        IDLE,
        ALIGN,
        OPEN_LOOP,
        CLOSED_LOOP,
        FAULT,
        CALIBRATING         // measure the Hall edge angles, then IDLE
    };

    enum class MotorEvent : uint8_t
    {
        INIT_DONE,
        START_HALL,         // IDLE → CLOSED_LOOP
        START_SENSORLESS,   // IDLE → ALIGN → OPEN_LOOP → CLOSED_LOOP
        ALIGNMENT_DONE,
        CLOSED_LOOP_READY,
        STOP,
        FAULT_DETECTED,
        RESET_FAULT,
        CALIBRATE,
        CALIBRATION_DONE
    };

    enum class FaultCode : uint8_t
    {
        NONE,
        DRIVER_FAULT,       // gate driver nFAULT
        MOTOR_OVER_CURRENT,
        DRIVER_OVER_CURRENT,
        OVER_VOLTAGE,
        UNDER_VOLTAGE,
        MOTOR_OVER_TEMP,
        DRIVER_OVER_TEMP,
        SENSOR_ERROR,
        SENSORLESS_LOST     // SMO speed below SL_LOST_RPM: angle estimate is not reliable
    };

    class StateMachine
    {
        public:

            void event_handler(MotorEvent event);

            MotorState getState() const { return motor_state;}

        private:
            volatile MotorState motor_state = MotorState::INIT;
            bool                init_done   = false;    // INIT finished once; a fault reset goes back to IDLE

    };
}
