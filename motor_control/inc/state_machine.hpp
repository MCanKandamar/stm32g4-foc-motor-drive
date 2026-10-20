#pragma once
#include <cstdint>

namespace motor_control
{
    enum class MotorState : uint8_t
    {
        IDLE,
        ADC_OFFSET_CAL,
        ALIGN,
        OPEN_LOOP,
        CLOSED_LOOP,
        FAULT
    };

    enum class MotorEvent : uint8_t
    {
        START,
        ADC_OFFSET_CAL_DONE,
        ALIGNMENT_DONE,
        CLOSED_LOOP_READY,
        STOP,
        FAULT_DETECTED,
        RESET_FAULT
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
        SENSOR_ERROR
    };

    enum class CtrlMode : uint8_t
    {
        ALIGNMENT,
        HALL_SENSOR_CAL,
        OPEN_LOOP_CTRL,
        CURRENT_CTRL,
        SPEED_CTRL
    };

    class StateMachine
    {
        public:

            void event_handler(MotorEvent event);

            MotorState getState() const { return motor_state;}

        private:
            volatile MotorState motor_state = MotorState::IDLE;

    };
}
