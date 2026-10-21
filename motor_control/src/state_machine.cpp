#include "state_machine.hpp"
#include <cassert>

namespace motor_control
{

    void StateMachine::event_handler(MotorEvent event)
    {
        if (event == MotorEvent::FAULT_DETECTED)
        {
            motor_state = MotorState::FAULT;
        }

        if (event == MotorEvent::STOP && motor_state != MotorState::FAULT)
        {
            motor_state = MotorState::IDLE;
        }

        switch (motor_state)
        {
            case motor_control::MotorState::IDLE:
                if (event == MotorEvent::START)
                    motor_state = MotorState::ADC_OFFSET_CAL;
                else if (event == MotorEvent::CALIBRATE)
                    motor_state = MotorState::CALIBRATING;
                break;

            case motor_control::MotorState::CALIBRATING:
                if (event == MotorEvent::CALIBRATION_DONE)
                    motor_state = MotorState::IDLE;
                break;

            case motor_control::MotorState::ADC_OFFSET_CAL:
                if (event == MotorEvent::ADC_OFFSET_CAL_DONE)
                    motor_state = MotorState::ALIGN;
                break;
            
            case motor_control::MotorState::ALIGN:
                if (event == MotorEvent::ALIGNMENT_DONE)
                    motor_state = MotorState::OPEN_LOOP;
                break;

            case motor_control::MotorState::OPEN_LOOP:
                if (event == MotorEvent::CLOSED_LOOP_READY)
                    motor_state = MotorState::CLOSED_LOOP;
                break;

            case motor_control::MotorState::CLOSED_LOOP:
                if (event == MotorEvent::STOP)
                    motor_state = MotorState::IDLE;
                break;

            case motor_control::MotorState::FAULT:
                if (event == MotorEvent::RESET_FAULT)
                    motor_state = MotorState::IDLE;
                break;
        }
    }
}
