#include "cia402.hpp"

namespace motor_control
{
    // CiA 402 command table: bits 7 3 2 1 0, x = don't care
    Cia402Cmd decode_controlword(uint16_t controlword)
    {
        const uint16_t cw = controlword;

        if ((cw & 0x0082U) == 0x0000U) return Cia402Cmd::DISABLE_VOLTAGE;     // 0xxx xx0x
        if ((cw & 0x0086U) == 0x0002U) return Cia402Cmd::QUICK_STOP;          // 0xxx x01x
        if ((cw & 0x0087U) == 0x0006U) return Cia402Cmd::SHUTDOWN;            // 0xxx x110
        if ((cw & 0x008FU) == 0x0007U) return Cia402Cmd::SWITCH_ON;           // 0xxx 0111
        if ((cw & 0x008FU) == 0x000FU) return Cia402Cmd::ENABLE_OPERATION;    // 0xxx 1111
        return Cia402Cmd::NONE;
    }

    uint16_t cia402_error_code(FaultCode fault)
    {
        switch (fault)
        {
            case FaultCode::NONE:                return 0x0000U;
            case FaultCode::DRIVER_FAULT:        return 0x5000U;    // device hardware (gate driver nFAULT)
            case FaultCode::MOTOR_OVER_CURRENT:  return 0x2310U;    // continuous over current, output side
            case FaultCode::DRIVER_OVER_CURRENT: return 0x2300U;    // current, device output side
            case FaultCode::OVER_VOLTAGE:        return 0x3210U;    // DC link over-voltage
            case FaultCode::UNDER_VOLTAGE:       return 0x3220U;    // DC link under-voltage
            case FaultCode::MOTOR_OVER_TEMP:     return 0x4310U;    // excess temperature drive (motor)
            case FaultCode::DRIVER_OVER_TEMP:    return 0x4210U;    // excess temperature device
            case FaultCode::SENSOR_ERROR:        return 0x7300U;    // sensor
            case FaultCode::SENSORLESS_LOST:     return 0xFF01U;    // device specific: SMO lock lost
        }
        return 0xFF00U;
    }

    Cia402Actions Cia402::update(uint16_t controlword, MotorState motor_state)
    {
        Cia402Actions action{};

        // 1) MotorState always wins
        if (motor_state == MotorState::FAULT)
        {
            state = Cia402State::FAULT;
        }
        else if (motor_state == MotorState::INIT)
        {
            state = Cia402State::NOT_READY_TO_SWITCH_ON;
        }
        else if (state == Cia402State::FAULT || state == Cia402State::NOT_READY_TO_SWITCH_ON)
        {
            state = Cia402State::SWITCH_ON_DISABLED;        // fault reset or INIT done
        }
        else if (state == Cia402State::OPERATION_ENABLED)
        {
            if (motor_state != MotorState::IDLE)
                run_seen = true;
            else if (run_seen)                              // unexpected stop: do not restart by itself
                state = Cia402State::SWITCH_ON_DISABLED;
        }
        else if (state == Cia402State::QUICK_STOP_ACTIVE && motor_state == MotorState::IDLE)
        {
            state = Cia402State::SWITCH_ON_DISABLED;        // motor stopped
        }

        // 2) Controlword
        const Cia402Cmd c      = decode_controlword(controlword);
        const bool      fr_bit = (controlword & 0x0080U) != 0U;
        const bool      fault_reset_edge = fr_bit && !last_fault_reset_bit;
        last_fault_reset_bit = fr_bit;

        // 3) Transitions
        switch (state)
        {
            case Cia402State::NOT_READY_TO_SWITCH_ON:
                break;                          // no commands until INIT is done

            case Cia402State::SWITCH_ON_DISABLED:
                if (c == Cia402Cmd::SHUTDOWN)
                    state = Cia402State::READY_TO_SWITCH_ON;
                break;

            case Cia402State::READY_TO_SWITCH_ON:
                if (c == Cia402Cmd::SWITCH_ON || c == Cia402Cmd::ENABLE_OPERATION)
                {
                    state             = Cia402State::SWITCHED_ON;   // 0x0F: OE on the next cycle
                    action.offset_cal = true;
                }
                else if (c == Cia402Cmd::DISABLE_VOLTAGE || c == Cia402Cmd::QUICK_STOP)
                    state = Cia402State::SWITCH_ON_DISABLED;
                break;

            case Cia402State::SWITCHED_ON:
                if (c == Cia402Cmd::ENABLE_OPERATION && motor_state == MotorState::IDLE)
                {
                    state                  = Cia402State::OPERATION_ENABLED;
                    run_seen               = false;
                    action.enter_operation = true;
                }
                else if (c == Cia402Cmd::SHUTDOWN)
                    state = Cia402State::READY_TO_SWITCH_ON;
                else if (c == Cia402Cmd::DISABLE_VOLTAGE || c == Cia402Cmd::QUICK_STOP)
                    state = Cia402State::SWITCH_ON_DISABLED;
                break;

            case Cia402State::OPERATION_ENABLED:
                if (c == Cia402Cmd::SWITCH_ON)
                    state = Cia402State::SWITCHED_ON;
                else if (c == Cia402Cmd::SHUTDOWN)
                    state = Cia402State::READY_TO_SWITCH_ON;
                else if (c == Cia402Cmd::DISABLE_VOLTAGE)
                    state = Cia402State::SWITCH_ON_DISABLED;
                else if (c == Cia402Cmd::QUICK_STOP)
                    state = Cia402State::QUICK_STOP_ACTIVE;     // coast to stop, no braking
                action.leave_operation = (state != Cia402State::OPERATION_ENABLED);
                break;

            case Cia402State::QUICK_STOP_ACTIVE:
                if (c == Cia402Cmd::DISABLE_VOLTAGE)
                    state = Cia402State::SWITCH_ON_DISABLED;
                break;

            case Cia402State::FAULT:
                action.reset_fault = fault_reset_edge;  // when FAULT clears, step 1 moves to SOD
                break;
        }

        return action;
    }

    uint16_t Cia402::statusword(bool voltage_ok) const
    {
        uint16_t sw = 0U;

        switch (state)
        {
            case Cia402State::NOT_READY_TO_SWITCH_ON:
                return 0x0000U;                     // standard: not ready, no extra bits
            case Cia402State::SWITCH_ON_DISABLED: sw = 0x0040U; break;
            case Cia402State::READY_TO_SWITCH_ON: sw = 0x0021U; break;
            case Cia402State::SWITCHED_ON:        sw = 0x0023U; break;
            case Cia402State::OPERATION_ENABLED:  sw = 0x0027U; break;
            case Cia402State::QUICK_STOP_ACTIVE:  sw = 0x0007U; break;
            case Cia402State::FAULT:              sw = 0x0008U; break;
        }

        if (voltage_ok)
            sw |= 0x0010U;                          // bit 4: voltage enabled
        sw |= 0x0200U;                              // bit 9: remote, controlword is used

        return sw;
    }
}
