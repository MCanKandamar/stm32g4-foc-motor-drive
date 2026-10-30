#pragma once
#include "state_machine.hpp"
#include <cstdint>

namespace motor_control
{
    // No "Fault Reaction Active": here it is instant (power is cut at once).
    enum class Cia402State : uint8_t
    {
        NOT_READY_TO_SWITCH_ON, // MotorState::INIT: device is starting, controlword is ignored
        SWITCH_ON_DISABLED,
        READY_TO_SWITCH_ON,
        SWITCHED_ON,            // IDLE + current offset measured, power stage off
        OPERATION_ENABLED,      // motor is running (ALIGN … CLOSED_LOOP)
        QUICK_STOP_ACTIVE,
        FAULT
    };

    // Command decoded from the controlword. Bit 7 (fault reset) is checked separately, on its edge.
    enum class Cia402Cmd : uint8_t
    {
        NONE,
        SHUTDOWN,               // 0x06
        SWITCH_ON,              // 0x07 (in Operation Enabled: disable operation)
        ENABLE_OPERATION,       // 0x0F
        DISABLE_VOLTAGE,        // 0x00
        QUICK_STOP              // 0x02
    };

    Cia402Cmd decode_controlword(uint16_t controlword);

    // CiA 301 / CiA 402 emergency error code (0x603F)
    uint16_t cia402_error_code(FaultCode fault);

    // What the caller must do after update()
    struct Cia402Actions
    {
        bool enter_operation;   // start the motor in the selected mode
        bool leave_operation;   // stop the motor
        bool offset_cal;        // refresh the current offset
        bool reset_fault;       // ask for a fault reset
    };

    class Cia402
    {
        public:
            // controlword 0x6040, motor_state of the motor state machine
            Cia402Actions update(uint16_t controlword, MotorState motor_state);

            Cia402State getState() const { return state; }

            // Statusword 0x6041 without the mode bits 10 and 12.
            // voltage_ok: DC bus within limits (bit 4).
            uint16_t statusword(bool voltage_ok) const;

        private:
            Cia402State state = Cia402State::NOT_READY_TO_SWITCH_ON;
            bool run_seen = false;                  // in OE, the motor was seen leaving IDLE
            bool last_fault_reset_bit = false;
    };
}
