#pragma once
#include <cstdint>

namespace motor_control
{
    // Where the ISR gets the angle
    enum class AngleSource : uint8_t
    {
        FIXED,          // fixed angle (alignment)
        OPEN_LOOP,      // angle ramp
        HALL,           // Hall + PLL
        SMO             // SMO + PLL
    };

    // Which loop the ISR closes
    enum class LoopMode : uint8_t
    {
        OFF,            // FOC does not run
        CURRENT,        // idq_ref given directly
        SPEED           // idq_ref from the speed PI
    };

    // Angle source chosen by the user (fixed at START)
    enum class SensorMode : uint8_t
    {
        HALL,           // ADC → CL, angle from Hall
        SENSORLESS      // ADC → ALIGN → OL → CL, angle from SMO
    };
}
