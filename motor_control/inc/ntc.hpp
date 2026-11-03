#pragma once
#include <cstdint>

namespace motor_control
{
    // NTC from the ADC reference to the input, fixed resistor from the input to GND
    struct NtcParams
    {
        float r25_ohm;          // NTC resistance at 25 °C
        float beta;             // B value [K]
        float r_fixed_ohm;      // resistor to GND
    };

    // 12-bit ADC reading → temperature [°C] (Beta formula).
    // Open NTC (raw 0) reads very cold, shorted NTC (raw 4095) very hot.
    float ntc_temp_c(uint16_t raw, const NtcParams& p);
}
