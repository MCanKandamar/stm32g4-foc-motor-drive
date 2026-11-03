#include "ntc.hpp"
#include <cmath>

namespace motor_control
{
    float ntc_temp_c(uint16_t raw, const NtcParams& p)
    {
        constexpr float ADC_FULL = 4095.0f;
        constexpr float T25_K    = 298.15f;
        constexpr float K_TO_C   = 273.15f;

        // keep away from 0 and full scale: no divide by zero, no log(0)
        const float x = std::fmin(std::fmax(static_cast<float>(raw), 1.0f), ADC_FULL - 1.0f);

        // raw / full = r_fixed / (r_ntc + r_fixed)
        const float r_ntc = p.r_fixed_ohm * (ADC_FULL - x) / x;

        const float inv_t = 1.0f / T25_K + logf(r_ntc / p.r25_ohm) / p.beta;
        return 1.0f / inv_t - K_TO_C;
    }
}
