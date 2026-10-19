#include "svpwm.hpp"
#include "types.hpp"
#include <algorithm>


namespace motor_control
{
    duties_t svpwm(const Vabc_t& Vabc, float Vdc)
    {
        duties_t duties;

        if (Vdc <= 0.0f) {
            duties.duty_a = 0.0f;
            duties.duty_b = 0.0f;
            duties.duty_c = 0.0f;
            return duties;
        }

        float inverse_Vdc = 1.0f / Vdc;

        // Calculate the maximum and minimum duty cycles based on the input voltages
        float v_max = std::max({Vabc.Va, Vabc.Vb, Vabc.Vc});
        float v_min = std::min({Vabc.Va, Vabc.Vb, Vabc.Vc});

        // Calculate the duty cycles for each phase
        duties.duty_a = 0.5f + (Vabc.Va - (v_min + v_max) / 2.0f) * inverse_Vdc;
        duties.duty_b = 0.5f + (Vabc.Vb - (v_min + v_max) / 2.0f) * inverse_Vdc;
        duties.duty_c = 0.5f + (Vabc.Vc - (v_min + v_max) / 2.0f) * inverse_Vdc;

        // Clamp the duty cycles to the range [0, 1]
        duties.duty_a = std::clamp(duties.duty_a, 0.0f, 1.0f);
        duties.duty_b = std::clamp(duties.duty_b, 0.0f, 1.0f);
        duties.duty_c = std::clamp(duties.duty_c, 0.0f, 1.0f);

        return duties;
    }
}