#include "regen_limit.hpp"
#include <algorithm>

namespace motor_control
{
    float regen_scale(float vdc, float v_start, float v_stop)
    {
        return std::clamp((v_stop - vdc) / (v_stop - v_start), 0.0f, 1.0f);
    }

    float limit_braking_iq(float iq, float speed, float scale)
    {
        return (iq * speed < 0.0f) ? iq * scale : iq;
    }
}
