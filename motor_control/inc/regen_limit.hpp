#pragma once

namespace motor_control
{
    // Braking pushes energy back into the DC bus. A lab supply cannot take it, so the bus rises.
    // Share of the braking current allowed at this bus voltage:
    // 1 at or below v_start, 0 at or above v_stop, a straight line between.
    float regen_scale(float vdc, float v_start, float v_stop);

    // iq brakes when it works against the speed (iq · speed < 0): scale it.
    // Driving iq and iq at zero speed pass unchanged.
    float limit_braking_iq(float iq, float speed, float scale);
}
