#pragma once
#include "types.hpp"



namespace motor_control
{
    constexpr float PI          = 3.14159265359f;
    constexpr float PI_DIV_3    = PI / 3.0f;
    constexpr float TWO_PI      = 2.0f * PI;
    constexpr float RAD_TO_RPM  = 60.0f / TWO_PI;
    constexpr float RAD_TO_DEG  = 180.0f / PI;

    // [0, 2π) aralığına sar (girdi en fazla bir tur dışarıda)
    inline float wrap_2pi(float x)
    {
        if      (x >= TWO_PI) x -= TWO_PI;
        else if (x <  0.0f)   x += TWO_PI;
        return x;
    }

    // [-π, π) aralığına sar (girdi en fazla bir tur dışarıda)
    inline float wrap_pi(float x)
    {
        if      (x >=  PI) x -= TWO_PI;
        else if (x <  -PI) x += TWO_PI;
        return x;
    }


    using SinCosFn = SinCos_t (*) (float theta);
    SinCos_t sincos_std(float theta);

}
