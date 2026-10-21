#include "math_utils.hpp"
#include <cmath>

namespace motor_control
{
    SinCos_t sincos_std(float theta)
    {
        return {sinf(theta), cosf(theta)};
    }
}