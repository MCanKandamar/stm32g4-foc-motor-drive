#pragma once

#include "transforms.hpp"
#include "types.hpp"

namespace motor_control
{

    duties_t svpwm(const Vabc_t& input, float Vdc);

}