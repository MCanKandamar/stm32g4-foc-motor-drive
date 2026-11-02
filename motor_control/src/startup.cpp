#include "startup.hpp"
#include "math_utils.hpp"
#include <cmath>

namespace motor_control
{
    float OpenLoopAngle::update(float omega, float step_time)
    {
        theta = wrap_2pi(theta + omega * step_time);

        travelled += std::fabs(omega) * step_time;

        return theta;
    }

    void OpenLoopAngle::reset(float theta0)
    {
        theta     = theta0;
        travelled = 0.0f;
    }

    void OpenLoopAngle::clear_travelled()
    {
        travelled = 0.0f;
    }

}
