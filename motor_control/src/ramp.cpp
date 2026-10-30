#include "ramp.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control
{
    float Ramp::update(float target, float rate_up, float rate_down, float step_time)
    {
        const float delta = target - output;

        // Moving away from zero? (when output = 0, any direction is away)
        const bool  away_from_zero = (delta > 0.0f) ? (output >= 0.0f) : (output <= 0.0f);
        const float max_step       = (away_from_zero ? rate_up : rate_down) * step_time;

        if (std::fabs(delta) <= max_step)
        {
            output = target;                    // exact match: for the "target reached" check
        }
        else
        {
            const float next = output + std::clamp(delta, -max_step, max_step);

            // When slowing down, do not cross zero: stop at 0, the next step goes on with rate_up
            output = (!away_from_zero && next * output < 0.0f) ? 0.0f : next;
        }
        return output;
    }
}
