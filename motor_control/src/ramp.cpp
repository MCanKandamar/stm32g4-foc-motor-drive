#include "ramp.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control
{
    float Ramp::update(float target, float rate_up, float rate_down, float step_time)
    {
        const float delta = target - output;

        // Sıfırdan uzaklaşıyor mu? (output = 0 iken her yön uzaklaşmadır)
        const bool  away_from_zero = (delta > 0.0f) ? (output >= 0.0f) : (output <= 0.0f);
        const float max_step       = (away_from_zero ? rate_up : rate_down) * step_time;

        if (std::fabs(delta) <= max_step)
        {
            output = target;                    // tam eşitlik: "hedefe ulaşıldı" kontrolü için
        }
        else
        {
            const float next = output + std::clamp(delta, -max_step, max_step);

            // Yavaşlarken sıfırı geçme: 0'da dur, bir sonraki adım rate_up ile devam eder
            output = (!away_from_zero && next * output < 0.0f) ? 0.0f : next;
        }
        return output;
    }
}
