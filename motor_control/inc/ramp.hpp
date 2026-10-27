#pragma once

namespace motor_control
{
    // Hedefe sabit eğimle yaklaşan referans (CiA 402 profile velocity / torque rampası).
    // rate_up: |çıkış| büyürken, rate_down: |çıkış| küçülürken [birim/s], ikisi de > 0.
    // Sıfırı geçerken önce rate_down ile 0'a iner, sonra rate_up ile devam eder.
    class Ramp
    {
        public:
            float update(float target, float rate_up, float rate_down, float step_time);

            void  reset(float value = 0.0f) { output = value; }
            float get_output() const { return output; }

        private:
            float output{0.0f};
    };
}
