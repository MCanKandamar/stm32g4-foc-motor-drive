#pragma once

namespace motor_control
{
    // Reference that moves to the target at a fixed slope (CiA 402 profile velocity / torque ramp).
    // rate_up: while |output| grows, rate_down: while |output| shrinks [unit/s], both > 0.
    // When crossing zero, it first goes to 0 with rate_down, then goes on with rate_up.
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
