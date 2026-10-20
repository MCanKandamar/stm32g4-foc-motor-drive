#pragma once

namespace motor_control
{
    class OpenLoopAngle
    {
        public:
            float update(float omega, float step_time);

            void  reset(float theta0 = 0.0f);
            float get_travelled() const { return travelled; }
            void  clear_travelled();

        private:
            float theta{0.0f};
            volatile float travelled{0.0f};
    };
}
