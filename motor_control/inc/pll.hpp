#pragma once

#include "types.hpp"
#include "pi_controller.hpp"

namespace motor_control
{

    class AnglePLL
    {
        public:
            AnglePLL(const PiConfig_& pll_config);

            float update(float theta_measured);

            float get_angle() const {return pll_angle;}
            float get_speed() const {return pll_speed;}

            void reset(float theta = 0.0f, float omega = 0.0f);

        private:
            PiController pll_controller;
            float pll_angle{0.0f};
            float pll_speed{0.0f};
            float step_time;
    };
}