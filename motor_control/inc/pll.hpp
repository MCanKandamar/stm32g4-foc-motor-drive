#pragma once

#include "types.hpp"
#include "pi_controller.hpp"
#include "observer.hpp"

namespace motor_control
{
    class AnglePLL
    {
        public:
            AnglePLL(const PiConfig_& pll_config);

            float update(float theta_measured);
            float update_emf(const Z_alphabeta_t& z_alphabeta);
            float get_angle() const {return pll_angle;}
            float get_speed() const {return pll_speed_filtered;}

            void reset(float theta = 0.0f, float omega = 0.0f);

        private:
            PiController pll_controller;
            float pll_angle{0.0f};
            float pll_speed{0.0f};
            float pll_speed_filtered{0.0f};
            float step_time;
    };
}