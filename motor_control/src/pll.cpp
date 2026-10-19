#include "pll.hpp"
#include "hall_sensor.hpp"

namespace motor_control
{   
    AnglePLL::AnglePLL(const PiConfig_& pll_config)
        : pll_controller(pll_config), step_time(pll_config.step_time) {}

    float AnglePLL::update(float theta_measured)
    {
        using namespace motor_control;
        
        pll_angle += pll_speed * step_time;

        if      (pll_angle >= TWO_PI) pll_angle -= TWO_PI;
        else if (pll_angle <  0.0f)   pll_angle += TWO_PI;

        float error = theta_measured - pll_angle;
        if      (error >  PI) error -= TWO_PI;
        else if (error < -PI) error += TWO_PI;

        pll_speed = pll_controller.update(error, 0.0f);
        return pll_angle;
    }

    void AnglePLL::reset(float theta, float omega)
    {
        pll_angle = theta;
        pll_speed = omega;
        pll_controller.setIntegral(omega);
    }

}

