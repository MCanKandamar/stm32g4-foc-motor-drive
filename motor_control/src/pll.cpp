#include "pll.hpp"
#include "math_utils.hpp"
#include "types.hpp"
#include <cmath>

namespace motor_control
{   
    AnglePLL::AnglePLL(const PiConfig_& pll_config, SinCosFn sincos)
        : pll_controller(pll_config),
        step_time(pll_config.step_time),
        sincos_(sincos) {}

    float AnglePLL::update(float theta_measured)
    {   
        pll_angle = wrap_2pi(pll_angle + pll_speed * step_time);

        float error = wrap_pi(theta_measured - pll_angle);

        pll_speed = pll_controller.update(error, 0.0f);
        pll_speed_filtered = pll_controller.getIntegral();
        return pll_angle;
    }

    float AnglePLL::update_emf(const Z_alphabeta_t& z_alphabeta)
    {
        pll_angle = wrap_2pi(pll_angle + pll_speed * step_time);

        SinCos_t sc = sincos_(pll_angle);

        float err_raw = -z_alphabeta.z_alpha * sc.cos - z_alphabeta.z_beta * sc.sin;

        float E = sqrtf(z_alphabeta.z_alpha * z_alphabeta.z_alpha + z_alphabeta.z_beta * z_alphabeta.z_beta);
        if (E < 0.01f) E = 0.01f;
        float e_norm = err_raw / E;

        pll_speed = pll_controller.update(e_norm, 0.0f);
        pll_speed_filtered = pll_controller.getIntegral();

        return pll_angle;
    }

    void AnglePLL::reset(float theta, float omega)
    {
        pll_angle = theta;
        pll_speed = omega;
        pll_speed_filtered = omega;
        pll_controller.setIntegral(omega);
    }
}

