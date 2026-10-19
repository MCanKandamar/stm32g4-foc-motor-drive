#include "current_control.hpp"
#include "types.hpp"
#include <numbers>
#include <cmath>

namespace motor_control
{
    current_control::current_control(const PiConfig_& id_config, const PiConfig_& iq_config)
        : id_controller(id_config), iq_controller(iq_config)
    {
    }

    duties_t current_control::update(const Idq_ref_t& idq_ref, const Iabc_t& currents, const SinCos_t& sin_cos, float Vdc)
    {
        // Clarke and Park transforms
        idq_fdb = park(clarke(currents), sin_cos);

        const float sqrt3 = std::numbers::sqrt3_v<float>;

        float V_max = Vdc / sqrt3;
        id_controller.setLimits(-V_max, V_max);
        V_dq.Vd = id_controller.update(idq_ref.id_ref, idq_fdb.id_fdb);

        float V_max_q = std::sqrt(std::fmax(0.0f, V_max * V_max - V_dq.Vd * V_dq.Vd));
        iq_controller.setLimits(-V_max_q, V_max_q);
        V_dq.Vq = iq_controller.update(idq_ref.iq_ref, idq_fdb.iq_fdb);


        // Perform inverse Park and Clarke transforms to convert the d-q voltages back to the a-b-c frame
        Vabc_t v_abc = inverseClarke(inversePark(V_dq, sin_cos));

        // Calculate the duty cycles using SVPWM
        duties_t output = svpwm(v_abc, Vdc);

        return output;
    }

    void current_control::reset()
    {
        id_controller.reset();
        iq_controller.reset();
    }
}

