#include "speed_control.hpp"
#include "pi_controller.hpp"
#include "types.hpp"

namespace motor_control
{
    speed_control::speed_control(const PidConfig_& speed_config)
        : SpeedController(speed_config)
    {
    }

    Idq_ref_t speed_control::update(float speed_reference, float speed_feedback)
    {
        idq_ref.iq_ref = SpeedController.update(speed_reference, speed_feedback);
        idq_ref.id_ref = 0;

        return idq_ref;
    }

    void speed_control::reset(float iq_init)
    {
        SpeedController.reset();
        SpeedController.setIntegral(iq_init);
        idq_ref = {0.0f, iq_init};
    }
}