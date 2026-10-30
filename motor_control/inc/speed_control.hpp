#pragma once
#include "pi_controller.hpp"
#include "types.hpp"

namespace motor_control
{
    
    class speed_control
    {
        private:
            PidController SpeedController;
            float speed_feedback{0.0f};  // Current speed feedback value
            float speed_reference{0.0f};  // Current speed reference value

            Idq_ref_t idq_ref;

        public:
            speed_control(const PidConfig_& speed_config);

            Idq_ref_t update(float speed_reference, float speed_feedback);

            Idq_ref_t get_output() const { return idq_ref; }   // output of the last update()

            void reset(float iq_init = 0.0f);   // iq_init: for a smooth start after a switch
    };
}

