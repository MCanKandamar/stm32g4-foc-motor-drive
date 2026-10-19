#pragma once

#include "transforms.hpp"
#include "svpwm.hpp"
#include "pi_controller.hpp"
#include "types.hpp"

namespace motor_control
{

    class current_control
    {
        private:
            PiController id_controller;
            PiController iq_controller;
            Idq_fdb_t idq_fdb;
            Vdq_t V_dq;

        public:
            current_control(const PiConfig_& id_config, const PiConfig_& iq_config);

            duties_t update(const Idq_ref_t& idq_ref, const Iabc_t& currents, const SinCos_t& sin_cos, float Vdc);

            Idq_fdb_t get_idq_feedback() const {return idq_fdb; }
            Vdq_t get_Vdq() const {return V_dq;}

            void reset();
    };
}