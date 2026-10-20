#pragma once

#include "types.hpp"

namespace motor_control
{
    struct smo_config
    {
        float phase_resistance_ohm;           // Phase resistance [Ω]
        float phase_inductance;               // phase inductance [H]

        float sample_time;

        float k_gain;                         // switching gain k > E_max

        float phi;                            // boundary layer width
    };

    struct Z_alphabeta_t        // correction terms
    {
        float z_alpha;
        float z_beta;
    };

    struct I_AlphaBeta_hat_t    // estimate
    {
        float i_alpha_hat;
        float i_beta_hat;
    };

    struct I_AlphaBeta_error_t
    {
        float i_alpha_error;
        float i_beta_error;
    };

    class smo_observer
    {
        public:
            smo_observer(const smo_config& cfg);

            Z_alphabeta_t update(const I_AlphaBeta_t& i_alphabeta, const V_AlphaBeta_t& v_alphabeta);

            I_AlphaBeta_error_t get_ialphabeta_error() const {return i_alphabeta_error;}

        private:
            smo_config config;

            const float F_smo; 
            const float G_smo;
            const float inv_phi;

            I_AlphaBeta_t i_alphabeta_hat{0.0f, 0.0f};
            Z_alphabeta_t z_alphabeta{0.0f, 0.0f};
            I_AlphaBeta_error_t i_alphabeta_error;

            float saturation_function(float error, float inv_phi);
    };
}