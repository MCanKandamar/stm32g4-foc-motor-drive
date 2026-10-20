#include "observer.hpp"

namespace motor_control
{
    smo_observer::smo_observer(const smo_config& cfg) 
    : config(cfg),
    F_smo(1 - (cfg.phase_resistance_ohm / cfg.phase_inductance) * cfg.sample_time),
    G_smo(cfg.sample_time / cfg.phase_inductance),
    inv_phi(1 / cfg.phi)
    {
    }

    Z_alphabeta_t smo_observer::update(const I_AlphaBeta_t& i_alphabeta, const V_AlphaBeta_t& v_alphabeta)
    {
        i_alphabeta_error.i_alpha_error = i_alphabeta_hat.ialpha - i_alphabeta.ialpha;
        i_alphabeta_error.i_beta_error  = i_alphabeta_hat.ibeta - i_alphabeta.ibeta;

        z_alphabeta.z_alpha = config.k_gain * saturation_function(i_alphabeta_error.i_alpha_error, inv_phi);
        z_alphabeta.z_beta  = config.k_gain * saturation_function(i_alphabeta_error.i_beta_error, inv_phi);

        i_alphabeta_hat.ialpha = F_smo * i_alphabeta_hat.ialpha + G_smo * (v_alphabeta.Valpha - z_alphabeta.z_alpha);
        i_alphabeta_hat.ibeta = F_smo * i_alphabeta_hat.ibeta + G_smo * (v_alphabeta.Vbeta - z_alphabeta.z_beta);
        
        return z_alphabeta;
    }
    
    float smo_observer::saturation_function(float error, float inv_phi)
    {
        float val = error * inv_phi; 
        if (val >= 1.0f)         return 1.0f;
        else if (val <= -1.0f)   return -1.0f;
        return val;
    }
}