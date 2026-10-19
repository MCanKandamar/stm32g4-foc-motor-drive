#include "transforms.hpp"
#include "types.hpp"
#include <numbers>

namespace motor_control
{
    I_AlphaBeta_t clarke(const Iabc_t& iabc)
    {
        I_AlphaBeta_t I_AlphaBeta;
        I_AlphaBeta.ialpha = ((2.0f * iabc.ia) - iabc.ib - iabc.ic) / 3.0f;
        I_AlphaBeta.ibeta = (iabc.ib - iabc.ic) / std::numbers::sqrt3_v<float>;
        return I_AlphaBeta;
    }

    Vabc_t inverseClarke(const V_AlphaBeta_t& I_AlphaBeta)
    {
        Vabc_t Vabc;
        Vabc.Va = I_AlphaBeta.Valpha;
        Vabc.Vb = -0.5f * I_AlphaBeta.Valpha + std::numbers::sqrt3_v<float> / 2.0f * I_AlphaBeta.Vbeta;
        Vabc.Vc = -0.5f * I_AlphaBeta.Valpha - std::numbers::sqrt3_v<float> / 2.0f * I_AlphaBeta.Vbeta;
        return Vabc;
    }

    Idq_fdb_t park(const I_AlphaBeta_t& I_AlphaBeta, SinCos_t sin_cos)
    {
        Idq_fdb_t idq_fdb;
        idq_fdb.id_fdb = I_AlphaBeta.ialpha * sin_cos.cos + I_AlphaBeta.ibeta * sin_cos.sin;
        idq_fdb.iq_fdb = -I_AlphaBeta.ialpha * sin_cos.sin + I_AlphaBeta.ibeta * sin_cos.cos;
        return idq_fdb;
    }

    V_AlphaBeta_t inversePark(const Vdq_t& input, SinCos_t sin_cos)
    {
        V_AlphaBeta_t V_AlphaBeta;
        V_AlphaBeta.Valpha = input.Vd * sin_cos.cos - input.Vq * sin_cos.sin;
        V_AlphaBeta.Vbeta = input.Vd * sin_cos.sin + input.Vq * sin_cos.cos;
        return V_AlphaBeta;
    }
}