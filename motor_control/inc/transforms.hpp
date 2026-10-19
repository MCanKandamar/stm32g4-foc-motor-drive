#pragma once
#include "types.hpp"

namespace motor_control
{
    I_AlphaBeta_t clarke(const Iabc_t& input);
    Vabc_t inverseClarke(const V_AlphaBeta_t& input);
    Idq_fdb_t park(const I_AlphaBeta_t& input, SinCos_t sin_cos);
    V_AlphaBeta_t inversePark(const Vdq_t& input, SinCos_t sin_cos);
}