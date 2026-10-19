#pragma once

namespace motor_control
{

    struct Iabc_t
    {
        float ia;
        float ib;
        float ic;
    };

    struct Vabc_t
    {
        float Va;
        float Vb;
        float Vc;
    };

    struct I_AlphaBeta_t
    {
        float ialpha;
        float ibeta;
    };

    struct V_AlphaBeta_t
    {
        float Valpha;
        float Vbeta;
    };

    struct Idq_fdb_t
    {
        float id_fdb;
        float iq_fdb;
    };

    struct Idq_ref_t
    {
        float id_ref;
        float iq_ref;
    };
    
    struct Vdq_t
    {
        float Vd;
        float Vq;
    };

    struct duties_t
    {
        float duty_a;
        float duty_b;
        float duty_c;
    };
    
    struct SinCos_t
    {
        float sin;
        float cos;
    };

    struct Hall_pin_t
    {
        int Hall_A;
        int Hall_B;
        int Hall_C;
    };
}