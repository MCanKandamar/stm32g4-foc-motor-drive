#include "app.hpp"
#include "observer.hpp"
#include "filters.hpp"
#include "pll.hpp"
#include "regen_limit.hpp"
#include "scrutiny.h"
#include "transforms.hpp"
#include "types.hpp"
#include <cmath>

static motor_control::LowPassFilter vdc_filter = {VDC_LPF_ALPHA, 0.0f, false};

/* Interrupt Callbacks -------------------------------------------------------*/
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    using namespace motor_control;

    if (hadc->Instance == ADC3)
    {   
        meas.isr_count = meas.isr_count + 1U;
        offsetCalSample();                      // add the sample if an offset measurement is running
        meas.Iabc               = get_abc_currents();
        meas.Vabc               = get_abc_voltages();
        meas.theta_e_rad_hall   = get_motor_angle_hal();
        meas.theta_e_rad_pll    = pll.update(meas.theta_e_rad_hall);
        meas.i_alphabeta        = clarke(meas.Iabc);
        meas.z_alphabeta        = smo.update(meas.i_alphabeta, meas.v_alphabeta);
        meas.theta_e_rad_smo    = pll_smo.update_emf(meas.z_alphabeta);
        meas.speed_rpm_hall     = hall.getMechanicalSpeedRpm();
        meas.speed_rpm_hall_pll = pll.get_speed() / POLE_PAIRS * RAD_TO_RPM;
        meas.speed_rpm_smo      = pll_smo.get_speed() / POLE_PAIRS * RAD_TO_RPM;
        meas.vdc                = vdc_filter.update(get_vdc());
        meas.regen_scale        = regen_scale(meas.vdc, VDC_REGEN_START, VDC_REGEN_STOP);

        // Here and not in Monitor: braking can raise the bus by volts within a few ms
        if (meas.vdc > VDC_MAX && foc_cfg.loop != LoopMode::OFF)
        {
            powerOff();
            fault = FaultCode::OVER_VOLTAGE;
        }

        if (foc_cfg.loop == LoopMode::OFF)      // the Supervisor turned PWM off
        {
            ScrutinyFocTick();
            return;
        }

        /* Angle Source -----------------------------------------------*/
        float speed_fb_rpm = 0.0f;
        switch (foc_cfg.angle_src)
        {
            case AngleSource::FIXED:
                meas.SinCos = foc_cfg.fixed_sincos;
                break;
            case AngleSource::OPEN_LOOP:
                meas.theta_ol = open_loop.update(foc_cfg.ol_speed, TS);
                if (foc_cfg.record_hall_cal)
                    hall.recordCalibration(meas.theta_ol);
                meas.SinCos = cordicSinCos(meas.theta_ol);
                break;
            case AngleSource::HALL:
                meas.SinCos  = cordicSinCos(meas.theta_e_rad_pll);
                // not speed_rpm_hall: its 1-turn average lags T/2 (30 ms at 1000 rpm) and makes the speed loop oscillate
                speed_fb_rpm = meas.speed_rpm_hall_pll;
                break;
            case AngleSource::SMO:
                meas.SinCos  = cordicSinCos(wrap_2pi(meas.theta_e_rad_smo + foc_cfg.angle_offset));
                speed_fb_rpm = meas.speed_rpm_smo;
                break;
        }

        /* Loop Mode --------------------------------------------------*/
        if (foc_cfg.loop == LoopMode::SPEED)
            meas.idq_ref = get_idq_ref( foc_cfg.speed_ref_rpm,
                                        speed_fb_rpm,
                                        meas.regen_scale );
        else
            meas.idq_ref = foc_cfg.idq_ref;

        // Every ISR, also in CURRENT mode: the speed PI limits above change only at 1 kHz
        meas.idq_ref.iq_ref = limit_braking_iq(meas.idq_ref.iq_ref, speed_fb_rpm, meas.regen_scale);

        /* Calculate Dutiess ------------------------------------------*/
        meas.duties = foc.update(   meas.idq_ref,
                                    meas.Iabc,
                                    meas.SinCos,
                                    meas.vdc        );
        
        meas.v_alphabeta = foc.get_Valphabeta();
        update_duties(meas.duties);
    
        ScrutinyFocTick();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   // Driver nFault
    using namespace motor_control;
    if (GPIO_Pin == FAULT_N_Pin)
    {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);  // PWM off now; ENABLE stays high until drvFaultPoll() reads the cause
        fault = FaultCode::DRIVER_FAULT;
        drv_fault_pending = true;
    }
}