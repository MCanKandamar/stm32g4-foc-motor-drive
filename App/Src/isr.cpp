#include "app.hpp"
#include "observer.hpp"
#include "pll.hpp"
#include "scrutiny.h"
#include "transforms.hpp"
#include "types.hpp"
#include <cmath>

/* Interrupt Callbacks -------------------------------------------------------*/
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    using namespace motor_control;

    if (hadc->Instance == ADC3)
    {   
        offsetCalSample();                      // add the sample if an offset measurement is running
        meas.Iabc               = get_abc_currents();
        meas.Vabc               = get_abc_voltages();
        meas.theta_e_rad_hall   = get_motor_angle_hal();
        meas.theta_e_rad_pll    = pll.update(meas.theta_e_rad_hall);
        meas.i_alphabeta        = clarke(meas.Iabc);
        meas.z_alphabeta        = smo.update(meas.i_alphabeta, meas.v_alphabeta);
        meas.theta_e_rad_smo    = pll_smo.update_emf(meas.z_alphabeta);
        meas.speed_rpm_hall     = hall.getMechanicalSpeedRpm();
        meas.speed_rpm_hall_pll = pll.get_speed() / 4.0f * RAD_TO_RPM;
        meas.speed_rpm_smo      = pll_smo.get_speed() / 4.0f * RAD_TO_RPM;

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
                                        speed_fb_rpm );
        else
            meas.idq_ref = foc_cfg.idq_ref;

        /* Calculate Dutiess ------------------------------------------*/
        meas.duties = foc.update(   meas.idq_ref,
                                    meas.Iabc,
                                    meas.SinCos,
                                    status.Vdc      );
        
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
        powerOff();
        fault = FaultCode::DRIVER_FAULT;
    }
}