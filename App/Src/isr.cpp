#include "app.hpp"
#include "observer.hpp"
#include "pll.hpp"
#include "state_machine.hpp"
#include "scrutiny.h"
#include "transforms.hpp"
#include "types.hpp"
#include <cmath>

/* Interrupt callbacks -------------------------------------------------------*/
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    using namespace motor_control;

    if (hadc->Instance == ADC3)
    {   
        meas.Iabc               = get_abc_currents();
        meas.Vabc               = get_abc_voltages();
        meas.theta_e_rad_hall   = get_motor_angle_hal();
        meas.theta_e_rad_pll    = pll.update(meas.theta_e_rad_hall);
        meas.i_alphabeta = clarke(meas.Iabc);

        Z_alphabeta_t z_alphabeta = smo.update(meas.i_alphabeta, meas.v_alphabeta);
        meas.theta_e_rad_smo = pll_smo.update_emf(z_alphabeta);

        meas.speed_rpm_hall = hall.getMechanicalSpeedRpm();
        meas.speed_rpm_smo  = pll_smo.get_speed() / 4.0f * motor_control::RAD_TO_RPM;

        if (setpoints.ctrl_mode == CtrlMode::SPEED_CTRL)
            meas.idq_ref = get_idq_ref(setpoints.speed_ref_rpm,
                                       meas.speed_rpm_hall);
        else
            meas.idq_ref = setpoints.idq_ref;
        
        
        
        switch (setpoints.ctrl_mode)
        {
            case CtrlMode::ALIGNMENT:
                meas.SinCos = setpoints.align_sincos;
                break;
            case CtrlMode::OPEN_LOOP_CTRL:
                setpoints.ctrl_mode = CtrlMode::CURRENT_CTRL;
            case CtrlMode::CURRENT_CTRL:
                meas.SinCos = cordicSinCos(meas.theta_e_rad_pll);
                break;
            case CtrlMode::SPEED_CTRL:
                meas.SinCos = cordicSinCos(meas.theta_e_rad_pll);
                break;
            default:
                powerOff();
                return;
        }

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