#include "Supervisor.h"
#include "app.hpp"
#include "tx_api.h"

static motor_control::MotorState currState;
static void handle_commands();

void SupervisorInit(void)
{   

    HAL_TIMEx_HallSensor_Start(&htim3);

    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_dma.Vabc_phase_raw, 3);
    __HAL_DMA_DISABLE_IT(&hdma_adc4, DMA_IT_TC | DMA_IT_HT);

    HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc_dma.adc_vdc_vrefint_raw, 2);
    __HAL_DMA_DISABLE_IT(&hdma_adc5, DMA_IT_TC | DMA_IT_HT);

    tx_thread_sleep(2); 
    
    adc_cal.adc_ref_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_dma.adc_vdc_vrefint_raw[1], ADC_RESOLUTION_12B) / 1000.0f;
    adc_cal.volt_per_lsb    = adc_cal.adc_ref_voltage / ((1U << ADC_BIT) - 1U);
    adc_cal.amp_per_lsb     = adc_cal.volt_per_lsb / SHUNT_RES_OHM / SHUNT_AMP_GAIN;

    DRV_ENABLE_RESET();
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    meas.hall_state = readHallPins();
    hall.HallSensorInit(meas.hall_state);
}

void SupervisorRun(void)
{
    using namespace motor_control;
    
    handle_commands();

    if (fault != FaultCode::NONE) motor_state.event_handler(MotorEvent::FAULT_DETECTED);

    MotorState s = motor_state.getState();

    if (s != currState) {
        currState = s; enter_state(s);
    }

    if (s == MotorState::ALIGN && HAL_GetTick() - setpoints.t_state >= ALIGN_MS / 2)
        setpoints.align_sc = {0.0f, 1.0f};

    if (s == MotorState::ALIGN && HAL_GetTick() - setpoints.t_state >= ALIGN_MS)
    {
        if (meas.hall_state != 5) fault = FaultCode::SENSOR_ERROR;   // θ=0 → Hall 5 bekleniyor
        setpoints.idq_ref   = {0.0f, 0.0f};
        setpoints.ctrl_mode = CtrlMode::CURRENT_CTRL;
        motor_state.event_handler(MotorEvent::ALIGNMENT_DONE);
    }
}



static void handle_commands()
{
    using namespace motor_control;

    if (cmd.stop)
    {
        cmd.stop = false;
        motor_state.event_handler(MotorEvent::STOP);
    }
    if (cmd.reset_fault)
    { 
        cmd.reset_fault = false; fault = FaultCode::NONE;
        motor_state.event_handler(MotorEvent::RESET_FAULT);
    }
    if (cmd.start)
    {
        cmd.start = false;
        motor_state.event_handler(MotorEvent::START);
    }
    //setpoints.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
    //                                     -SPEED_REF_MAX_RPM, SPEED_REF_MAX_RPM);
}
