#include "Supervisor.h"
#include "app.hpp"
#include "tx_api.h"
#include <cstdint>

static motor_control::MotorState currState;
static void handle_commands();
static void run_align();
static void run_open_loop();


enum class AlignStep { ALIGN_TO_90_DEG, ALIGN_TO_0_DEG, CALIB_FORWARD, CALIB_BACKWARD };
static AlignStep align_step;

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
    hall.init(meas.hall_state);
}

void SupervisorRun(void)
{
    using namespace motor_control;
    
    handle_commands();

    if (fault != FaultCode::NONE)
    {
        motor_state_machine.event_handler(MotorEvent::FAULT_DETECTED);
    }

    MotorState motor_state = motor_state_machine.getState();

    if (motor_state != currState)
    {
        currState = motor_state;
        enter_state(motor_state);
        if (motor_state == MotorState::ALIGN)
        {
            align_step = AlignStep::ALIGN_TO_90_DEG;
        }
    }

    if (motor_state == MotorState::ALIGN)       run_align();
    if (motor_state == MotorState::OPEN_LOOP)   run_open_loop();
}


/* Functions ----------------------------------------------------------------*/

static void run_align()
{
    using namespace motor_control;

    const uint32_t now     = HAL_GetTick();
    const float    cal_rad = CAL_TURNS * TWO_PI;

    switch (align_step)
    {
        case AlignStep::ALIGN_TO_90_DEG:
            if (now - setpoints.t_state >= ALIGN_MS)
            {
                setpoints.align_sincos = {0.0f, 1.0f};    // 0°
                setpoints.t_state      = now;
                align_step             = AlignStep::ALIGN_TO_0_DEG;
            }
            break;

        case AlignStep::ALIGN_TO_0_DEG:
            if (now - setpoints.t_state >= ALIGN_MS)
            {
                if (meas.hall_state != 5)         // θ=0 → wait Hall 5
                {
                    fault = FaultCode::SENSOR_ERROR;
                    return;
                }
                hall.startCalibration();
                open_loop.reset(0.0f);
                setpoints.ol_speed  = OMEGA_CAL;
                setpoints.ctrl_mode = CtrlMode::HALL_SENSOR_CAL;
                align_step          = AlignStep::CALIB_FORWARD;
            }
            break;

        case AlignStep::CALIB_FORWARD:
            if (open_loop.get_travelled() >= cal_rad)
            {
                open_loop.clear_travelled();
                setpoints.ol_speed = -OMEGA_CAL;
                align_step         = AlignStep::CALIB_BACKWARD;
            }
            break;

        case AlignStep::CALIB_BACKWARD:
            if (open_loop.get_travelled() >= cal_rad)
            {
                setpoints.ol_speed = 0.0f;

                HAL_NVIC_DisableIRQ(ADC3_IRQn);
                const bool ok = hall.finishCalibration();
                HAL_NVIC_EnableIRQ(ADC3_IRQn);

                if (!ok)
                {
                    fault = FaultCode::SENSOR_ERROR;
                    return;
                }

                setpoints.idq_ref   = {0.0f, 0.0f};
                setpoints.ctrl_mode = CtrlMode::CURRENT_CTRL;
                motor_state_machine.event_handler(MotorEvent::ALIGNMENT_DONE);
            }
            break;
    }
}


static void run_open_loop()
{
    using namespace motor_control;

    const uint32_t now = HAL_GetTick();

    setpoints.ol_speed = std::min(setpoints.ol_speed + OL_RAMP * SUPERVISOR_DT, OMEGA_OL);

    if (setpoints.ol_speed < OMEGA_OL)
        setpoints.t_state = now;  

    if (now - setpoints.t_state >= OL_HOLD_MS)
        motor_state_machine.event_handler(MotorEvent::CLOSED_LOOP_READY);
}



static void handle_commands()
{
    using namespace motor_control;

    if (cmd.stop)
    {
        cmd.stop = false;
        motor_state_machine.event_handler(MotorEvent::STOP);
    }
    if (cmd.reset_fault)
    { 
        cmd.reset_fault = false; fault = FaultCode::NONE;
        motor_state_machine.event_handler(MotorEvent::RESET_FAULT);
    }
    if (cmd.start)
    {
        cmd.start = false;
        motor_state_machine.event_handler(MotorEvent::START);
    }
    //setpoints.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
    //                                     -SPEED_REF_MAX_RPM, SPEED_REF_MAX_RPM);
}
