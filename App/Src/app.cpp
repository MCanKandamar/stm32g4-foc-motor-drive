#include "app.hpp"
#include "hall_sensor.hpp"
#include "observer.hpp"
#include "pi_controller.hpp"
#include "pll.hpp"
#include "tx_api.h"
#include "types.hpp"

namespace
{
    motor_control::PiConfig_ idq_pi_cfg(
    {
        .Kp            = 1.3f,
        .Ki            = 500.0f,
        .step_time     = (1.0f / 20'000.0f),
        .out_max       = 5.0f,
        .out_min       = -5.0f,
    });

    motor_control::PidConfig_ speed_pi_cfg(
    {
        .Kp            = 0.001f,
        .Ki            = 0.0015f,
        .Kd            = 0.0f,
        .step_time     = 0.001f,
        .out_max       = 9.0f,
        .out_min       = -9.0f,
    });

    motor_control::PiConfig_ pll_pi_cfg(
    {
        .Kp             = 2.0f * 0.8f * (motor_control::TWO_PI * 30.0f),
        .Ki             = (motor_control::TWO_PI * 30.0f) * (motor_control::TWO_PI * 30.0f),
        .step_time      = 1.0f / 20'000.0f,
        .out_max        =  5000.0f,  
        .out_min        = -5000.0f,
    });

    motor_control::PiConfig_ pll_smo_pi_cfg(
    {
        .Kp             = 2.0f * 0.8f * (motor_control::TWO_PI * 30.0f),
        .Ki             = (motor_control::TWO_PI * 30.0f) * (motor_control::TWO_PI * 30.0f),
        .step_time      = 1.0f / 20'000.0f,
        .out_max        =  2000.0f,  
        .out_min        = -2000.0f,
    });

    motor_control::HallSensor hall(
    {
        .f_cnt_hz = 1'000'000.0f,
        .pole_pairs = 4,
        .edge_angle_rad = { 283.8f * DEG, 347.7f * DEG,  39.5f  * DEG,
                            99.4f  * DEG, 158.2f * DEG,  217.2f * DEG },
        .timeout_s = 0.05f,
    });

    motor_control::smo_config smo_cfg(
    {
        .phase_resistance_ohm = 0.20f,
        .phase_inductance     = 0.30e-3f,
        .sample_time          = 1.0f / 20'000.0f,
        .k_gain               = 20.0f,
        .phi                  = 3.3f,
    });
}



motor_control::current_control    foc(idq_pi_cfg, idq_pi_cfg);
motor_control::speed_control      speed_ctrl(speed_pi_cfg);
motor_control::StateMachine       motor_state_machine;
motor_control::AnglePLL           pll(pll_pi_cfg);
motor_control::AnglePLL           pll_smo(pll_smo_pi_cfg);
motor_control::smo_observer       smo(smo_cfg);
motor_control::OpenLoopAngle      open_loop;

volatile motor_control::FaultCode fault = motor_control::FaultCode::NONE;

Measurements meas;
Setpoints    setpoints;
Status       status;
AdcCalib     adc_cal;
AdcDma       adc_dma;
Commands     cmd;


/* Functions -------------------------------------------------------*/

void enter_state(motor_control::MotorState state)
{   
    using namespace motor_control;
    switch(state)
    {
        case MotorState::IDLE:      
            powerOff();
            break;

        case MotorState::ADC_OFFSET_CAL:
            adcOffsetCalibration();
            HAL_ADCEx_InjectedStart_IT(&hadc3);
            __HAL_ADC_DISABLE_IT(&hadc3, ADC_IT_JEOC);
            __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOS);
            motor_state_machine.event_handler(MotorEvent::ADC_OFFSET_CAL_DONE);
            break;

        case MotorState::ALIGN:
            setpoints.idq_ref   = {I_ALIGN, 0.0f};
            setpoints.align_sincos  = {1.0f, 0.0f};
            setpoints.ctrl_mode = CtrlMode::ALIGNMENT;
            setpoints.t_state   = HAL_GetTick();
            powerOn();
            break;

        case MotorState::OPEN_LOOP:
            open_loop.reset(3.0f * PI / 2.0f);      // -90°: q ekseni rotorun durduğu 0°'ye denk gelir, sıçrama olmaz
            setpoints.ol_speed  = 0.0f;
            setpoints.idq_ref   = {0.0f, I_OL};
            pll_smo.reset();
            setpoints.ctrl_mode = CtrlMode::OPEN_LOOP_CTRL;
            setpoints.t_state   = HAL_GetTick();
            break;

        case MotorState::CLOSED_LOOP:
            speed_ctrl.reset();
            pll_smo.reset();
            setpoints.ctrl_mode = CtrlMode::SPEED_CTRL;
            powerOn();
            break;

        case MotorState::FAULT:
            powerOff();
            break;
    }
}

void DRV_ENABLE_SET()
{
    HAL_GPIO_WritePin(  DRV_ENABLE_GPIO_Port,
                        DRV_ENABLE_Pin,
                        GPIO_PIN_SET    );
}

void DRV_ENABLE_RESET()
{
    HAL_GPIO_WritePin(  DRV_ENABLE_GPIO_Port,
                        DRV_ENABLE_Pin,
                        GPIO_PIN_RESET   );
}

void powerOn()
{
    foc.reset();
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);  
    DRV_ENABLE_SET();
    tx_thread_sleep(2);  
    __HAL_GPIO_EXTI_CLEAR_IT(FAULT_N_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    if (HAL_GPIO_ReadPin(FAULT_N_GPIO_Port, FAULT_N_Pin) == GPIO_PIN_RESET)
    {                                          
        powerOff();
        fault = motor_control::FaultCode::DRIVER_FAULT;
        return;
    }                      
    __HAL_TIM_MOE_ENABLE(&htim8);
}

void powerOff()
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    DRV_ENABLE_RESET();
}

uint8_t readHallPins()
{
    uint8_t pinA = HAL_GPIO_ReadPin(HAL_A_GPIO_Port, HAL_A_Pin);
    uint8_t pinB = HAL_GPIO_ReadPin(HAL_B_GPIO_Port, HAL_B_Pin);
    uint8_t pinC = HAL_GPIO_ReadPin(HAL_C_GPIO_Port, HAL_C_Pin);

    return (pinA << 0U) | (pinB << 1U) | (pinC << 2U);
}

float get_motor_angle_hal()
{   
    using namespace motor_control;
    static uint8_t last_read = 0;
    const bool edge = (TIM3->SR & TIM_SR_CC1IF) != 0; 
    if (edge)
    {
        TIM3->SR = ~TIM_SR_CC1IF;
    }
    uint8_t now_read = readHallPins();
    
    if (edge || now_read == last_read)
        meas.hall_state = now_read;
    last_read = now_read;

    hall.update(meas.hall_state, TIM3->CCR1, TIM3->CNT);
    return hall.getElectricalAngleRad();
}

motor_control::Idq_ref_t get_idq_ref(float omega_ref_rpm, float omega_fdbck_rpm)
{
    using namespace motor_control;
    static uint8_t div = 0;
    static Idq_ref_t idq_reference = {0, 0};
    if (setpoints.ctrl_mode == CtrlMode::SPEED_CTRL && ++div >= 20)   // 1 kHz
    {
        div = 0;
        idq_reference = speed_ctrl.update(
            omega_ref_rpm,
            omega_fdbck_rpm);
    }
    return idq_reference;
}

void update_duties(motor_control::duties_t duties)
{
    /* Update PWM duty cycless ------------------------------------*/ 
    TIM8->CCR1 = static_cast<uint32_t>(duties.duty_a * TIM8->ARR);
    TIM8->CCR2 = static_cast<uint32_t>(duties.duty_b * TIM8->ARR);
    TIM8->CCR3 = static_cast<uint32_t>(duties.duty_c * TIM8->ARR);
}

motor_control::SinCos_t cordicSinCos(float theta)
{
    constexpr float RAD_TO_Q31      = static_cast<float>(1U << 31) / motor_control::PI;
    constexpr float Q31_TO_FLOAT    = 1.0f / static_cast<float>(1U << 31);

    if (theta >= motor_control::PI)     theta -= motor_control::TWO_PI;

    CORDIC->WDATA = static_cast<int32_t>(theta * RAD_TO_Q31);

    int32_t sin = static_cast<int32_t>(CORDIC->RDATA);
    int32_t cos = static_cast<int32_t>(CORDIC->RDATA);

    motor_control::SinCos_t sc;
    sc.sin  = sin * Q31_TO_FLOAT;
    sc.cos  = cos * Q31_TO_FLOAT;
    return  sc;
}

