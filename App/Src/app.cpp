#include "app.hpp"
#include "hall_sensor.hpp"
#include "observer.hpp"
#include "pi_controller.hpp"
#include "pll.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "types.hpp"

namespace
{
    motor_control::PiConfig_ idq_pi_cfg(
    {
        .Kp            = 1.3f,
        .Ki            = 500.0f,
        .step_time     = TS,
        .out_max       = 5.0f,
        .out_min       = -5.0f,
    });

    // Kp [A/rpm], Ki [A/(rpm·s)]. Tuned on the board 2026-10-30 (tools/hil/speed_step.py):
    // crossover ~8 Hz, limited by the 30 Hz speed PLLs; 1000 → 1300 rpm step rises in ~20 ms.
    motor_control::PidConfig_ speed_pi_cfg(
    {
        .Kp            = 0.0025f,
        .Ki            = 0.02f,
        .Kd            = 0.0f,
        .step_time     = 0.001f,
        .out_max       = IQ_SPEED_MAX,
        .out_min       = -IQ_SPEED_MAX,
    });

    motor_control::PiConfig_ pll_pi_cfg(
    {
        .Kp             = 2.0f * 0.8f * (motor_control::TWO_PI * 30.0f),
        .Ki             = (motor_control::TWO_PI * 30.0f) * (motor_control::TWO_PI * 30.0f),
        .step_time      = TS,
        .out_max        =  5000.0f,  
        .out_min        = -5000.0f,
    });

    motor_control::PiConfig_ pll_smo_pi_cfg(
    {
        .Kp             = 2.0f * 0.8f * (motor_control::TWO_PI * 30.0f),
        .Ki             = (motor_control::TWO_PI * 30.0f) * (motor_control::TWO_PI * 30.0f),
        .step_time      = TS,
        .out_max        =  2000.0f,  
        .out_min        = -2000.0f,
    });

    // Chip reset values for now; the register values are chosen later.
    constexpr motor_control::Drv8350s::Config DRV_CONFIG{};

    // DRV8350S SDO is open-drain and the board has no pull-up resistor; the weak MCU pull-up
    // rises too slowly for the SPI peripheral (even at 664 kHz). So the SPI is done by software,
    // slow enough for the pull-up: 2.5 µs per half clock (200 kHz), one frame ≈ 85 µs.
    constexpr uint32_t DRV_HALF_CLOCK_CYCLES = 425;    // 2.5 µs at 170 MHz

    void waitCycles(uint32_t cycles)
    {
        const uint32_t start = DWT->CYCCNT;
        while (DWT->CYCCNT - start < cycles) {}
    }

    // SPI mode 1: the DRV sends a bit on the rising edge and reads SDI on the falling edge.
    // An interrupt in between only makes the clock slower, which the DRV accepts.
    uint16_t drvTransfer(uint16_t tx)
    {
        uint16_t rx = 0;
        HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
        waitCycles(DRV_HALF_CLOCK_CYCLES);
        for (int bit = 15; bit >= 0; --bit)
        {
            HAL_GPIO_WritePin(DRV_SDI_GPIO_Port, DRV_SDI_Pin, ((tx >> bit) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_SET);
            waitCycles(DRV_HALF_CLOCK_CYCLES);
            rx = static_cast<uint16_t>((rx << 1) | (HAL_GPIO_ReadPin(DRV_SDO_GPIO_Port, DRV_SDO_Pin) == GPIO_PIN_SET));
            HAL_GPIO_WritePin(DRV_SCLK_GPIO_Port, DRV_SCLK_Pin, GPIO_PIN_RESET);
            waitCycles(DRV_HALF_CLOCK_CYCLES);
        }
        HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
        waitCycles(DRV_HALF_CLOCK_CYCLES);              // CS high ≥ 400 ns before the next frame
        return rx;
    }

    motor_control::smo_config smo_cfg(
    {
        .phase_resistance_ohm = 0.20f,
        .phase_inductance     = 0.30e-3f,
        .sample_time          = TS,
        .k_gain               = 20.0f,
        .phi                  = 3.3f,
    });
}

motor_control::HallSensor hall(
{
    .f_cnt_hz = 1'000'000.0f,
    .pole_pairs = motor_control::BLF02.number_of_poles / 2,
    .edge_angle_rad = { 283.8f * DEG, 347.7f * DEG,  39.5f  * DEG,
                        99.4f  * DEG, 158.2f * DEG,  217.2f * DEG },
    .timeout_s = 0.05f,
});


motor_control::current_control    foc(idq_pi_cfg, idq_pi_cfg);
motor_control::speed_control      speed_ctrl(speed_pi_cfg);
motor_control::StateMachine       motor_state_machine;
motor_control::AnglePLL           pll(pll_pi_cfg);
motor_control::AnglePLL           pll_smo(pll_smo_pi_cfg, cordicSinCos);
motor_control::smo_observer       smo(smo_cfg);
motor_control::OpenLoopAngle      open_loop;

motor_control::Drv8350s           drv(drvTransfer);

volatile motor_control::FaultCode fault = motor_control::FaultCode::NONE;
volatile bool                     drv_fault_pending = false;
motor_control::Drv8350s::Status   drv_status;

Measurements meas;
FocConfig    foc_cfg;
Status       status;
AdcCalib     adc_cal;
AdcDma       adc_dma;
Commands     cmd;


/* Functions -------------------------------------------------------*/

// The only place that writes angle_src and loop, so the ISR cannot run between the two writes and see half a setting.
void set_foc_mode(motor_control::AngleSource angle_src, motor_control::LoopMode loop)
{
    HAL_NVIC_DisableIRQ(ADC3_IRQn);
    foc_cfg.angle_src = angle_src;
    foc_cfg.loop      = loop;
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
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
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!drv.configure(DRV_CONFIG))           // registers were reset while ENABLE was low
    {
        powerOff();
        fault = motor_control::FaultCode::DRIVER_FAULT;
        return;
    }
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

// After nFAULT: read the cause before ENABLE low resets the driver.
void drvFaultPoll()
{
    using motor_control::Drv8350s;

    if (!drv_fault_pending)
        return;
    drv_fault_pending = false;

    if (HAL_GPIO_ReadPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin) == GPIO_PIN_SET)   // still awake
    {
        drv_status = drv.read_status();
        if (drv_status.fault & Drv8350s::VDS_OCP)
            fault = motor_control::FaultCode::DRIVER_OVER_CURRENT;
        else if (drv_status.fault & Drv8350s::OTSD)
            fault = motor_control::FaultCode::DRIVER_OVER_TEMP;
    }
    powerOff();
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

motor_control::Idq_ref_t get_idq_ref(float omega_ref_rpm, float omega_fdbck_rpm, float regen_scale)
{
    using namespace motor_control;
    static uint8_t div = 0;
    if (++div >= 20)   // 1 kHz
    {
        div = 0;
        // Braking iq has the opposite sign of the speed: shrink only that side, so the integral cannot wind up
        const float brake_max = IQ_SPEED_MAX * regen_scale;
        if (omega_fdbck_rpm > 0.0f)
            speed_ctrl.setLimits(-brake_max, IQ_SPEED_MAX);
        else if (omega_fdbck_rpm < 0.0f)
            speed_ctrl.setLimits(-IQ_SPEED_MAX, brake_max);
        else
            speed_ctrl.setLimits(-IQ_SPEED_MAX, IQ_SPEED_MAX);
        speed_ctrl.update(omega_ref_rpm, omega_fdbck_rpm);
    }
    return speed_ctrl.get_output();     // last output between updates (the reset(iq_init) value after a switch)
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

