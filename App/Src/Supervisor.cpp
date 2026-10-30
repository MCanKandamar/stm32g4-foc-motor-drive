#include "Supervisor.h"
#include "StartupSeq.hpp"
#include "app.hpp"
#include "tx_api.h"
#include <cstdint>

using namespace motor_control;

static MotorState currState;

static void handle_commands();
static void enter_init();
static void run_init();
static void init_peripherals();
static void enter_off();


void SupervisorInit(void)
{
    // Power-up: enter INIT by hand. currState is also INIT (0), so SupervisorRun does not see it as a new state.
    enter_init();
    currState = MotorState::INIT;
}

void SupervisorRun(void)
{
    handle_commands();
    offsetCalPoll();

    if (fault != FaultCode::NONE && motor_state_machine.getState() != MotorState::FAULT)
        motor_state_machine.event_handler(MotorEvent::FAULT_DETECTED);

    const MotorState s = motor_state_machine.getState();

    // 1) On a new state: run enter once
    if (s != currState)
    {
        currState = s;

        switch (s)
        {
            case MotorState::INIT:           enter_init();        break;
            case MotorState::IDLE:           enter_off();         break;
            case MotorState::ALIGN:          enter_align();       break;
            case MotorState::OPEN_LOOP:      enter_open_loop();   break;
            case MotorState::CLOSED_LOOP:    enter_closed_loop(); break;
            case MotorState::FAULT:          enter_off();         break;
            case MotorState::CALIBRATING:    enter_calibrating(); break;
        }
    }

    // 2) Every 1 ms: run
    switch (s)
    {
        case MotorState::INIT:           run_init();        break;
        case MotorState::ALIGN:          run_align();       break;
        case MotorState::OPEN_LOOP:      run_open_loop();   break;
        case MotorState::CLOSED_LOOP:    run_closed_loop(); break;
        case MotorState::CALIBRATING:    run_calibrating(); break;
        default:                                            break;  // IDLE, FAULT: wait
    }
}


/* INIT: device setup at power-up, then current offset ----------------------*/

static void enter_init()
{
    static bool hw_ready = false;       // peripherals only once (a fault reset can go back to INIT)

    if (!hw_ready)
    {
        init_peripherals();
        hw_ready = true;
    }

    adc_cal.offset_valid = false;
    offsetCalStart();                   // power stage off, ~6.4 ms
}

static void run_init()
{
    if (adc_cal.offset_valid)
        motor_state_machine.event_handler(MotorEvent::INIT_DONE);
}

static void init_peripherals()
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

    // FOC ISR: injected conversions triggered by TIM8. Runs with LoopMode::OFF in IDLE and also measures the offset.
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    __HAL_ADC_DISABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOS);
}


/* IDLE / FAULT -------------------------------------------------------------*/

static void enter_off()
{
    powerOff();
    foc_cfg.record_hall_cal = false;
    set_foc_mode(AngleSource::FIXED, LoopMode::OFF);
}


/* Commands -----------------------------------------------------------------*/

static void handle_commands()
{
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
    if (cmd.offset_cal)
    {
        cmd.offset_cal = false;
        if (motor_state_machine.getState() == MotorState::IDLE)
            offsetCalStart();                   // power stage off, current is zero
    }

    // While the offset refreshes, start / calibrate wait: the flag stays and is handled when it is done
    if (offsetCalBusy())
        return;

    if (cmd.start)
    {
        cmd.start = false;
        if (motor_state_machine.getState() == MotorState::IDLE)
            motor_state_machine.event_handler(prepare_start());     // START_HALL / START_SENSORLESS
    }
    if (cmd.calibrate)
    {
        cmd.calibrate = false;
        motor_state_machine.event_handler(MotorEvent::CALIBRATE);
    }
}
