
#include "Monitor.h"
#include "app.hpp"

static uint32_t t_led = 0;
static bool vdc_seen = false;

static bool iwdg_started = false;

static uint32_t t_rate = 0;
static uint32_t isr_count_last = 0;

volatile bool alive_supervisor  = false;
volatile bool alive_canopen     = false;
volatile bool alive_monitor     = false;

volatile bool reset_by_iwdg     = false;    // the last reset came from the watchdog (read in Scrutiny)

void MonitorRun(void)
{
    using namespace motor_control;

    /* Started here, not in main: start-up work must not eat the 100 ms timeout.
       Once started, only a reset stops it. */
    if (!iwdg_started)
    {
        reset_by_iwdg = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0U;
        __HAL_RCC_CLEAR_RESET_FLAGS();      // so the next reset shows only its own cause
        MX_IWDG_Init();
        iwdg_started = true;

        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADC_Start(&hadc1);              // first board temperature conversion
    }

    /* Kick only if every watched task ran since the last kick */
    if (alive_supervisor && alive_canopen && alive_monitor)
    {
        HAL_IWDG_Refresh(&hiwdg);
        alive_supervisor = false;
        alive_canopen    = false;
        alive_monitor    = false;
    }

    status.dir         = hall.getDirection();
    status.idq_fdb     = foc.get_idq_feedback();

    status.Vdc = voltage_divider_get_voltage(  (adc_cal.volt_per_lsb * adc_dma.adc_vdc_vrefint_raw[0]),
                                               39.0f,
                                               2.2f    );
                                        
    // The MCU can start before the DC supply: stay quiet until the supply is seen once.
    // A start without the supply still faults, because the motor is no longer waiting.
    if (status.Vdc >= VDC_READY)
        vdc_seen = true;

    const MotorState state = motor_state_machine.getState();
    const bool waiting = !vdc_seen && (state == MotorState::INIT || state == MotorState::IDLE);

    if (status.Vdc > VDC_MAX) {
        powerOff();
        fault = FaultCode::OVER_VOLTAGE;
    }
    else if (status.Vdc < VDC_MIN && !waiting) {
        powerOff();
        fault = FaultCode::UNDER_VOLTAGE;
    }

    // Board temperature: read the last conversion, then start the next one (no waiting)
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
    {
        status.pcb_temp_c = ntc_temp_c(static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1)), PCB_NTC);
        if (status.pcb_temp_c > PCB_TEMP_MAX) {
            powerOff();
            fault = FaultCode::DRIVER_OVER_TEMP;
        }
        HAL_ADC_Start(&hadc1);
    }

    uint32_t now = HAL_GetTick();

    // Lost or extra FOC calls make every ISR speed wrong (e.g. at -O0 every 2nd ADC3 interrupt is lost)
    if (now - t_rate >= FOC_RATE_CHECK_MS)
    {
        const uint32_t count = meas.isr_count;
        status.foc_isr_hz = static_cast<float>(count - isr_count_last) * 1000.0f / static_cast<float>(now - t_rate);
        if (isr_count_last != 0U && std::fabs(status.foc_isr_hz * TS - 1.0f) > FOC_RATE_TOL)   // 0: the ISR has not started yet
        {
            powerOff();
            fault = FaultCode::FOC_RATE_ERROR;
        }
        isr_count_last = count;
        t_rate         = now;
    }

    if (now - t_led >= LED_PERIOD_MS)
    {
        t_led = now;
        HAL_GPIO_TogglePin(PROCCESS_LED_GPIO_Port, PROCCESS_LED_Pin);
    }
}