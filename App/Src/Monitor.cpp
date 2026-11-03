
#include "Monitor.h"
#include "app.hpp"

static uint32_t t_led = 0;
static bool vdc_seen = false;

static bool iwdg_started = false;

volatile bool alive_supervisor  = false;
volatile bool alive_canopen     = false;
volatile bool alive_monitor     = false;

void MonitorRun(void)
{
    using namespace motor_control;

    /* Started here, not in main: start-up work must not eat the 100 ms timeout.
       Once started, only a reset stops it. */
    if (!iwdg_started)
    {
        MX_IWDG_Init();
        iwdg_started = true;
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

    uint32_t now = HAL_GetTick();
    if (now - t_led >= LED_PERIOD_MS)
    {
        t_led = now;
        HAL_GPIO_TogglePin(PROCCESS_LED_GPIO_Port, PROCCESS_LED_Pin);
    }
}