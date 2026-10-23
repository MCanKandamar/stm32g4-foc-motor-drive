
#include "Monitor.h"
#include "app.hpp"

static uint32_t t_led = 0;

void MonitorInit(void)
{
    
}

void MonitorRun(void)
{
    using namespace motor_control;

    status.theta_deg   = hall.getElectricalAngleRad() * RAD_TO_DEG;
    status.rpm         = hall.getMechanicalSpeedRpm();
    status.dir         = hall.getDirection();
    status.idq_fdb     = foc.get_idq_feedback();

    status.Vdc = voltage_divider_get_voltage(  (adc_cal.volt_per_lsb * adc_dma.adc_vdc_vrefint_raw[0]),
                                               39.0f,
                                               2.2f    );
                                        
    if (status.Vdc > VDC_MAX) {
        powerOff();
        fault = FaultCode::OVER_VOLTAGE;
    }
    else if (status.Vdc < VDC_MIN) {
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