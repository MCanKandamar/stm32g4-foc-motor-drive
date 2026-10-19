#include "app.hpp"
#include "types.hpp"
#include "filters.hpp"

void adcOffsetCalibration()
{
    uint32_t sum[3] = {0,0,0};
    for (int i = 0; i < 100; i++) {
        HAL_ADC_Start(&hadc3);
        for (int j = 0; j < 3; j++) {
            HAL_ADC_PollForConversion(&hadc3, 10);
            sum[j] += HAL_ADC_GetValue(&hadc3);
        }
    }
    HAL_ADC_Stop(&hadc3);
    for (int i = 0; i < 3; i++) {
        adc_cal.amplifier_offset_raw[i] = sum[i] / 100;
    } 
}

motor_control::Iabc_t get_abc_currents()
{   
    using namespace motor_control;
    /* Get Abc Currents ------------------------------------------*/ 
    uint16_t adc_raw[3];
    adc_raw[0] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
    adc_raw[1] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
    adc_raw[2] = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3);

    Iabc_t I_abc;
    I_abc.ia = (adc_raw[0] - adc_cal.amplifier_offset_raw[0]) * adc_cal.amp_per_lsb;
    I_abc.ib = (adc_raw[1] - adc_cal.amplifier_offset_raw[1]) * adc_cal.amp_per_lsb;
    I_abc.ic = (adc_raw[2] - adc_cal.amplifier_offset_raw[2]) * adc_cal.amp_per_lsb;

    // (1-d)*25us - 0.88us >= ~3us. AD8418 Fig.18
    constexpr float D_VALID_MAX = 0.84f;

    const duties_t& d = meas.duties;
    const bool va = d.duty_a <= D_VALID_MAX;
    const bool vb = d.duty_b <= D_VALID_MAX;
    const bool vc = d.duty_c <= D_VALID_MAX;

    switch (va + vb + vc)
    {
        case 3:                                
            break;
        case 2:                     
            if      (!va) I_abc.ia = -I_abc.ib - I_abc.ic;
            else if (!vb) I_abc.ib = -I_abc.ia - I_abc.ic;
            else          I_abc.ic = -I_abc.ia - I_abc.ib;
            break;
        default:                             
            return meas.Iabc;    
    }


    if (std::fabs(I_abc.ia) > I_MAX || std::fabs(I_abc.ib) > I_MAX || std::fabs(I_abc.ic) > I_MAX) {
        powerOff();
        fault = FaultCode::MOTOR_OVER_CURRENT;
    }

    constexpr float ALPHA = 0.7f;

    static LowPassFilter filter_a{ALPHA, 0.0f, false};
    static LowPassFilter filter_b{ALPHA, 0.0f, false};
    static LowPassFilter filter_c{ALPHA, 0.0f, false};

    I_abc.ia = filter_a.update(I_abc.ia);
    I_abc.ib = filter_b.update(I_abc.ib);
    I_abc.ic = filter_c.update(I_abc.ic);

    return I_abc;
}

motor_control::Vabc_t get_abc_voltages()
{
    using namespace motor_control;
    /* Get Abc Voltages ------------------------------------------*/ 
    Vabc_t V_abc;
    float* v_div = meas.Vabc_phase_divider_voltage;
    for (int i = 0; i < 3; i++)
    {
        v_div[i] = adc_dma.Vabc_phase_raw[i] * adc_cal.volt_per_lsb;
    }
    V_abc.Va = voltage_divider_get_voltage(v_div[0], 39.0f, 2.2f);
    V_abc.Vb = voltage_divider_get_voltage(v_div[1], 39.0f, 2.2f);
    V_abc.Vc = voltage_divider_get_voltage(v_div[2], 39.0f, 2.2f);

    return V_abc;
}

float voltage_divider_get_voltage(float voltage, float kohm_top, float kohm_bottom)
{
    return voltage * ((kohm_top + kohm_bottom) / kohm_bottom);
}
