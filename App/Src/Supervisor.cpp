#include "Supervisor.h"
#include "app.hpp"
#include "tx_api.h"
#include <cstdint>

static motor_control::MotorState currState;
static uint32_t t_state;        // mevcut state/adımın başladığı an [ms]
static void handle_commands();
static void run_align();    
static void run_open_loop();
static void run_closed_loop();


enum class AlignStep
{
    ALIGN_TO_90_DEG,
    ALIGN_TO_0_DEG,
    CALIB_FORWARD,
    CALIB_BACKWARD
};
static AlignStep align_step;

enum class OlStep
{
    RAMP,           // hız OMEGA_OL'e çıkar
    HOLD,           // SMO açıyı yakalasın
    IQ_DOWN         // iq'yu azalt, OL açısı rotora otursun
};
static OlStep ol_step;
static float  ol_delta;         // θ_ol − θ_smo [rad], Scrutiny'de izlemek için

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
        t_state   = HAL_GetTick();
        enter_state(motor_state);
        if (motor_state == MotorState::ALIGN)      align_step = AlignStep::ALIGN_TO_90_DEG;
        if (motor_state == MotorState::OPEN_LOOP)  ol_step    = OlStep::RAMP;
    }

    if (motor_state == MotorState::ALIGN)       run_align();
    if (motor_state == MotorState::OPEN_LOOP)   run_open_loop();
    if (motor_state == MotorState::CLOSED_LOOP) run_closed_loop();
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
            if (now - t_state >= ALIGN_MS)
            {
                foc_cfg.fixed_sincos = {0.0f, 1.0f};    // 0°
                t_state              = now;
                align_step           = AlignStep::ALIGN_TO_0_DEG;
            }
            break;

        case AlignStep::ALIGN_TO_0_DEG:
            if (now - t_state >= ALIGN_MS)
            {
                if (meas.hall_state != 5)         // θ=0 → wait Hall 5
                {
                    fault = FaultCode::SENSOR_ERROR;
                    return;
                }
                hall.startCalibration();
                open_loop.reset(0.0f);
                foc_cfg.ol_speed        = OMEGA_CAL;
                foc_cfg.record_hall_cal = true;
                set_foc_mode(AngleSource::OPEN_LOOP, LoopMode::CURRENT);
                align_step              = AlignStep::CALIB_FORWARD;
            }
            break;

        case AlignStep::CALIB_FORWARD:
            if (open_loop.get_travelled() >= cal_rad)
            {
                open_loop.clear_travelled();
                foc_cfg.ol_speed   = -OMEGA_CAL;
                align_step         = AlignStep::CALIB_BACKWARD;
            }
            break;

        case AlignStep::CALIB_BACKWARD:
            if (open_loop.get_travelled() >= cal_rad)
            {
                foc_cfg.ol_speed = 0.0f;

                HAL_NVIC_DisableIRQ(ADC3_IRQn);
                foc_cfg.record_hall_cal = false;     // finish'ten sonra ISR kayıt eklemesin
                const bool ok = hall.finishCalibration();
                HAL_NVIC_EnableIRQ(ADC3_IRQn);

                if (!ok)
                {
                    fault = FaultCode::SENSOR_ERROR;
                    return;
                }

                foc_cfg.idq_ref = {0.0f, 0.0f};
                set_foc_mode(AngleSource::HALL, LoopMode::CURRENT);
                motor_state_machine.event_handler(MotorEvent::ALIGNMENT_DONE);
            }
            break;
    }
}

static void run_open_loop()
{
    using namespace motor_control;

    const uint32_t now = HAL_GetTick();

    ol_delta = wrap_pi(meas.theta_ol - meas.theta_e_rad_smo);

    switch (ol_step)
    {
        case OlStep::RAMP:
            foc_cfg.ol_speed = std::min(foc_cfg.ol_speed + OL_RAMP * SUPERVISOR_DT, OMEGA_OL);
            if (foc_cfg.ol_speed >= OMEGA_OL)
            {
                t_state = now;
                ol_step = OlStep::HOLD;
            }
            break;

        case OlStep::HOLD:
            if (now - t_state >= OL_HOLD_MS)
                ol_step = OlStep::IQ_DOWN;
            break;

        case OlStep::IQ_DOWN:
        {
            // iq azaldıkça δ küçülür; δ≈0'da OL açısı rotorla çakışır, iq = yükün ihtiyacı.
            // δ=0 maksimum tork noktası, altına inersek senkron kaçar → payla geç.
            // IQ_OL_MIN'e inildiyse yük çok hafif: yine geç, kalan farkı angle_offset eritir.
            const float iq = foc_cfg.idq_ref.iq_ref;

            if (std::fabs(ol_delta) < DELTA_SWITCH || iq <= IQ_OL_MIN)
                motor_state_machine.event_handler(MotorEvent::CLOSED_LOOP_READY);
            else
                foc_cfg.idq_ref.iq_ref = std::max(iq - IQ_DOWN_STEP, IQ_OL_MIN);
            break;
        }
    }
}

static void run_closed_loop()
{
    // OL→SMO açı farkını yavaşça 0'a erit
    const float offset = foc_cfg.angle_offset;

    if (std::fabs(offset) <= OFFSET_STEP)
        foc_cfg.angle_offset = 0.0f;
    else
        foc_cfg.angle_offset = offset - std::copysign(OFFSET_STEP, offset);
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
    //foc_cfg.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
    //                                     -SPEED_REF_MAX_RPM, SPEED_REF_MAX_RPM);
}
