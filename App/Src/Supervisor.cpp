#include "Supervisor.h"
#include "app.hpp"
#include "tx_api.h"
#include <cstdint>

using namespace motor_control;

static MotorState currState;
static uint32_t   t_state;      // mevcut state/adımın başladığı an [ms]

static void handle_commands();

// Her durum: enter = duruma girişte bir kez, run = durumda kaldıkça her 1 ms
static void enter_off();
static void enter_adc_cal();
static void run_adc_cal();
static void enter_align();
static void run_align();
static void enter_open_loop();
static void run_open_loop();
static void enter_closed_loop();
static void run_closed_loop();
static void enter_calibrating();
static void run_calibrating();


enum class AlignStep
{
    TO_90_DEG,
    TO_0_DEG
};
static AlignStep align_step;

enum class CalStep
{
    ALIGN,          // 90° → 0°
    FORWARD,        // 4 elektriksel tur ileri
    BACKWARD        // 4 elektriksel tur geri
};
static CalStep cal_step;

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
    handle_commands();

    if (fault != FaultCode::NONE && motor_state_machine.getState() != MotorState::FAULT)
        motor_state_machine.event_handler(MotorEvent::FAULT_DETECTED);

    const MotorState s = motor_state_machine.getState();

    // 1) Yeni duruma girildiyse: enter bir kez
    if (s != currState)
    {
        currState = s;
        t_state   = HAL_GetTick();

        switch (s)
        {
            case MotorState::IDLE:           enter_off();         break;
            case MotorState::ADC_OFFSET_CAL: enter_adc_cal();     break;
            case MotorState::ALIGN:          enter_align();       break;
            case MotorState::OPEN_LOOP:      enter_open_loop();   break;
            case MotorState::CLOSED_LOOP:    enter_closed_loop(); break;
            case MotorState::FAULT:          enter_off();         break;
            case MotorState::CALIBRATING:    enter_calibrating(); break;
        }
    }

    // 2) Her 1 ms: run
    switch (s)
    {
        case MotorState::ADC_OFFSET_CAL: run_adc_cal();     break;
        case MotorState::ALIGN:          run_align();       break;
        case MotorState::OPEN_LOOP:      run_open_loop();   break;
        case MotorState::CLOSED_LOOP:    run_closed_loop(); break;
        case MotorState::CALIBRATING:    run_calibrating(); break;
        default:                                            break;  // IDLE, FAULT: bekle
    }
}


/* Ortak yardımcılar --------------------------------------------------------*/

// Akım ofsetini ölç, ADC3 enjekte kesmesini (FOC ISR) başlat
static void start_current_sensing()
{
    adcOffsetCalibration();
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    __HAL_ADC_DISABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOS);
}

// Gücü aç, rotoru 90°'ye çekmeye başla. Devamı align_done().
static void start_align()
{
    foc_cfg.idq_ref      = {I_ALIGN, 0.0f};
    foc_cfg.fixed_sincos = {1.0f, 0.0f};        // 90°
    set_foc_mode(AngleSource::FIXED, LoopMode::CURRENT);
    powerOn();
    align_step = AlignStep::TO_90_DEG;
}

// 90°'de ALIGN_MS, sonra 0°'de ALIGN_MS bekler. Rotor 0°'ye oturunca true.
static bool align_done()
{
    const uint32_t now = HAL_GetTick();

    if (now - t_state < ALIGN_MS)
        return false;

    if (align_step == AlignStep::TO_90_DEG)
    {
        foc_cfg.fixed_sincos = {0.0f, 1.0f};    // 0°
        t_state              = now;
        align_step           = AlignStep::TO_0_DEG;
        return false;
    }
    return true;
}


/* IDLE / FAULT -------------------------------------------------------------*/

static void enter_off()
{
    powerOff();
    foc_cfg.record_hall_cal = false;
    set_foc_mode(AngleSource::FIXED, LoopMode::OFF);
}


/* ADC_OFFSET_CAL -----------------------------------------------------------*/

static void enter_adc_cal()
{
    start_current_sensing();
}

static void run_adc_cal()
{
    motor_state_machine.event_handler(MotorEvent::ADC_OFFSET_CAL_DONE);
}


/* ALIGN: rotoru 0°'ye oturt (OL bu açıdan başlar) --------------------------*/

static void enter_align()
{
    start_align();
}

static void run_align()
{
    if (align_done())
        motor_state_machine.event_handler(MotorEvent::ALIGNMENT_DONE);
}


/* OPEN_LOOP ----------------------------------------------------------------*/

static void enter_open_loop()
{
    open_loop.reset(3.0f * PI / 2.0f);      // -90°: q ekseni rotorun durduğu 0°'ye denk gelir, sıçrama olmaz
    foc_cfg.ol_speed = 0.0f;
    foc_cfg.idq_ref  = {0.0f, I_OL};
    pll_smo.reset();
    set_foc_mode(AngleSource::OPEN_LOOP, LoopMode::CURRENT);
    ol_step = OlStep::RAMP;
}

static void run_open_loop()
{
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


/* CLOSED_LOOP --------------------------------------------------------------*/

static void enter_closed_loop()
{
    speed_ctrl.reset(foc_cfg.idq_ref.iq_ref);   // PI, OL'de azaltılmış iq'dan başlasın
    HAL_NVIC_DisableIRQ(ADC3_IRQn);             // ofset ve mod aynı ISR örneğine göre
    foc_cfg.angle_offset = wrap_pi(meas.theta_ol - meas.theta_e_rad_smo);
    set_foc_mode(AngleSource::SMO, LoopMode::SPEED);   // IRQ'yu tekrar açar
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


/* CALIBRATING: Hall kenar açılarını ölç, bitince IDLE ----------------------*/

static void enter_calibrating()
{
    start_current_sensing();
    start_align();
    cal_step = CalStep::ALIGN;
}

static void run_calibrating()
{
    const float cal_rad = CAL_TURNS * TWO_PI;

    switch (cal_step)
    {
        case CalStep::ALIGN:
            if (!align_done())
                break;

            if (meas.hall_state != 5)               // θ=0 → Hall 5 beklenir
            {
                fault = FaultCode::SENSOR_ERROR;
                return;
            }
            hall.startCalibration();
            open_loop.reset(0.0f);
            foc_cfg.ol_speed        = OMEGA_CAL;
            foc_cfg.record_hall_cal = true;
            set_foc_mode(AngleSource::OPEN_LOOP, LoopMode::CURRENT);
            cal_step                = CalStep::FORWARD;
            break;

        case CalStep::FORWARD:
            if (open_loop.get_travelled() >= cal_rad)
            {
                open_loop.clear_travelled();
                foc_cfg.ol_speed = -OMEGA_CAL;
                cal_step         = CalStep::BACKWARD;
            }
            break;

        case CalStep::BACKWARD:
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
                motor_state_machine.event_handler(MotorEvent::CALIBRATION_DONE);
            }
            break;
    }
}


/* Komutlar -----------------------------------------------------------------*/

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
    if (cmd.start)
    {
        cmd.start = false;
        motor_state_machine.event_handler(MotorEvent::START);
    }
    if (cmd.calibrate)
    {
        cmd.calibrate = false;
        motor_state_machine.event_handler(MotorEvent::CALIBRATE);
    }
    //foc_cfg.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
    //                                     -SPEED_REF_MAX_RPM, SPEED_REF_MAX_RPM);
}
