#include "StartupSeq.hpp"
#include "app.hpp"
#include <cstdint>

using namespace motor_control;

static uint32_t t_state;        // mevcut adımın başladığı an [ms]

struct RunMode
{
    SensorMode sensor;
    LoopMode   loop;
};
static RunMode run_mode;        // START anında cmd'den kopyalanır, çalışırken değişmez
static uint32_t t_sl_ok;        // SMO hızının en son SL_LOST_RPM üstünde görüldüğü an [ms]

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
    t_state    = HAL_GetTick();
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


/* ADC_OFFSET_CAL -----------------------------------------------------------*/

void enter_adc_cal()
{
    run_mode = { cmd.sensor, cmd.loop };            // START anında sabitle
    if (run_mode.loop == LoopMode::OFF)
        run_mode.loop = LoopMode::SPEED;            // OFF geçersiz → SPEED

    start_current_sensing();
}

void run_adc_cal()
{
    if (run_mode.sensor == SensorMode::HALL)
    {
        foc_cfg.idq_ref = {0.0f, 0.0f};             // iq = 0 ile güvenli başla
        set_foc_mode(AngleSource::HALL, LoopMode::CURRENT);
        powerOn();
        motor_state_machine.event_handler(MotorEvent::HALL_READY);           // → CLOSED_LOOP
    }
    else
        motor_state_machine.event_handler(MotorEvent::ADC_OFFSET_CAL_DONE);  // → ALIGN
}


/* ALIGN: rotoru 0°'ye oturt (OL bu açıdan başlar) --------------------------*/

void enter_align()
{
    start_align();
}

void run_align()
{
    if (align_done())
        motor_state_machine.event_handler(MotorEvent::ALIGNMENT_DONE);
}


/* OPEN_LOOP ----------------------------------------------------------------*/

void enter_open_loop()
{
    open_loop.reset(3.0f * PI / 2.0f);      // -90°: q ekseni rotorun durduğu 0°'ye denk gelir, sıçrama olmaz
    foc_cfg.ol_speed = 0.0f;
    foc_cfg.idq_ref  = {0.0f, I_OL};
    pll_smo.reset();
    set_foc_mode(AngleSource::OPEN_LOOP, LoopMode::CURRENT);
    ol_step = OlStep::RAMP;
}

void run_open_loop()
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

void enter_closed_loop()
{
    const AngleSource src = (run_mode.sensor == SensorMode::HALL) ? AngleSource::HALL
                                                                  : AngleSource::SMO;

    speed_ctrl.reset(foc_cfg.idq_ref.iq_ref);   // PI, o anki iq'dan başlasın (OL'de azaltılmış, Hall'da 0)
    HAL_NVIC_DisableIRQ(ADC3_IRQn);             // ofset ve mod aynı ISR örneğine göre
    foc_cfg.angle_offset = (src == AngleSource::SMO)
                         ? wrap_pi(meas.theta_ol - meas.theta_e_rad_smo)   // OL'den geliyoruz
                         : 0.0f;                                           // Hall'da ofset yok
    set_foc_mode(src, run_mode.loop);           // IRQ'yu tekrar açar
    t_sl_ok = HAL_GetTick();
}

void run_closed_loop()
{
    // OL→SMO açı farkını yavaşça 0'a erit
    const float offset = foc_cfg.angle_offset;

    if (std::fabs(offset) <= OFFSET_STEP)
        foc_cfg.angle_offset = 0.0f;
    else
        foc_cfg.angle_offset = offset - std::copysign(OFFSET_STEP, offset);

    // Sensörsüz: düşük hızda EMF küçük, SMO açısı güvenilmez.
    // 1) ref en az SL_MIN_RPM (OL sadece ileri kalkar, yön değişimi yok)
    // 2) SMO hızı SL_LOST_MS boyunca SL_LOST_RPM altında (ya da ters) → hata, güç kesilir
    float ref_min = -SPEED_REF_MAX_RPM;

    if (run_mode.sensor == SensorMode::SENSORLESS)
    {
        ref_min = SL_MIN_RPM;

        const uint32_t now = HAL_GetTick();
        if (meas.speed_rpm_smo >= SL_LOST_RPM)
            t_sl_ok = now;
        else if (now - t_sl_ok >= SL_LOST_MS)
            fault = FaultCode::SENSORLESS_LOST;
    }

    // Kullanıcı referansı (cmd) → foc_cfg
    if (run_mode.loop == LoopMode::SPEED)
        foc_cfg.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
                                           ref_min, SPEED_REF_MAX_RPM);
    else
        foc_cfg.idq_ref = {0.0f, std::clamp(static_cast<float>(cmd.iq_ref),
                                            -IQ_REF_MAX, IQ_REF_MAX)};
}


/* CALIBRATING: Hall kenar açılarını ölç, bitince IDLE ----------------------*/

void enter_calibrating()
{
    start_current_sensing();
    start_align();
    cal_step = CalStep::ALIGN;
}

void run_calibrating()
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
