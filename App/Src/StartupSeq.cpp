#include "StartupSeq.hpp"
#include "app.hpp"
#include <cstdint>

using namespace motor_control;

static uint32_t t_state;        // start time of the current step [ms]

struct RunMode
{
    SensorMode sensor;
    LoopMode   loop;
};
static RunMode run_mode;        // copied from cmd at START, fixed while running
static uint32_t t_sl_ok;        // last time the SMO speed was above SL_LOST_RPM [ms]

enum class AlignStep
{
    TO_90_DEG,
    TO_0_DEG
};
static AlignStep align_step;

enum class CalStep
{
    ALIGN,          // 90° → 0°
    FORWARD,        // 4 electrical turns forward
    BACKWARD        // 4 electrical turns backward
};
static CalStep cal_step;

enum class OlStep
{
    RAMP,           // speed rises to OMEGA_OL
    HOLD,           // let the SMO catch the angle
    IQ_DOWN         // lower iq so the OL angle settles on the rotor
};
static OlStep ol_step;
static float  ol_delta;         // θ_ol − θ_smo [rad], to watch in Scrutiny


/* Common helpers -----------------------------------------------------------*/

// Turn power on, start pulling the rotor to 90°. align_done() does the rest.
static void start_align()
{
    foc_cfg.idq_ref      = {I_ALIGN, 0.0f};
    foc_cfg.fixed_sincos = {1.0f, 0.0f};        // 90°
    set_foc_mode(AngleSource::FIXED, LoopMode::CURRENT);
    t_state    = HAL_GetTick();
    powerOn();
    align_step = AlignStep::TO_90_DEG;
}

// Waits ALIGN_MS at 90°, then ALIGN_MS at 0°. Returns true when the rotor sits at 0°.
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


/* START: fix the mode, choose the next state after IDLE --------------------*/

MotorEvent prepare_start()
{
    run_mode = { cmd.sensor, cmd.loop };            // fix at START
    if (run_mode.loop == LoopMode::OFF)
        run_mode.loop = LoopMode::SPEED;            // OFF is not valid → SPEED

    return (run_mode.sensor == SensorMode::HALL) ? MotorEvent::START_HALL           // → CLOSED_LOOP
                                                 : MotorEvent::START_SENSORLESS;    // → ALIGN
}


/* ALIGN: set the rotor to 0° (OL starts from this angle) -------------------*/

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
    open_loop.reset(3.0f * PI / 2.0f);      // -90°: the q axis lines up with the rotor at 0°, so there is no jump
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
            // As iq drops, δ gets smaller; at δ≈0 the OL angle matches the rotor and iq = what the load needs.
            // δ=0 is the maximum torque point; going below it loses sync → switch with some margin.
            // If iq reached IQ_OL_MIN the load is very light: switch anyway, angle_offset removes the rest.
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

    if (src == AngleSource::HALL)                   // straight from IDLE: start safely with iq = 0
    {
        foc_cfg.idq_ref = {0.0f, 0.0f};
        set_foc_mode(AngleSource::HALL, LoopMode::CURRENT);
        powerOn();
    }

    speed_ctrl.reset(foc_cfg.idq_ref.iq_ref);   // start the PI from the present iq (lowered in OL, 0 with Hall)
    HAL_NVIC_DisableIRQ(ADC3_IRQn);             // offset and mode must match the same ISR sample
    foc_cfg.angle_offset = (src == AngleSource::SMO)
                         ? wrap_pi(meas.theta_ol - meas.theta_e_rad_smo)   // coming from OL
                         : 0.0f;                                           // no offset with Hall
    set_foc_mode(src, run_mode.loop);           // enables the IRQ again
    t_sl_ok = HAL_GetTick();
}

void run_closed_loop()
{
    // slowly fade the OL→SMO angle difference to 0
    const float offset = foc_cfg.angle_offset;

    if (std::fabs(offset) <= OFFSET_STEP)
        foc_cfg.angle_offset = 0.0f;
    else
        foc_cfg.angle_offset = offset - std::copysign(OFFSET_STEP, offset);

    // Sensorless: at low speed the EMF is small, so the SMO angle is not reliable.
    // 1) ref is at least SL_MIN_RPM (OL starts forward only, no direction change)
    // 2) SMO speed below SL_LOST_RPM (or reversed) for SL_LOST_MS → fault, power is cut
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

    // User reference (cmd) → foc_cfg
    if (run_mode.loop == LoopMode::SPEED)
        foc_cfg.speed_ref_rpm = std::clamp(static_cast<float>(cmd.speed_ref_rpm),
                                           ref_min, SPEED_REF_MAX_RPM);
    else
        foc_cfg.idq_ref = {0.0f, std::clamp(static_cast<float>(cmd.iq_ref),
                                            -IQ_REF_MAX, IQ_REF_MAX)};
}


/* CALIBRATING: measure the Hall edge angles, then IDLE ---------------------*/

void enter_calibrating()
{
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

            if (meas.hall_state != 5)               // θ=0 → expect Hall 5
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
                foc_cfg.record_hall_cal = false;     // after finish, the ISR must not add records
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
