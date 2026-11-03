#include "canopen_bridge.h"
#include "app.hpp"
#include "cia402.hpp"
#include "ramp.hpp"

extern "C" {
#include "CANopen.h"
#include "301/CO_ODinterface.h"
#include "OD.h"

extern CO_t* CO;                    // CO_app_STM32.c
}

using namespace motor_control;

/* Constants -----------------------------------------------------------------*/

// EMCY bit that marks which fault is active: manufacturer "critical" range (0x40..);
// it sets bit 7 (manufacturer) and bit 0 (generic) in error register 0x1001.
static constexpr uint8_t  EM_FAULT_BIT_BASE = 0x40U;

static constexpr int8_t   MODE_PV           = 3;        // 0x6060: profile velocity → LoopMode::SPEED
static constexpr int8_t   MODE_TQ           = 4;        // 0x6060: profile torque   → LoopMode::CURRENT, always Hall
static constexpr uint8_t  SENSOR_MODE_HALL  = 1U;       // 0x2012: 0 = sensorless, 1 = Hall (pv only)
static constexpr int16_t  ABORT_OPTION_MAX  = 3;        // 0x6007: 0 none, 1 fault, 2 disable voltage, 3 quick stop
static constexpr uint8_t  COMMAND_STOP      = 2U;       // 0x2011
static constexpr uint8_t  COMMAND_CALIBRATE = 4U;       // 0x2011

static constexpr uint16_t SW_TARGET_REACHED = 0x0400U;  // statusword bit 10
static constexpr uint16_t SW_SPEED_ZERO     = 0x1000U;  // statusword bit 12, pv only
static constexpr uint16_t SW_REMOTE         = 0x0200U;  // statusword bit 9: the controlword is obeyed
static constexpr uint16_t CW_DISABLE        = 0x0000U;  // Disable Voltage

static constexpr float    BRIDGE_DT         = 0.001f;   // CanopenBridgeUpdate period [s]
static constexpr float    VEL_WINDOW_RPM    = 30.0f;    // pv bit 10: |actual − target| below this
static constexpr float    VEL_ZERO_RPM      = 10.0f;    // pv bit 12: speed counts as zero

/* State ---------------------------------------------------------------------*/

static Cia402 cia402;                   // the logic is in motor_control/cia402

static Ramp   vel_ramp;                 // [rpm]
static Ramp   trq_ramp;                 // [‰ of 0x6076]
static float  vel_target = 0.0f;        // limited 0x60FF [rpm]
static float  trq_target = 0.0f;        // limited 0x6071 [‰]

static OD_extension_t mode_guard_ext;   // filled by CanopenBridgeInit
static OD_extension_t sensor_guard_ext;
static OD_extension_t abort_guard_ext;

/* Small helpers -------------------------------------------------------------*/

static bool is_torque_mode()
{
    return OD_RAM.x6060_modesOfOperation == MODE_TQ;
}

static bool is_operation_enabled()
{
    return cia402.getState() == Cia402State::OPERATION_ENABLED;
}

static bool can_owns_motor()
{
    return cmd.control_source == ControlSource::CANOPEN;
}

// ‰ (of 0x6076) ↔ iq [A]: T = iq · kt. 1000 ‰ = 400 mNm = 6.06 A
static float permille_per_amp()
{
    const float rated_nm = static_cast<float>(OD_RAM.x6076_motorRatedTorque) / 1000.0f;
    return BLF02.torque_constant / rated_nm * 1000.0f;
}

static float limit_speed(int32_t rpm)
{
    return std::clamp(static_cast<float>(rpm), -SPEED_REF_MAX_RPM, SPEED_REF_MAX_RPM);
}

// The SMO speed is valid only while the SMO gives the angle; stopped or in start-up it can be
// far off (e.g. −3800 rpm at standstill). Hall speed goes to 0 after the Hall timeout.
static float actual_speed_rpm()
{
    return (foc_cfg.angle_src == AngleSource::SMO) ? meas.speed_rpm_smo : meas.speed_rpm_hall;
}

static bool comm_lost()
{
    return (CO != nullptr) && (CO_NMT_getInternalState(CO->NMT) != CO_NMT_OPERATIONAL);
}

/* OD write checks (SDO and RPDO) --------------------------------------------*/

// 0x6060: the mode is fixed at START; a different value in Operation Enabled gives SDO abort 0x08000022.
static ODR_t write_mode_guard(OD_stream_t* stream, const void* buf, OD_size_t count, OD_size_t* countWritten)
{
    if (count != 1U)
        return ODR_TYPE_MISMATCH;

    const int8_t mode = *static_cast<const int8_t*>(buf);

    if (mode != MODE_PV && mode != MODE_TQ)
        return ODR_INVALID_VALUE;
    if (is_operation_enabled() && mode != OD_RAM.x6060_modesOfOperation)
        return ODR_DATA_DEV_STATE;

    return OD_writeOriginal(stream, buf, count, countWritten);
}

// 0x2012: same rule as 0x6060.
static ODR_t write_sensor_guard(OD_stream_t* stream, const void* buf, OD_size_t count, OD_size_t* countWritten)
{
    if (count != 1U)
        return ODR_TYPE_MISMATCH;

    const uint8_t sensor = *static_cast<const uint8_t*>(buf);

    if (sensor > SENSOR_MODE_HALL)
        return ODR_INVALID_VALUE;
    if (is_operation_enabled() && sensor != OD_RAM.x2012_sensorMode)
        return ODR_DATA_DEV_STATE;

    return OD_writeOriginal(stream, buf, count, countWritten);
}

// 0x6007: only 0..3 are defined.
static ODR_t write_abort_guard(OD_stream_t* stream, const void* buf, OD_size_t count, OD_size_t* countWritten)
{
    if (count != 2U)
        return ODR_TYPE_MISMATCH;

    const int16_t option = static_cast<int16_t>(CO_getUint16(buf));

    if (option < 0 || option > ABORT_OPTION_MAX)
        return ODR_INVALID_VALUE;

    return OD_writeOriginal(stream, buf, count, countWritten);
}

static void add_write_guard(OD_entry_t* entry, OD_extension_t& ext, decltype(OD_extension_t::write) write)
{
    ext.read  = OD_readOriginal;
    ext.write = write;
    OD_extension_init(entry, &ext);
}

/* Motor → OD ----------------------------------------------------------------*/

static void write_motor_data(MotorState ms, float speed_rpm)
{
    // with PWM off the ISR skips foc.update(), so iq feedback would keep its last value
    const float iq_a = (foc_cfg.loop != LoopMode::OFF) ? status.idq_fdb.iq_fdb : 0.0f;

    OD_RAM.x2000_vdcMV      = static_cast<uint16_t>(lroundf(status.Vdc * 1000.0f));   // 24.1 V → 24100
    OD_RAM.x2001_speedRpm   = static_cast<int16_t>(lroundf(speed_rpm));
    OD_RAM.x2002_motorState = static_cast<uint8_t>(ms);
    OD_RAM.x2003_faultCode  = static_cast<uint8_t>(fault);

    OD_RAM.x6061_modesOfOperationDisplay = OD_RAM.x6060_modesOfOperation;      // does not change in OE
    OD_RAM.x606C_velocityActualValue     = static_cast<int32_t>(lroundf(speed_rpm));
    OD_RAM.x6077_torqueActualValue       = static_cast<int16_t>(lroundf(iq_a * permille_per_amp()));
}

/* Targets: OD → ramp → cmd --------------------------------------------------*/

// 0x2010: speed ref without a ramp. When written, the ramp jumps to it and 0x60FF gets the same value
// (so the ramp does not pull it back). The last write of 0x2010 or 0x60FF wins.
static void apply_speed_ref_write()
{
    static int32_t last_speed_ref = 0;

    if (OD_RAM.x2010_speedRefRpm == last_speed_ref)
        return;

    last_speed_ref              = OD_RAM.x2010_speedRefRpm;
    OD_RAM.x60FF_targetVelocity = last_speed_ref;
    vel_ramp.reset(limit_speed(last_speed_ref));
}

static void update_targets()
{
    apply_speed_ref_write();

    const float trq_max = IQ_REF_MAX * permille_per_amp();     // 5 A ≈ ‰825

    vel_target = limit_speed(OD_RAM.x60FF_targetVelocity);
    cmd.decel_rpm_s = std::max(static_cast<float>(OD_RAM.x6084_profileDeceleration), 1.0f);
    trq_target = std::clamp(static_cast<float>(OD_RAM.x6071_targetTorque), -trq_max, trq_max);
}

static void update_torque_ramp(MotorState ms)
{
    if (ms == MotorState::CLOSED_LOOP)
    {
        const float slope = std::max(static_cast<float>(OD_RAM.x6087_torqueSlope), 1.0f);
        trq_ramp.update(trq_target, slope, slope, BRIDGE_DT);
    }
    else
    {
        trq_ramp.reset();                       // stay at 0 until start-up is done
    }

    cmd.iq_ref = trq_ramp.get_output() / permille_per_amp();
}

static void update_speed_ramp(MotorState ms, float speed_rpm)
{
    if (ms == MotorState::CLOSED_LOOP)
    {
        const float acc = std::max(static_cast<float>(OD_RAM.x6083_profileAcceleration), 1.0f);
        const float dec = std::max(static_cast<float>(OD_RAM.x6084_profileDeceleration), 1.0f);
        vel_ramp.update(vel_target, acc, dec, BRIDGE_DT);
    }
    else
    {
        vel_ramp.reset(speed_rpm);              // in a sensorless start-up (ALIGN, OL) start the ramp from the actual speed
    }

    cmd.speed_ref_rpm = vel_ramp.get_output();
}

// Only in Operation Enabled; otherwise the ramps are 0.
static void update_ramps(MotorState ms, float speed_rpm)
{
    if (!is_operation_enabled())
    {
        vel_ramp.reset();
        trq_ramp.reset();
    }
    else if (is_torque_mode())
    {
        update_torque_ramp(ms);
    }
    else
    {
        update_speed_ramp(ms, speed_rpm);
    }
}

/* Commands: OD → cmd --------------------------------------------------------*/

// 0x2011. Start and fault reset come only from the CiA 402 controlword. Stop is always allowed.
static void handle_command_object()
{
    switch (OD_RAM.x2011_command)
    {
        case COMMAND_STOP:      cmd.stop      = true; break;
        case COMMAND_CALIBRATE: if (can_owns_motor()) cmd.calibrate = true; break;
        default:                                      break;
    }
    OD_RAM.x2011_command = 0;
}

// A new owner must not inherit a running motor.
static void handle_owner_change()
{
    static ControlSource last_source = ControlSource::CANOPEN;
    const ControlSource  source      = cmd.control_source;

    if (source != last_source)
    {
        cmd.start = false;
        cmd.stop  = true;
    }
    last_source = source;
}

// Write the mode to cmd before START; the Supervisor fixes it at START.
static void enter_operation_enabled()
{
    if (is_torque_mode())
    {
        cmd.iq_ref = 0.0f;
        cmd.loop   = LoopMode::CURRENT;
        cmd.sensor = SensorMode::HALL;          // torque control does not work sensorless
    }
    else
    {
        cmd.speed_ref_rpm = 0.0f;
        cmd.loop          = LoopMode::SPEED;
        cmd.sensor        = (OD_RAM.x2012_sensorMode == SENSOR_MODE_HALL) ? SensorMode::HALL
                                                                          : SensorMode::SENSORLESS;
    }
    cmd.start = true;
}

static void apply_cia402_actions(const Cia402Actions& action)
{
    if (action.enter_operation)
        enter_operation_enabled();
    if (action.leave_operation)
    {
        cmd.start = false;                  // cancel start if it is not handled yet; the Supervisor checks start after stop
        if (action.soft_stop)
            cmd.stop     = true;            // 0x06 / 0x07: slow down on 0x6084 first
        else
            cmd.stop_now = true;            // 0x00, 0x02, lost master: PWM off at once
    }
    if (action.offset_cal)
        cmd.offset_cal = true;
    if (action.reset_fault)
        cmd.reset_fault = true;
    if (action.comm_fault && fault == FaultCode::NONE)
        fault = FaultCode::COMMUNICATION_LOST;
}

/* Status: bridge → OD -------------------------------------------------------*/

// on a fault change: clear the old fault with EMCY "error reset", report the new one with EMCY
static void fault_update()
{
    static FaultCode last_fault = FaultCode::NONE;
    const FaultCode  f          = fault;

    if (f == last_fault)
        return;

    if (CO != nullptr)
    {
        if (last_fault != FaultCode::NONE)
            CO_errorReset(CO->em, EM_FAULT_BIT_BASE + static_cast<uint8_t>(last_fault), 0U);
        if (f != FaultCode::NONE)
            CO_errorReport(CO->em, EM_FAULT_BIT_BASE + static_cast<uint8_t>(f), cia402_error_code(f), 0U);
    }

    OD_RAM.x603F_errorCode = cia402_error_code(f);
    last_fault             = f;
}

// Mode-specific statusword bits (only in Operation Enabled)
static uint16_t profile_status_bits()
{
    uint16_t sw = 0U;

    if (is_torque_mode())
    {
        if (trq_ramp.get_output() == trq_target)
            sw |= SW_TARGET_REACHED;
    }
    else
    {
        const float actual = static_cast<float>(OD_RAM.x606C_velocityActualValue);

        if (vel_ramp.get_output() == vel_target && std::fabs(actual - vel_target) <= VEL_WINDOW_RPM)
            sw |= SW_TARGET_REACHED;
        if (std::fabs(actual) <= VEL_ZERO_RPM)
            sw |= SW_SPEED_ZERO;
    }
    return sw;
}

static void write_statusword()
{
    uint16_t sw = cia402.statusword(status.Vdc > VDC_MIN && status.Vdc < VDC_MAX);

    if (is_operation_enabled())
        sw |= profile_status_bits();
    if (!can_owns_motor())
        sw &= static_cast<uint16_t>(~SW_REMOTE);

    OD_RAM.x6041_statusword = sw;
}

/* Public --------------------------------------------------------------------*/

void CanopenBridgeInit(void)
{
    add_write_guard(OD_ENTRY_H6060, mode_guard_ext, write_mode_guard);
    add_write_guard(OD_ENTRY_H2012, sensor_guard_ext, write_sensor_guard);
    add_write_guard(OD_ENTRY_H6007, abort_guard_ext, write_abort_guard);
}

void CanopenBridgeUpdate(void)
{
    const MotorState ms        = motor_state_machine.getState();
    const float      speed_rpm = actual_speed_rpm();

    write_motor_data(ms, speed_rpm);

    update_targets();
    update_ramps(ms, speed_rpm);
    handle_command_object();
    handle_owner_change();

    fault_update();

    const uint16_t controlword = can_owns_motor() ? OD_RAM.x6040_controlword : CW_DISABLE;

    apply_cia402_actions(cia402.update(controlword, ms, comm_lost(),
                                       OD_RAM.x6007_abortConnectionOptionCode));
    write_statusword();
}

bool CanopenBridgeStartAllowed(void)
{
    return !can_owns_motor() || is_operation_enabled();
}
