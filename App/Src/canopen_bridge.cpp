#include "canopen_bridge.h"
#include "app.hpp"

extern "C" {
#include "301/CO_ODinterface.h"
#include "OD.h"
}

using namespace motor_control;

/* CiA 402 durum makinesi ---------------------------------------------------*/

// "Not Ready to Switch On" ve "Fault Reaction Active" yok: ikisi de bizde anlık.
enum class Cia402State : uint8_t
{
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,            // IDLE + akım ofseti ölçülmüş, güç katı kapalı
    OPERATION_ENABLED,      // motor çalışıyor (ADC_OFFSET_CAL … CLOSED_LOOP)
    QUICK_STOP_ACTIVE,
    FAULT
};

// Controlword'den çözülen komut. Bit 7 (fault reset) ayrıca kenarla bakılır.
enum class Cia402Cmd : uint8_t
{
    NONE,
    SHUTDOWN,               // 0x06
    SWITCH_ON,              // 0x07 (Operation Enabled'da: disable operation)
    ENABLE_OPERATION,       // 0x0F
    DISABLE_VOLTAGE,        // 0x00
    QUICK_STOP              // 0x02
};

static Cia402State cia_state = Cia402State::SWITCH_ON_DISABLED;
static bool        run_seen  = false;     // OE'de motorun IDLE'dan çıktığı görüldü
static bool        last_fault_reset_bit = false;

// CiA 402 komut tablosu: bit 7 3 2 1 0, x = bakılmaz
static Cia402Cmd decode_controlword(uint16_t cw)
{
    if ((cw & 0x0082U) == 0x0000U) return Cia402Cmd::DISABLE_VOLTAGE;     // 0xxx xx0x
    if ((cw & 0x0086U) == 0x0002U) return Cia402Cmd::QUICK_STOP;          // 0xxx x01x
    if ((cw & 0x0087U) == 0x0006U) return Cia402Cmd::SHUTDOWN;            // 0xxx x110
    if ((cw & 0x008FU) == 0x0007U) return Cia402Cmd::SWITCH_ON;           // 0xxx 0111
    if ((cw & 0x008FU) == 0x000FU) return Cia402Cmd::ENABLE_OPERATION;    // 0xxx 1111
    return Cia402Cmd::NONE;
}

static void enter_operation_enabled()
{
    cia_state = Cia402State::OPERATION_ENABLED;
    run_seen  = false;
    cmd.start = true;
}

// Operation Enabled'dan çıkış: motoru durdur, yeni duruma geç.
// Start henüz işlenmediyse iptal et; Supervisor stop'tan sonra start'a bakar.
static void leave_operation_enabled(Cia402State next)
{
    cia_state = next;
    cmd.start = false;
    cmd.stop  = true;
}

static void cia402_update()
{
    const MotorState ms = motor_state_machine.getState();

    // 1) MotorState her zaman haklı
    if (ms == MotorState::FAULT)
    {
        cia_state = Cia402State::FAULT;
    }
    else if (cia_state == Cia402State::FAULT)
    {
        cia_state = Cia402State::SWITCH_ON_DISABLED;    // fault reset tamamlandı
    }
    else if (cia_state == Cia402State::OPERATION_ENABLED)
    {
        if (ms != MotorState::IDLE)
            run_seen = true;
        else if (run_seen)                              // beklenmeyen duruş: kendiliğinden kalkmasın
            cia_state = Cia402State::SWITCH_ON_DISABLED;
    }
    else if (cia_state == Cia402State::QUICK_STOP_ACTIVE && ms == MotorState::IDLE)
    {
        cia_state = Cia402State::SWITCH_ON_DISABLED;    // motor durdu
    }

    // 2) Controlword
    const uint16_t  cw     = OD_RAM.x6040_controlword;
    const Cia402Cmd c      = decode_controlword(cw);
    const bool      fr_bit = (cw & 0x0080U) != 0U;
    const bool      fault_reset_edge = fr_bit && !last_fault_reset_bit;
    last_fault_reset_bit = fr_bit;

    // 3) Geçişler
    switch (cia_state)
    {
        case Cia402State::SWITCH_ON_DISABLED:
            if (c == Cia402Cmd::SHUTDOWN)
                cia_state = Cia402State::READY_TO_SWITCH_ON;
            break;

        case Cia402State::READY_TO_SWITCH_ON:
            if (c == Cia402Cmd::SWITCH_ON || c == Cia402Cmd::ENABLE_OPERATION)
            {
                cia_state      = Cia402State::SWITCHED_ON;    // 0x0F: bir sonraki turda OE
                cmd.offset_cal = true;
            }
            else if (c == Cia402Cmd::DISABLE_VOLTAGE || c == Cia402Cmd::QUICK_STOP)
                cia_state = Cia402State::SWITCH_ON_DISABLED;
            break;

        case Cia402State::SWITCHED_ON:
            if (c == Cia402Cmd::ENABLE_OPERATION && ms == MotorState::IDLE)
                enter_operation_enabled();
            else if (c == Cia402Cmd::SHUTDOWN)
                cia_state = Cia402State::READY_TO_SWITCH_ON;
            else if (c == Cia402Cmd::DISABLE_VOLTAGE || c == Cia402Cmd::QUICK_STOP)
                cia_state = Cia402State::SWITCH_ON_DISABLED;
            break;

        case Cia402State::OPERATION_ENABLED:
            if (c == Cia402Cmd::SWITCH_ON)
                leave_operation_enabled(Cia402State::SWITCHED_ON);
            else if (c == Cia402Cmd::SHUTDOWN)
                leave_operation_enabled(Cia402State::READY_TO_SWITCH_ON);
            else if (c == Cia402Cmd::DISABLE_VOLTAGE)
                leave_operation_enabled(Cia402State::SWITCH_ON_DISABLED);
            else if (c == Cia402Cmd::QUICK_STOP)
                leave_operation_enabled(Cia402State::QUICK_STOP_ACTIVE);   // serbest duruş, frenleme yok
            break;

        case Cia402State::QUICK_STOP_ACTIVE:
            if (c == Cia402Cmd::DISABLE_VOLTAGE)
                cia_state = Cia402State::SWITCH_ON_DISABLED;
            break;

        case Cia402State::FAULT:
            if (fault_reset_edge)
                cmd.reset_fault = true;     // FAULT'tan çıkınca 1. adım SOD'a alır
            break;
    }

    // 4) Statusword
    uint16_t sw = 0U;
    switch (cia_state)
    {
        case Cia402State::SWITCH_ON_DISABLED: sw = 0x0040U; break;
        case Cia402State::READY_TO_SWITCH_ON: sw = 0x0021U; break;
        case Cia402State::SWITCHED_ON:        sw = 0x0023U; break;
        case Cia402State::OPERATION_ENABLED:  sw = 0x0027U; break;
        case Cia402State::QUICK_STOP_ACTIVE:  sw = 0x0007U; break;
        case Cia402State::FAULT:              sw = 0x0008U; break;
    }
    if (status.Vdc > VDC_MIN && status.Vdc < VDC_MAX)
        sw |= 0x0010U;                      // bit 4: voltage enabled
    sw |= 0x0200U;                          // bit 9: remote, controlword dinleniyor

    OD_RAM.x6041_statusword = sw;
}


void CanopenBridgeUpdate(void)
{
    //  motor → OD
    const float speed_rpm = (foc_cfg.angle_src == AngleSource::HALL) ? meas.speed_rpm_hall
                                                                     : meas.speed_rpm_smo;

    OD_RAM.x2000_vdcMV       = static_cast<uint16_t>(lroundf(status.Vdc * 1000.0f));   // 24.1 V → 24100
    OD_RAM.x2001_speedRpm    = static_cast<int16_t>(lroundf(speed_rpm));
    OD_RAM.x2002_motorState  = static_cast<uint8_t>(motor_state_machine.getState());
    OD_RAM.x2003_faultCode   = static_cast<uint8_t>(fault);

    // speed ref: OD → cmd
    static int32_t last_speed_ref = 0;

    if (OD_RAM.x2010_speedRefRpm != last_speed_ref)
    {
        last_speed_ref     = OD_RAM.x2010_speedRefRpm;
        cmd.speed_ref_rpm  = static_cast<float>(last_speed_ref);
    }

    // command: OD → cmd. Start ve fault reset sadece CiA 402 controlword'den.
    switch (OD_RAM.x2011_command)
    {
        case 2: cmd.stop        = true; break;
        case 4: cmd.calibrate   = true; break;
        default:                        break;
    }
    OD_RAM.x2011_command = 0;

    cia402_update();
}
