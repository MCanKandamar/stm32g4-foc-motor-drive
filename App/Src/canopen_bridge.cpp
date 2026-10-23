#include "canopen_bridge.h"
#include "app.hpp"

extern "C" {
#include "301/CO_ODinterface.h" 
#include "OD.h"
}

using namespace motor_control;

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

    // command: OD → cmd
    switch (OD_RAM.x2011_command)
    {
        case 1: cmd.start       = true; break;
        case 2: cmd.stop        = true; break;
        case 3: cmd.reset_fault = true; break;
        case 4: cmd.calibrate   = true; break;
        default:                        break;
    }
    OD_RAM.x2011_command = 0;
}
