"""Small drive commands over CAN SDO (node-ID 13, SLCAN adapter on test_pdo.PORT).

Usage:
    python tools/hil/drive_util.py status        read Vdc, motor state, fault, 0x603F, 0x1001, statusword
    python tools/hil/drive_util.py fault-reset   controlword 0x00 -> 0x80 -> 0x00, only if Vdc > 20 V
    python tools/hil/drive_util.py calibrate     Hall edge calibration (0x2011 = 4, motor turns slowly, ~10 s)

The calibration is kept in RAM only: it is lost when the board loses power.
"""
import argparse
import threading
import time

import can
import test_pdo as t

ST_CAL = 6                                   # MotorState::CALIBRATING
VDC_MIN_MV = 20000


def status():
    return dict(vdc_mV=t.sdo_read(0x2000), motor_state=t.sdo_read(0x2002), fault=t.sdo_read(0x2003),
                err_603F=hex(t.sdo_read(0x603F)), err_reg_1001=hex(t.sdo_read(0x1001)),
                sw=hex(t.sdo_read(0x6041)))


def fault_reset():
    vdc = t.sdo_read(0x2000)
    if vdc <= VDC_MIN_MV:
        raise SystemExit(f"Vdc {vdc} mV <= {VDC_MIN_MV} mV: no fault reset")
    for cw in (0x0000, 0x0080, 0x0000):      # reset on the rising edge of bit 7
        t.sdo_write(0x6040, 0, cw, 2)
        time.sleep(0.05)
    time.sleep(0.5)


def calibrate():
    t_from = t.now()
    t.sdo_write(0x6040, 0, 0x0000, 2)
    if t.sdo_read(0x2002) != t.ST_IDLE:
        raise SystemExit("motor not IDLE: calibration not started")
    t.sdo_write(0x2011, 0, 4, 1)
    t0 = t.now()
    seq = []
    try:
        while t.now() - t0 < 20.0:
            t.guard(t_from, False)
            ms = t.sdo_read(0x2002)
            if not seq or seq[-1][1] != ms:
                seq.append((round(t.now() - t0, 2), ms))
                print(f"  t={seq[-1][0]:5.2f} s  motor_state={ms}", flush=True)
            if ms == t.ST_FAULT or (ms == t.ST_IDLE and len(seq) > 1):
                break
            time.sleep(0.05)
    except BaseException:
        t.sdo_write(0x2011, 0, 2, 1)         # stop
        raise
    states = [s for _, s in seq]
    ok = states in ([ST_CAL, t.ST_IDLE], [t.ST_IDLE, ST_CAL, t.ST_IDLE])
    print("RESULT", "PASS" if ok else "FAIL", dict(states=seq, fault=t.sdo_read(0x2003)), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('command', choices=('status', 'fault-reset', 'calibrate'))
    args = ap.parse_args()

    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if args.command == 'fault-reset':
            fault_reset()
        elif args.command == 'calibrate':
            calibrate()
        print(status(), flush=True)
    finally:
        t._stop_rx.set()
        time.sleep(0.1)
        t.bus.shutdown()


if __name__ == '__main__':
    main()
