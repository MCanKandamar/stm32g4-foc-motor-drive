"""Regen test on the board: a fast speed drop makes the motor brake and push energy into the DC bus.

Usage (SLCAN adapter on test_pdo.PORT):
    python tools/hil/regen_test.py --motor-ok                          Hall, 3000 -> 500 rpm, 0x6084 = 100000 rpm/s
    python tools/hil/regen_test.py --sensorless --motor-ok
    python tools/hil/regen_test.py --scrutiny --motor-ok               also log the FOC loop (Scrutiny GUI running)

The motor runs at --v1 in profile velocity, then the target 0x60FF jumps to --v2 (RPDO1). With a high
0x6084 the speed PI asks for full braking iq. Without the regen limit the bus rises until the 30 V
over-voltage trip; with it the bus should stay near 27-29 V and the motor slows down more gently.
0x2000 (Vdc), 0x606C and 0x2002 are read over SDO every ~20 ms for 3 s after the drop.
With --scrutiny the datalogger samples every 40th FOC cycle (500 Hz) from the drop on.
Output: tools/hil/logs/regen_<sensor>_<time>.csv (and regen_<sensor>_<time>_foc.csv) and metrics.
"""
import argparse
import csv
import threading
import time
from pathlib import Path

import can
import test_pdo as t

FB = {'hall': '/global/meas/speed_rpm_hall_pll', 'sensorless': '/global/meas/speed_rpm_smo'}
OUT = Path(__file__).parent / 'logs'


def arm(c, sensor, v1):
    from scrutiny.sdk import datalogging as dl
    sig = {'ref': '/global/foc_cfg/speed_ref_rpm', 'fb': FB[sensor], 'vdc': '/global/meas/vdc',
           'scale': '/global/meas/regen_scale', 'iq_ref': '/global/meas/idq_ref/iq_ref'}
    cfg = dl.DataloggingConfig(sampling_rate=0, decimation=40, timeout=20.0, name='regen')
    ax = cfg.add_axis('value')
    for k, p in sig.items():
        cfg.add_signal(p, ax, name=k)
    cfg.configure_xaxis(dl.XAxisType.IdealTime)
    cfg.configure_trigger(dl.TriggerCondition.LessThan, [sig['ref'], v1 - 5.0], position=0.1)
    return c.start_datalog(cfg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v1', type=int, default=3000, help='speed before the drop [rpm]')
    ap.add_argument('--v2', type=int, default=500, help='speed after the drop [rpm]')
    ap.add_argument('--decel', type=int, default=100000, help='0x6084 for the drop [rpm/s]')
    ap.add_argument('--sensorless', action='store_true', help='SMO angle (default: Hall)')
    ap.add_argument('--scrutiny', action='store_true', help='also log the FOC loop with Scrutiny')
    ap.add_argument('--motor-ok', action='store_true', help='allow the run (motor rotates)')
    args = ap.parse_args()
    if not args.motor_ok:
        raise SystemExit("the motor rotates: add --motor-ok")
    sensor = 'sensorless' if args.sensorless else 'hall'
    t.SPEED_ABORT_RPM = args.v1 + 300

    c = None
    if args.scrutiny:
        import scrutiny_link as sl
        c = sl.connect()
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    poll, req, fault = [], None, None
    try:
        if not t.preflight():
            raise SystemExit("preflight failed")
        accel = t.sdo_read(0x6083)
        t.sdo_write(0x6084, 0, args.decel, 4)
        t_from = t.now()
        t.to_switch_on_disabled()
        t.sdo_write(0x6060, 0, 3, 1)
        t.sdo_write(0x2012, 0, 1 if sensor == 'hall' else 0, 1)
        t.rpdo1(0x06, args.v1); t.wait_sw(0x0231)
        t.rpdo1(0x07, args.v1); t.wait_sw(0x0233)
        t.rpdo1(0x0F, args.v1)
        try:
            t.wait_sw(0x0027, mask=0x006F)
            t.wait_state(t.ST_CL, 15.0, t_from, True)
            t0 = t.now()
            while t.now() - t0 < 1.5 + args.v1 / max(accel, 1):
                t.guard(t_from, True)
                time.sleep(0.02)
            vdc_before = t.sdo_read(0x2000) / 1000.0
            if c is not None:
                req = arm(c, sensor, args.v1)
                time.sleep(0.3)
            t_drop = t.now()
            t.rpdo1(0x0F, args.v2)
            while t.now() - t_drop < 3.0:
                ms, mv, rpm = t.sdo_read(0x2002), t.sdo_read(0x2000), t.sdo_read(0x606C, signed=True)
                poll.append((t.now() - t_drop, ms, mv / 1000.0, rpm))
                if ms == t.ST_FAULT:
                    fault = t.sdo_read(0x2003)
                    break
        finally:
            t.rpdo1(0x06, 0)
        if fault is None:
            t.wait_state(t.ST_IDLE, 10.0, t.now())
        print(f"   rotor still after {t.coast(sensor == 'hall'):.1f} s", flush=True)
        if req is not None:
            req.wait_for_completion(25)
            acq = req.fetch_acquisition(10)
    finally:
        t.safe_state()
        for idx, key in ((0x6060, '6060'), (0x2012, '2012')):
            if key in t.orig:
                t.sdo_write(idx, 0, t.orig[key], 1)
        t.sdo_write(0x6084, 0, 1000, 4)
        t._stop_rx.set(); time.sleep(0.1); t.bus.shutdown()
        if c is not None:
            c.disconnect()

    OUT.mkdir(exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    f = OUT / f'regen_{sensor}_{stamp}.csv'
    with f.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s', 'state', 'vdc', 'speed'])
        w.writerows([[f"{a:.3f}", b, f"{v:.2f}", r] for a, b, v, r in poll])
    near = [a for a, _, _, r in poll if abs(r - args.v2) < 100]
    res = dict(sensor=sensor, v1=args.v1, v2=args.v2, decel=args.decel, vdc_before=vdc_before,
               vdc_max_sdo=max(v for _, _, v, _ in poll), fault=fault,
               t_to_v2_s=round(near[0], 2) if near else None)
    if req is not None:
        x = acq.xdata.get_data()
        cols = {ds.series.name: ds.series.get_data() for ds in acq.ydata}
        ff = OUT / f'regen_{sensor}_{stamp}_foc.csv'
        with ff.open('w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_s'] + list(cols))
            for i in range(len(x)):
                w.writerow([f"{x[i]:.4f}"] + [f"{cols[k][i]:.3f}" for k in cols])
        res.update(vdc_max_foc=round(max(cols['vdc']), 2), scale_min=round(min(cols['scale']), 2),
                   iq_min=round(min(cols['iq_ref']), 2))
        print(f"foc csv: {ff}  ({len(x)} samples)")
    print(f"csv: {f}  ({len(poll)} samples)")
    print("RESULT", res, flush=True)


if __name__ == '__main__':
    main()
