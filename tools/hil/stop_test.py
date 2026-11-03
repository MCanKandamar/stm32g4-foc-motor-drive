"""Stop test on the board: 0x06 slows down on the 0x6084 ramp, 0x00 turns PWM off at once.

Usage (Scrutiny GUI running, SLCAN adapter on test_pdo.PORT):
    python tools/hil/stop_test.py --motor-ok                       Hall, 1000 rpm, 0x06
    python tools/hil/stop_test.py --sensorless --motor-ok
    python tools/hil/stop_test.py --cw 0x00 --motor-ok             disable voltage: no ramp expected
    python tools/hil/stop_test.py --decel 500 --motor-ok           write 0x6084 [rpm/s] first
    python tools/hil/stop_test.py --can-only --motor-ok            no Scrutiny: poll 0x606C and 0x2002 over SDO

The motor runs at --target in profile velocity, then the controlword --cw is sent over RPDO1.
The datalogger samples every 100th FOC cycle (200 Hz, about 2.4 s) from when the stop starts.
Expected with 0x06/0x07: the speed ref falls at 0x6084 to 0 rpm (Hall) or 300 rpm (sensorless),
then the motor goes IDLE. With 0x00/0x02: IDLE within a few ms.
With --can-only, 0x606C and 0x2002 are read over SDO about every 20 ms instead.
Output: tools/hil/logs/stop_<sensor>_<cw>_<time>.csv (t_s, state, ref, fb, vdc, iq_ref, or with
--can-only t_s, state, speed) and metrics.
"""
import argparse
import csv
import threading
import time
from pathlib import Path

import can
import test_pdo as t

FB = {'hall': '/global/meas/speed_rpm_hall_pll', 'sensorless': '/global/meas/speed_rpm_smo'}
STATE = '/global/motor_state_machine/motor_state'
OUT = Path(__file__).parent / 'logs'


def arm(c, sensor, cw):
    from scrutiny.sdk import datalogging as dl
    sig = {'state': STATE, 'ref': '/global/foc_cfg/speed_ref_rpm', 'fb': FB[sensor],
           'vdc': '/global/status/Vdc', 'iq_ref': '/global/meas/idq_ref/iq_ref'}
    cfg = dl.DataloggingConfig(sampling_rate=0, decimation=100, timeout=20.0, name='stop')
    ax = cfg.add_axis('value')
    for k, p in sig.items():
        cfg.add_signal(p, ax, name=k)
    cfg.configure_xaxis(dl.XAxisType.IdealTime)
    if cw in (0x06, 0x07):
        cfg.configure_trigger(dl.TriggerCondition.LessThan, [sig['ref'], cfg_target - 5.0], position=0.1)
    else:
        cfg.configure_trigger(dl.TriggerCondition.LessThan, [STATE, 4], position=0.1)    # leaves CLOSED_LOOP
    return c.start_datalog(cfg)


def main():
    global cfg_target
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=int, default=1000, help='speed before the stop [rpm]')
    ap.add_argument('--sensorless', action='store_true', help='SMO angle (default: Hall)')
    ap.add_argument('--cw', type=lambda s: int(s, 0), default=0x06, help='stop controlword (0x06, 0x07, 0x00, 0x02)')
    ap.add_argument('--decel', type=int, help='write 0x6084 [rpm/s] before the run')
    ap.add_argument('--can-only', action='store_true', help='no Scrutiny: poll 0x606C and 0x2002 over SDO')
    ap.add_argument('--motor-ok', action='store_true', help='allow the run (motor rotates)')
    args = ap.parse_args()
    if not args.motor_ok:
        raise SystemExit("the motor rotates: add --motor-ok")
    sensor = 'sensorless' if args.sensorless else 'hall'
    cfg_target = args.target

    c = None
    if not args.can_only:
        import scrutiny_link as sl
        c = sl.connect()
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if not t.preflight():
            raise SystemExit("preflight failed")
        if args.decel is not None:
            t.sdo_write(0x6084, 0, args.decel, 4)
        decel = t.sdo_read(0x6084)
        t_from = t.now()
        t.to_switch_on_disabled()
        t.sdo_write(0x6060, 0, 3, 1)
        t.sdo_write(0x2012, 0, 1 if sensor == 'hall' else 0, 1)
        t.rpdo1(0x06, args.target); t.wait_sw(0x0231)
        t.rpdo1(0x07, args.target); t.wait_sw(0x0233)
        t.rpdo1(0x0F, args.target)
        try:
            t.wait_sw(0x0027, mask=0x006F)
            t.wait_state(t.ST_CL, 15.0, t_from, True)
            t0 = t.now()
            while t.now() - t0 < 2.5 + args.target / 1000.0:
                t.guard(t_from, True)
                time.sleep(0.02)
            req = None if args.can_only else arm(c, sensor, args.cw)
            time.sleep(0.3)
            poll = []
            t_stop = t.now()
            t.rpdo1(args.cw, args.target)
            if args.can_only:
                t_idle = None
                while t.now() - t_stop < 10.0:
                    ms, v = t.sdo_read(0x2002), t.sdo_read(0x606C, signed=True)
                    poll.append((t.now() - t_stop, ms, v))
                    if ms == t.ST_IDLE and t_idle is None:
                        t_idle = t.now() - t_stop
                    if t_idle is not None and t.now() - t_stop > t_idle + 0.3:
                        break
                if t_idle is None:
                    raise RuntimeError("no IDLE within 10 s")
            else:
                t_idle = t.wait_state(t.ST_IDLE, 10.0, t_stop)
        finally:
            t.rpdo1(0x06, 0)
        print(f"   stop {args.cw:#04x}: IDLE after {t_idle * 1000:.0f} ms, 0x6084 = {decel} rpm/s", flush=True)
        if req is not None:
            req.wait_for_completion(25)
            acq = req.fetch_acquisition(10)
        print(f"   rotor still after {t.coast(sensor == 'hall'):.1f} s", flush=True)
    finally:
        t.safe_state()
        for idx, key in ((0x6060, '6060'), (0x2012, '2012')):
            if key in t.orig:
                t.sdo_write(idx, 0, t.orig[key], 1)
        if args.decel is not None:
            t.sdo_write(0x6084, 0, 1000, 4)
        t._stop_rx.set(); time.sleep(0.1); t.bus.shutdown()
        if c is not None:
            c.disconnect()

    OUT.mkdir(exist_ok=True)
    if args.can_only:
        f = OUT / time.strftime(f'stop_can_{sensor}_{args.cw:02x}_%Y%m%d_%H%M%S.csv')
        with f.open('w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t_s', 'state', 'speed'])
            w.writerows([[f"{a:.3f}", b, v] for a, b, v in poll])
        print(f"csv: {f}  ({len(poll)} samples)")
        print("RESULT", dict(sensor=sensor, cw=hex(args.cw), decel=decel, idle_ms=round(t_idle * 1000),
                             speed_at_idle=next(v for a, b, v in poll if b == t.ST_IDLE)), flush=True)
        return

    x = acq.xdata.get_data()
    cols = {ds.series.name: ds.series.get_data() for ds in acq.ydata}
    f = OUT / time.strftime(f'stop_{sensor}_{args.cw:02x}_%Y%m%d_%H%M%S.csv')
    with f.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s'] + list(cols))
        for i in range(len(x)):
            w.writerow([f"{x[i]:.4f}"] + [f"{cols[k][i]:.3f}" for k in cols])
    cl = [i for i, s in enumerate(cols['state']) if s == 4]
    ramp_s = (x[cl[-1]] - x[0]) - (x[-1] - x[0]) * 0.1 if cl else 0.0
    print(f"csv: {f}  ({len(x)} samples, {x[-1] - x[0]:.2f} s)")
    print("RESULT", dict(sensor=sensor, cw=hex(args.cw), decel=decel, idle_ms=round(t_idle * 1000),
                         ramp_in_log_s=round(ramp_s, 2), ref_end=round(cols['ref'][cl[-1]], 1) if cl else None,
                         vdc_max=round(max(cols['vdc']), 2), vdc_min=round(min(cols['vdc']), 2)), flush=True)


if __name__ == '__main__':
    main()
