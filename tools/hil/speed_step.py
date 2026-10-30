"""Speed-loop step response on the board: profile velocity, 0x2010 step, Scrutiny datalog.

Usage (Scrutiny GUI running, SLCAN adapter on test_pdo.PORT):
    python tools/hil/speed_step.py --motor-ok                          Hall, 1000 -> 1300 rpm, gains unchanged
    python tools/hil/speed_step.py --kp 0.002 --ki 0.05 --motor-ok     write the speed PI gains (RAM) first
    python tools/hil/speed_step.py --sensorless --v1 800 --v2 1200 --motor-ok
    python tools/hil/speed_step.py --ramp --motor-ok                   v1 -> v2 on the 0x6083/0x6084 ramp

The motor is started over CAN (RPDO1, controlword 06/07/0F) and runs at v1 until the ramp is done.
Then 0x2010 (speed ref without a ramp) jumps to v2, or with --ramp the target 0x60FF changes to v2
(RPDO1) and the profile ramp moves the ref. The datalogger samples the 20 kHz FOC loop every DECIM
cycles (500 Hz, about 1 s for 4 float signals in the 8 KB buffer) and triggers when the speed
reference passes the middle of the step, or with --ramp when it leaves v1.
Gains written with --kp/--ki stay in RAM until the next reset (the code values come back then).
Output: tools/hil/logs/step_<sensor>_<v1>_<v2>_<time>.csv (t_s, ref, fb, avg, iq_ref) and metrics.
"""
import argparse
import csv
import threading
import time
from pathlib import Path

import can
import scrutiny_link as sl
import test_pdo as t
from scrutiny.sdk import datalogging as dl

DECIM = 40                                   # 20 kHz / 40 = 500 Hz
KP = '/global/speed_ctrl/SpeedController/config/Kp'
KI = '/global/speed_ctrl/SpeedController/config/Ki'
FB = {'hall': '/global/meas/speed_rpm_hall_pll', 'sensorless': '/global/meas/speed_rpm_smo'}
OUT = Path(__file__).parent / 'logs'


def arm(c, sensor, v1, v2, ramp):
    sig = {'ref': '/global/foc_cfg/speed_ref_rpm', 'fb': FB[sensor],
           'avg': '/global/meas/speed_rpm_hall', 'iq_ref': '/global/meas/idq_ref/iq_ref'}
    cfg = dl.DataloggingConfig(sampling_rate=0, decimation=DECIM, timeout=20.0, name='speed step')
    ax = cfg.add_axis('rpm')
    ax_i = cfg.add_axis('A')
    for k, p in sig.items():
        cfg.add_signal(p, ax_i if k == 'iq_ref' else ax, name=k)
    cfg.configure_xaxis(dl.XAxisType.IdealTime)
    cond = dl.TriggerCondition.GreaterThan if v2 > v1 else dl.TriggerCondition.LessThan
    level = v1 + (2.0 if v2 > v1 else -2.0) if ramp else (v1 + v2) / 2.0
    cfg.configure_trigger(cond, [sig['ref'], level], position=0.1)
    return c.start_datalog(cfg)


def run_motor(sensor, v1, v2, settle_s, hold_s, ramp, on_ready):
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6060, 0, 3, 1)                # profile velocity
    t.sdo_write(0x2012, 0, 1 if sensor == 'hall' else 0, 1)
    t.rpdo1(0x06, v1); t.wait_sw(0x0231)
    t.rpdo1(0x07, v1); t.wait_sw(0x0233)
    t.rpdo1(0x0F, v1)
    try:
        t.wait_sw(0x0027, mask=0x006F)
        t.wait_state(t.ST_CL, 10.0, t_from, True)
        t0 = t.now()
        while t.now() - t0 < settle_s:          # ramp to v1 and settle
            t.guard(t_from, True)
            time.sleep(0.02)
        t.sdo_write(0x2010, 0, v1, 4)           # no change if 0x2010 = v1 already, else the ramp jumps to v1
        on_ready()                              # arm here: before the run the ref may still hold the last v2
        time.sleep(0.3)
        if ramp:
            t.rpdo1(0x0F, v2)                   # new target, the profile ramp moves the ref
        else:
            t.sdo_write(0x2010, 0, v2, 4)       # the step
        t0 = t.now()
        while t.now() - t0 < hold_s:
            t.guard(t_from, True)
            time.sleep(0.02)
    finally:
        t.rpdo1(0x06, 0)
        t.sdo_write(0x2010, 0, 0, 4)
    t.wait_sw(0x0231, 0.5)
    print(f"   stopped, rotor still after {t.coast(sensor == 'hall'):.1f} s", flush=True)


def ramp_metrics(x, ref, fb, iq, v1, v2):
    d = v2 - v1
    i0 = next(i for i in range(len(ref)) if (ref[i] - v1) * d > 0.0)
    i1 = next(i for i in range(i0, len(ref)) if ref[i] == v2)
    track = [fb[i] - ref[i] for i in range(i0, i1)]
    after = [(fb[i] - v2) * (1 if d > 0 else -1) for i in range(i1, len(fb))]
    out = [i for i in range(i1, len(fb)) if abs(fb[i] - v2) > 10.0]
    return dict(ramp_ms=round((x[i1] - x[i0]) * 1000.0),
                track_err_rpm=round(max(track, key=abs), 1),
                overshoot_rpm=round(max(after), 1),
                settle_10rpm_ms=round((x[out[-1]] - x[i1]) * 1000.0) if out else 0,
                iq_peak_a=round(max(iq[i0:], key=abs), 2))


def metrics(x, ref, fb, iq, v1, v2):
    i0 = next(i for i in range(len(ref)) if (ref[i] - v1) * (v2 - v1) > 0.5 * (v2 - v1) ** 2)
    d = v2 - v1
    y = [(f - v1) / d for f in fb]              # 0 → 1 over the step
    t_at = lambda lvl: next((x[i] - x[i0] for i in range(i0, len(y)) if y[i] >= lvl), float('nan'))
    out = [i for i in range(i0, len(y)) if abs(y[i] - 1.0) > 0.05]
    tail = y[int(len(y) * 0.7):]
    mean = sum(tail) / len(tail)
    return dict(rise_10_90_ms=round((t_at(0.9) - t_at(0.1)) * 1000.0, 1),
                overshoot_pct=round((max(y[i0:]) - 1.0) * 100.0, 1),
                settle_5pct_ms=round((x[out[-1]] - x[i0]) * 1000.0, 1) if out else 0.0,
                tail_err_rpm=round((mean - 1.0) * d, 1),
                tail_ripple_rpm=round((sum((v - mean) ** 2 for v in tail) / len(tail)) ** 0.5 * abs(d), 1),
                iq_peak_a=round(max(iq[i0:], key=abs), 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v1', type=int, default=1000, help='start speed [rpm]')
    ap.add_argument('--v2', type=int, default=1300, help='speed after the step [rpm]')
    ap.add_argument('--sensorless', action='store_true', help='SMO angle (default: Hall)')
    ap.add_argument('--ramp', action='store_true', help='move on the profile ramp (default: step on 0x2010)')
    ap.add_argument('--kp', type=float, help='speed PI Kp [A/rpm], written to RAM before the run')
    ap.add_argument('--ki', type=float, help='speed PI Ki [A/(rpm s)], written to RAM before the run')
    ap.add_argument('--motor-ok', action='store_true', help='allow the run (motor rotates)')
    args = ap.parse_args()
    if not args.motor_ok:
        raise SystemExit("the motor rotates: add --motor-ok")
    sensor = 'sensorless' if args.sensorless else 'hall'

    c = sl.connect()
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if not t.preflight():
            raise SystemExit("preflight failed")
        kp, ki = c.watch(KP), c.watch(KI)
        kp.wait_update(2); ki.wait_update(2)
        if args.kp is not None:
            kp.value = args.kp
        if args.ki is not None:
            ki.value = args.ki
        time.sleep(0.2)
        gains = dict(kp=kp.value, ki=ki.value)
        print("gains:", gains, flush=True)
        req = []
        run_motor(sensor, args.v1, args.v2, 2.5 + abs(args.v1) / 1000.0, 1.5, args.ramp,
                  lambda: req.append(arm(c, sensor, args.v1, args.v2, args.ramp)))
        req = req[0]
        req.wait_for_completion(25)
        acq = req.fetch_acquisition(10)
    finally:
        t.safe_state()
        for idx, key in ((0x6060, '6060'), (0x2012, '2012')):
            if key in t.orig:
                t.sdo_write(idx, 0, t.orig[key], 1)
        t._stop_rx.set(); time.sleep(0.1); t.bus.shutdown()
        c.disconnect()

    x = acq.xdata.get_data()
    cols = {ds.series.name: ds.series.get_data() for ds in acq.ydata}
    OUT.mkdir(exist_ok=True)
    kind = 'ramp' if args.ramp else 'step'
    f = OUT / time.strftime(f'{kind}_{sensor}_{args.v1}_{args.v2}_%Y%m%d_%H%M%S.csv')
    with f.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s'] + list(cols))
        for i in range(len(x)):
            w.writerow([f"{x[i]:.4f}"] + [f"{cols[k][i]:.3f}" for k in cols])
    m = (ramp_metrics if args.ramp else metrics)(x, cols['ref'], cols['fb'], cols['iq_ref'], args.v1, args.v2)
    print(f"csv: {f}  ({len(x)} samples, {x[-1] - x[0]:.2f} s)")
    print("RESULT", dict(sensor=sensor, v1=args.v1, v2=args.v2, **gains, **m), flush=True)


if __name__ == '__main__':
    main()
