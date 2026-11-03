"""Scrutiny datalog of a sensorless run: the start-up sequence, or the SMO angle against the Hall angle.

Usage (Scrutiny GUI running, SLCAN adapter on test_pdo.PORT):
    python tools/hil/sensorless_capture.py --startup --motor-ok        IDLE -> ALIGN -> OPEN_LOOP -> CLOSED_LOOP
    python tools/hil/sensorless_capture.py --angle --motor-ok          SMO vs Hall angle at a steady speed
    python tools/hil/sensorless_capture.py --angle --target 1500 --motor-ok
    python tools/hil/sensorless_capture.py --waves --motor-ok          SMO back-EMF and PWM duties at a steady speed

The motor is started over CAN in profile velocity with the sensorless angle (0x2012 = 0).
--startup samples every 400th FOC cycle (50 Hz, about 7 s) and triggers when the motor leaves IDLE.
--angle and --waves wait in closed loop for the speed to settle, then sample every 10th FOC cycle
(2 kHz, about 0.3 s or 0.2 s). The Hall sensors are read in both modes but only for comparison; the FOC uses the SMO angle.
Output: tools/hil/logs/<startup|angle|waves>_<target>_<time>.csv.
"""
import argparse
import csv
import math
import threading
import time
from pathlib import Path

import can
import scrutiny_link as sl
import test_pdo as t
from scrutiny.sdk import datalogging as dl

STATE = '/global/motor_state_machine/motor_state'
SIG_STARTUP = {'state': STATE, 'ref': '/global/foc_cfg/speed_ref_rpm',
               'ol_speed': '/global/foc_cfg/ol_speed', 'smo': '/global/meas/speed_rpm_smo',
               'hall': '/global/meas/speed_rpm_hall_pll', 'iq_ref': '/global/meas/idq_ref/iq_ref'}
SIG_ANGLE = {'hall': '/global/meas/theta_e_rad_pll', 'smo': '/global/meas/theta_e_rad_smo',
             'offset': '/global/foc_cfg/angle_offset'}
SIG_WAVES = {'z_alpha': '/global/meas/z_alphabeta/z_alpha', 'z_beta': '/global/meas/z_alphabeta/z_beta',
             'duty_a': '/global/meas/duties/duty_a', 'duty_b': '/global/meas/duties/duty_b',
             'duty_c': '/global/meas/duties/duty_c'}
OUT = Path(__file__).parent / 'logs'


def arm(c, kind):
    startup = kind == 'startup'
    sig = {'startup': SIG_STARTUP, 'angle': SIG_ANGLE, 'waves': SIG_WAVES}[kind]
    cfg = dl.DataloggingConfig(sampling_rate=0, decimation=400 if startup else 10, timeout=30.0,
                               name=f'sensorless {kind}')
    ax = cfg.add_axis('value')
    for k, p in sig.items():
        cfg.add_signal(p, ax, name=k)
    cfg.configure_xaxis(dl.XAxisType.IdealTime)
    if startup:
        cfg.configure_trigger(dl.TriggerCondition.GreaterThan, [STATE, 1], position=0.02)   # leaves IDLE
    else:
        cfg.configure_trigger(dl.TriggerCondition.AlwaysTrue, position=0.0)
    return c.start_datalog(cfg)


def run_motor(target, startup, on_ready):
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6060, 0, 3, 1)                # profile velocity
    t.sdo_write(0x2012, 0, 0, 1)                # sensorless
    if startup:
        on_ready()
        time.sleep(0.3)
    t.rpdo1(0x06, target); t.wait_sw(0x0231)
    t.rpdo1(0x07, target); t.wait_sw(0x0233)
    t.rpdo1(0x0F, target)
    try:
        t.wait_sw(0x0027, mask=0x006F)
        t.wait_state(t.ST_CL, 15.0, t_from, True)
        hold_s = 4.0 if startup else 2.5 + target / 1000.0     # startup: ramp to target, angle: settle first
        t0 = t.now()
        while t.now() - t0 < hold_s:
            t.guard(t_from, True)
            time.sleep(0.02)
        if not startup:
            on_ready()
            time.sleep(1.0)
            t.guard(t_from, True)
    finally:
        t.rpdo1(0x06, 0)
    t.wait_sw(0x0231, 0.5)
    print(f"   stopped, rotor still after {t.coast(False):.1f} s", flush=True)


def angle_metrics(hall, smo, offset):
    # electrical angle error, wrapped to -180..180 deg
    err = [math.degrees(math.remainder(s + o - h, 2.0 * math.pi)) for h, s, o in zip(hall, smo, offset)]
    mean = sum(err) / len(err)
    rms = (sum((e - mean) ** 2 for e in err) / len(err)) ** 0.5
    return dict(err_mean_deg=round(mean, 1), err_std_deg=round(rms, 1),
                err_max_deg=round(max(err, key=abs), 1))


def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--startup', action='store_true', help='log the start-up sequence')
    mode.add_argument('--angle', action='store_true', help='log the SMO and Hall angles at a steady speed')
    mode.add_argument('--waves', action='store_true', help='log the SMO back-EMF and the PWM duties at a steady speed')
    ap.add_argument('--target', type=int, default=1000, help='speed target [rpm]')
    ap.add_argument('--motor-ok', action='store_true', help='allow the run (motor rotates)')
    args = ap.parse_args()
    if not args.motor_ok:
        raise SystemExit("the motor rotates: add --motor-ok")

    c = sl.connect()
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if not t.preflight():
            raise SystemExit("preflight failed")
        kind = 'startup' if args.startup else 'angle' if args.angle else 'waves'
        req = []
        run_motor(args.target, args.startup, lambda: req.append(arm(c, kind)))
        req = req[0]
        req.wait_for_completion(30)
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
    f = OUT / time.strftime(f'{kind}_{args.target}_%Y%m%d_%H%M%S.csv')
    with f.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s'] + list(cols))
        for i in range(len(x)):
            w.writerow([f"{x[i]:.5f}"] + [f"{cols[k][i]:.4f}" for k in cols])
    print(f"csv: {f}  ({len(x)} samples, {x[-1] - x[0]:.2f} s)")
    if args.angle:
        print("RESULT", dict(target=args.target, **angle_metrics(cols['hall'], cols['smo'], cols['offset'])),
              flush=True)


if __name__ == '__main__':
    main()
