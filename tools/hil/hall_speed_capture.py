"""Scrutiny datalog of Hall vs SMO speed on a profile velocity Hall run 0 -> TARGET rpm.

Usage (Scrutiny GUI running, SLCAN adapter on test_pdo.PORT):
    python tools/hil/hall_speed_capture.py --motor-ok
    python tools/hil/hall_speed_capture.py --target 800 --motor-ok

The motor is started over CAN (RPDO1, controlword 06/07/0F). The datalogger samples the
20 kHz FOC loop every DECIM cycles (250 Hz, about 2 s for 4 float signals in the 8 KB
buffer) and triggers when the speed reference passes 20 rpm. Output:
tools/hil/logs/hall_smo_<target>_<time>.csv (t_s, ref, hall, smo, iq_ref).
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

DECIM = 80                                   # 20 kHz / 80 = 250 Hz
SIG = {'ref': '/global/foc_cfg/speed_ref_rpm', 'hall': '/global/meas/speed_rpm_hall',
       'smo': '/global/meas/speed_rpm_smo', 'iq_ref': '/global/meas/idq_ref/iq_ref'}
OUT = Path(__file__).parent / 'logs'


def arm(c):
    cfg = dl.DataloggingConfig(sampling_rate=0, decimation=DECIM, timeout=20.0, name='hall vs smo')
    ax = cfg.add_axis('rpm')
    ax_i = cfg.add_axis('A')
    for k, p in SIG.items():
        cfg.add_signal(p, ax_i if k == 'iq_ref' else ax, name=k)
    cfg.configure_xaxis(dl.XAxisType.IdealTime)
    cfg.configure_trigger(dl.TriggerCondition.GreaterThan, [SIG['ref'], 20.0], position=0.05)
    return c.start_datalog(cfg)


def run_motor(target, hold_s):
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6060, 0, 3, 1)                # profile velocity
    t.sdo_write(0x2012, 0, 1, 1)                # Hall
    t.rpdo1(0x06, target); t.wait_sw(0x0231)
    t.rpdo1(0x07, target); t.wait_sw(0x0233)
    t.rpdo1(0x0F, target)
    try:
        t.wait_sw(0x0027, mask=0x006F)
        t0 = t.now()
        while t.now() - t0 < hold_s:
            t.guard(t_from, True)
            time.sleep(0.02)
    finally:
        t.rpdo1(0x06, 0)
    t.wait_sw(0x0231, 0.5)
    print(f"   stopped, rotor still after {t.coast(True):.1f} s", flush=True)


def main():
    ap = argparse.ArgumentParser()
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
        req = arm(c)
        time.sleep(0.5)
        run_motor(args.target, 3.0)
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
    f = OUT / time.strftime(f'hall_smo_{args.target}_%Y%m%d_%H%M%S.csv')
    with f.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s'] + list(cols))
        for i in range(len(x)):
            w.writerow([f"{x[i]:.4f}"] + [f"{cols[k][i]:.2f}" for k in cols])
    print(f"csv: {f}  ({len(x)} samples, {x[-1] - x[0]:.2f} s)")


if __name__ == '__main__':
    main()
