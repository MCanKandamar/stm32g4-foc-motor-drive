"""CANopen phase 6 hardware tests (PDO + SYNC) for the BLDC drive, node-ID 13.

Usage (python-can, SLCAN adapter on PORT):
    python tools/hil/test_pdo.py --tests D
    python tools/hil/test_pdo.py --tests A,B,C --motor-ok
    python tools/hil/test_pdo.py --tests C --tq 80 --tq-time 0.45 --motor-ok

D: SYNC mode (motor off)   A: pv sensorless target change via RPDO1
B: pv Hall via RPDO1       C: tq via RPDO2 (an unloaded motor speeds up fast)
A/B/C rotate the motor and only run with --motor-ok. A speed above SPEED_ABORT_RPM or a
motor EMCY stops the run. On every exit: 0x6040 = 0, 0x2011 = 2, 0x1800:02 = 0xFE,
0x6060/0x2012 restored. Log: tools/hil/logs/pdo_<date>_<time>.json
"""
import argparse
import json
import queue
import threading
import time
from pathlib import Path

import can

PORT = 'COM15'
SDO_TX, SDO_RX, EMCY, HB = 0x60D, 0x58D, 0x08D, 0x70D
RPDO1, RPDO2, TPDO1, TPDO2, SYNC = 0x20D, 0x30D, 0x18D, 0x28D, 0x080
SPEED_ABORT_RPM = 2500
ST_IDLE, ST_CL, ST_FAULT = 1, 4, 5
TQ_SETTLED_S = 0.1           # C: 0x0637 is reached by ~97 ms, 0x6077 is read by SDO from here on

bus = None
rxq = queue.Queue()
log = []
_stop_rx = threading.Event()
T0 = time.perf_counter()
results = []
orig = {}


class SafetyStop(Exception):
    pass


def now():
    return time.perf_counter()


def _rx_thread():
    while not _stop_rx.is_set():
        m = bus.recv(timeout=0.05)
        if m is None:
            continue
        rec = (now(), m.arbitration_id, bytes(m.data))
        log.append(rec)
        rxq.put(rec)


def _drain():
    while True:
        try:
            rxq.get_nowait()
        except queue.Empty:
            return


def send(cid, data):
    bus.send(can.Message(arbitration_id=cid, data=bytes(data), is_extended_id=False))


def _wait_sdo(index, sub, timeout=0.5):
    t_end = now() + timeout
    while now() < t_end:
        try:
            t, cid, data = rxq.get(timeout=0.01)
        except queue.Empty:
            continue
        if cid == SDO_RX and data[1] == (index & 0xFF) and data[2] == (index >> 8) and data[3] == sub:
            return data
    raise TimeoutError(f"no SDO answer for 0x{index:04X}:{sub}")


def sdo_write(index, sub, value, size):
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
    _drain()
    send(SDO_TX, [cmd, index & 0xFF, index >> 8, sub] + list((value & 0xFFFFFFFF).to_bytes(4, 'little')))
    r = _wait_sdo(index, sub)
    if r[0] == 0x80:
        raise RuntimeError(f"SDO abort 0x{int.from_bytes(r[4:8], 'little'):08X} on write 0x{index:04X}:{sub}")


def sdo_read(index, sub=0, signed=False):
    _drain()
    send(SDO_TX, [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])
    r = _wait_sdo(index, sub)
    if r[0] == 0x80:
        raise RuntimeError(f"SDO abort 0x{int.from_bytes(r[4:8], 'little'):08X} on read 0x{index:04X}:{sub}")
    n = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(r[0], 4)
    return int.from_bytes(r[4:4 + n], 'little', signed=signed)


def frames(cid, t_from, t_to=None):
    t_to = t_to if t_to is not None else now()
    return [(t, d) for (t, i, d) in list(log) if i == cid and t_from <= t <= t_to]


def tpdo1(t_from, t_to=None):          # (t, statusword, rpm)
    return [(t, int.from_bytes(d[0:2], 'little'), int.from_bytes(d[2:6], 'little', signed=True))
            for t, d in frames(TPDO1, t_from, t_to)]


def tpdo2(t_from, t_to=None):          # (t, statusword, torque permille, mode)
    return [(t, int.from_bytes(d[0:2], 'little'), int.from_bytes(d[2:4], 'little', signed=True),
             int.from_bytes(d[4:5], 'little', signed=True)) for t, d in frames(TPDO2, t_from, t_to)]


def rpdo1(cw, rpm):
    send(RPDO1, cw.to_bytes(2, 'little') + (rpm & 0xFFFFFFFF).to_bytes(4, 'little'))


def rpdo2(cw, permille):
    send(RPDO2, cw.to_bytes(2, 'little') + (permille & 0xFFFF).to_bytes(2, 'little'))


def guard(t_from, check_speed):
    # skip "error reset" (code 0x0000) and CAN communication (0x81xx) EMCYs: they are not motor faults
    e = [(t, d) for t, d in frames(EMCY, t_from)
         if int.from_bytes(d[0:2], 'little') != 0 and (int.from_bytes(d[0:2], 'little') >> 8) != 0x81]
    if e:
        raise SafetyStop(f"EMCY {e[-1][1].hex(' ')}")
    if check_speed:
        f = tpdo1(now() - 0.25)
        if f and abs(f[-1][2]) > SPEED_ABORT_RPM:
            raise SafetyStop(f"speed {f[-1][2]} rpm > {SPEED_ABORT_RPM} rpm")


def wait_sw(expected, timeout=0.3, mask=0xFFFF):
    t0 = now()
    sw = None
    while now() - t0 < timeout:
        sw = sdo_read(0x6041)
        if (sw & mask) == expected:
            return sw
        time.sleep(0.002)
    raise RuntimeError(f"statusword {sw:#06x} (mask {mask:#06x}) != {expected:#06x} after {timeout} s")


def wait_state(state, timeout, t_from, check_speed=False):
    t0 = now()
    ms = None
    while now() - t0 < timeout:
        guard(t_from, check_speed)
        ms = sdo_read(0x2002)
        if ms == state:
            return now() - t0
        if ms == ST_FAULT:
            raise SafetyStop(f"motor FAULT, 0x2003 = {sdo_read(0x2003)}")
        time.sleep(0.05)
    raise RuntimeError(f"motor state {ms} != {state} after {timeout} s")


def record(test, expected, measured, ok, skipped=False):
    v = 'SKIPPED' if skipped else ('PASS' if ok else 'FAIL')
    results.append(dict(test=test, expected=expected, measured=measured, verdict=v))
    print(f"[{v}] {test}: expected {expected} | measured {measured}", flush=True)


def to_switch_on_disabled():
    sdo_write(0x6040, 0, 0x0000, 2)
    wait_sw(0x0250, 0.5)


def safe_state():
    for idx, sub, val, size in ((0x6040, 0, 0x0000, 2), (0x2011, 0, 2, 1), (0x1800, 2, 0xFE, 1)):
        for _ in range(3):
            try:
                sdo_write(idx, sub, val, size)
                break
            except Exception as e:
                print(f"  safe_state: 0x{idx:04X}:{sub} write failed: {e}")


# --- preflight ----------------------------------------------------------------

def preflight():
    # While the adapter channel was closed the node got no ACK: TX passive / TX overflow set the
    # communication bit of 0x1001 and NMT fell to pre-op. Wait until it is clear, then start the node.
    t = now()
    err = None
    while now() - t < 5.0:
        err = sdo_read(0x1001)
        if (err & 0x10) == 0:
            break
        time.sleep(0.1)
    print(f"0x1001 = {err:#04x} after {now() - t:.1f} s", flush=True)
    t = now()
    send(0x000, [0x01, 0x0D])                   # NMT start node 13
    time.sleep(1.2)
    hb = frames(HB, t + 0.05)
    nmt = hb[-1][1][0] if hb else None
    cfg = {k: sdo_read(i, s) for k, (i, s) in {
        '1400:01': (0x1400, 1), '1401:01': (0x1401, 1), '1600:00': (0x1600, 0), '1601:00': (0x1601, 0),
        '1800:01': (0x1800, 1), '1801:01': (0x1801, 1), '1A00:00': (0x1A00, 0), '1A01:00': (0x1A01, 0),
        '1800:02': (0x1800, 2), '1800:05': (0x1800, 5), '1005': (0x1005, 0), '1019': (0x1019, 0)}.items()}
    orig['6060'] = sdo_read(0x6060, signed=True)
    orig['2012'] = sdo_read(0x2012)
    st = dict(sw=hex(sdo_read(0x6041)), motor_state=sdo_read(0x2002), fault=sdo_read(0x2003))
    print("preflight:", dict(nmt=nmt, **{k: hex(v) for k, v in cfg.items()}, **orig, **st), flush=True)
    ok = (nmt == 0x05 and cfg['1400:01'] == 0x20D and cfg['1401:01'] == 0x30D and (cfg['1800:01'] & ~0x40000000) == 0x18D
          and (cfg['1801:01'] & ~0x40000000) == 0x28D and cfg['1600:00'] == 2 and cfg['1601:00'] == 2
          and cfg['1A00:00'] == 2 and cfg['1A01:00'] == 3 and st['motor_state'] == ST_IDLE)
    record('pre', 'NMT 05, PDO COB-IDs/mapping of phase 6, motor IDLE',
           f"NMT {nmt}, cfg {{{', '.join(f'{k}={v:#x}' for k, v in cfg.items())}}}, {st}", ok)
    return ok


# --- D: SYNC ------------------------------------------------------------------

def sync_burst(n=20, period=0.05):
    t_from = now()
    times = []
    for _ in range(n):
        times.append(now())
        send(SYNC, [])
        time.sleep(period)
    time.sleep(0.1)
    f = tpdo1(t_from)
    delays = []
    for ts in times:
        after = [t for t, _, _ in f if ts <= t < ts + period]
        if after:
            delays.append((after[0] - ts) * 1000.0)
    return len(f), len(frames(TPDO2, t_from)), delays


def test_d():
    to_switch_on_disabled()
    sdo_write(0x1800, 2, 1, 1)
    rb = sdo_read(0x1800, 2)
    time.sleep(0.2)
    t = now()
    time.sleep(1.0)
    n1, n2 = len(frames(TPDO1, t)), len(frames(TPDO2, t))
    record('D1', '0x1800:02 = 1, no SYNC for 1 s: TPDO1 0 frames, TPDO2 ~10',
           f"read back {rb}, TPDO1 {n1}, TPDO2 {n2}", rb == 1 and n1 == 0 and 9 <= n2 <= 11)

    n1, n2, d = sync_burst()
    record('D2', '20 SYNC @ 50 ms, type 1: TPDO1 20 (one per SYNC), TPDO2 stays ~10/s (not SYNC)',
           f"TPDO1 {n1}, TPDO2 {n2}, TPDO1 delay after SYNC (PC side) "
           f"{min(d):.1f}..{max(d):.1f} ms" if d else f"TPDO1 {n1}, TPDO2 {n2}, no delays",
           19 <= n1 <= 20)

    sdo_write(0x1800, 2, 2, 1)
    n1, n2, d = sync_burst()
    record('D3', '20 SYNC, type 2: TPDO1 10 (every 2nd SYNC)', f"TPDO1 {n1}", 9 <= n1 <= 10)

    sdo_write(0x1800, 2, 0xFE, 1)
    time.sleep(0.2)
    t = now()
    time.sleep(1.0)
    n1 = len(frames(TPDO1, t))
    record('D4', 'back to 0xFE: TPDO1 ~10 in 1 s', f"TPDO1 {n1}", 9 <= n1 <= 11)


def coast(hall):
    """Wait until the rotor stops after a shutdown (PWM off, it coasts)."""
    t0 = now()
    if not hall:                                # sensorless speed is not valid when stopped
        time.sleep(10.0)
        return now() - t0
    while now() - t0 < 20.0:
        sp = [abs(rpm) for _, _, rpm in tpdo1(now() - 0.5)]
        if len(sp) >= 4 and max(sp) < 20:
            break
        time.sleep(0.1)
    return now() - t0


# --- A / B: profile velocity over RPDO1 ----------------------------------------

def test_pv(name, sensor, v1, v2):
    t_from = now()
    to_switch_on_disabled()
    sdo_write(0x6060, 0, 3, 1)
    sdo_write(0x2012, 0, sensor, 1)
    rpdo1(0x06, v1)
    wait_sw(0x0231)
    rpdo1(0x07, v1)
    wait_sw(0x0233)
    rpdo1(0x0F, v1)
    wait_sw(0x0027, mask=0x006F)
    try:
        t_cl = wait_state(ST_CL, 10.0, t_from)
        t = now()
        sw = None
        while now() - t < 6.0:                  # wait for bit 10 at v1
            guard(t_from, True)
            sw = sdo_read(0x6041)
            if sw & 0x0400:
                break
            time.sleep(0.02)
        time.sleep(0.6)
        sp = [rpm for _, _, rpm in tpdo1(now() - 0.6)]
        v_before = round(sum(sp) / len(sp)) if sp else None
        record(f'{name}1', f'06/07/0F via RPDO1, CLOSED_LOOP, sw 0x0637, mean speed {v1} +-30 rpm',
               f"CLOSED_LOOP after {t_cl:.1f} s, sw {sw:#06x}, speed samples {sp} (mean {v_before})",
               sw == 0x0637 and v_before is not None and abs(v_before - v1) <= 30)

        t_c = now()
        rpdo1(0x0F, v2)                         # only the target changes, controlword stays 0x0F
        sws = []
        reached = None
        while now() - t_c < 4.0:
            guard(t_from, True)
            sw = sdo_read(0x6041)
            if not sws or sws[-1][1] != sw:
                sws.append((round((now() - t_c) * 1000.0), sw))
            if reached is None and sws and sws[-1][1] == 0x0637 and len(sws) > 1:
                reached = now() - t_c
            time.sleep(0.002)
        sp = [rpm for _, _, rpm in tpdo1(t_c)]
        left_oe = [hex(s) for _, s in sws if (s & 0x006F) != 0x0027]
        dip = min(sp) < v_before - 50 if sp else None
        record(f'{name}2', f'target {v1} -> {v2} (cw stays 0x0F): sw never leaves OE, no speed dip, 0x0637 again',
               f"sw changes (ms, sw) {[(t, hex(s)) for t, s in sws]}, left OE {left_oe or 'never'}, "
               f"TPDO1 speed {sp[0] if sp else None} -> {sp[-1] if sp else None} rpm (min {min(sp) if sp else None}), "
               f"0x0637 again after {reached and f'{reached:.2f} s'}",
               not left_oe and dip is False and reached is not None and abs(sp[-1] - v2) <= 30)
    finally:
        rpdo1(0x06, 0)
    wait_sw(0x0231, 0.5)
    t_stop = wait_state(ST_IDLE, 8.0, t_from)
    print(f"   {name}: stopped, IDLE after {t_stop:.1f} s, rotor still after {coast(sensor == 1):.1f} s", flush=True)


# --- C: profile torque over RPDO2 ----------------------------------------------

def test_c(permille=80, duration=0.45):
    t_from = now()
    to_switch_on_disabled()
    sdo_write(0x6060, 0, 4, 1)
    rpdo2(0x06, 0)
    wait_sw(0x0231)
    rpdo2(0x07, 0)
    wait_sw(0x0233)
    t_on = now()
    rpdo2(0x0F, permille)
    try:
        wait_sw(0x0027, mask=0x006F)
        sws = []
        tq_sdo = []
        while now() - t_on < duration:
            guard(t_from, True)
            sw = sdo_read(0x6041)
            if not sws or sws[-1][1] != sw:
                sws.append((round((now() - t_on) * 1000.0), sw))
            if now() - t_on >= TQ_SETTLED_S:
                tq_sdo.append(sdo_read(0x6077, signed=True))
            time.sleep(0.01)
    finally:
        rpdo2(0x06, 0)
    t_off = now()
    wait_sw(0x0231, 0.5)
    f2 = tpdo2(t_on + 0.3, t_off)               # skip start-up and torque ramp
    f1 = tpdo1(t_on, t_off)
    tq = [q for _, _, q, _ in f2]
    modes = sorted({m for _, _, _, m in f2})
    sp = [rpm for _, _, rpm in f1]
    mean_tq = sum(tq_sdo) / len(tq_sdo) if tq_sdo else None
    record('C1', f'RPDO2 06/07/0F + {permille} permille: sw 0x0637, mode display 4, SDO 0x6077 mean ~{permille}, '
                 'TPDO2 torque > 0, motor turns',
           f"sw changes (ms, sw) {[(t, hex(s)) for t, s in sws]}, mode {modes}, SDO 0x6077 {tq_sdo} "
           f"(mean {mean_tq and round(mean_tq, 1)}), TPDO2 torque {tq}, TPDO1 speed {sp}",
           any(s == 0x0637 for _, s in sws) and modes == [4] and mean_tq is not None
           and abs(mean_tq - permille) <= 15 and any(q > 0 for q in tq) and sp and max(abs(v) for v in sp) > 50)
    t_stop = wait_state(ST_IDLE, 10.0, t_from)
    print(f"   C: stopped, IDLE after {t_stop:.1f} s, rotor still after {coast(True):.1f} s", flush=True)


# --- main ---------------------------------------------------------------------

def main():
    global bus
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default='D,A,B,C')
    ap.add_argument('--log', default=str(Path(__file__).parent / 'logs'))
    # Break-away depends on where the rotor rests: 60 moved it from two places, one place needed 80
    # (70 did not). At 80 the unloaded motor gains ~6300 rpm/s, so 0.45 s stays below SPEED_ABORT_RPM.
    ap.add_argument('--tq', type=int, default=80, help='C: target torque [permille]')
    ap.add_argument('--tq-time', type=float, default=0.45, help='C: run time [s]')
    ap.add_argument('--motor-ok', action='store_true', help='allow A/B/C (motor rotates)')
    args = ap.parse_args()
    tests = [t.strip().upper() for t in args.tests.split(',')]

    bus = can.Bus(interface='slcan', channel=PORT, bitrate=500000)
    threading.Thread(target=_rx_thread, daemon=True).start()
    try:
        if preflight():
            for t in tests:
                if t in ('A', 'B', 'C') and not args.motor_ok:
                    record(t, 'motor test', 'not run: --motor-ok missing', False, skipped=True)
                    continue
                try:
                    if t == 'D':
                        test_d()
                    elif t == 'A':
                        test_pv('A', 0, 1000, 1500)
                    elif t == 'B':
                        test_pv('B', 1, 1000, 1500)
                    elif t == 'C':
                        test_c(args.tq, args.tq_time)
                except SafetyStop:
                    raise
                except Exception as e:
                    safe_state()
                    record(t, '-', f"exception: {e!r}", False)
                    time.sleep(1.0)
    except (SafetyStop, KeyboardInterrupt) as e:
        print(f"\n!!! STOPPED: {e!r}", flush=True)
        results.append(dict(test='STOP', expected='-', measured=repr(e), verdict='FAIL'))
    finally:
        safe_state()
        for idx, key in ((0x6060, '6060'), (0x2012, '2012')):
            if key in orig:
                try:
                    sdo_write(idx, 0, orig[key], 1)
                except Exception as e:
                    print(f"  restore 0x{idx:04X} failed: {e}")
        print("safe state: 0x6040=0, 0x2011=2, 0x1800:02=0xFE, 0x6060/0x2012 restored", flush=True)
        _stop_rx.set()
        time.sleep(0.1)
        bus.shutdown()
        print("\n=== RESULTS ===")
        for r in results:
            print(f"{r['test']:4} | {r['verdict']:4} | {r['expected']} | {r['measured']}")
        out = Path(args.log)
        out.mkdir(exist_ok=True)
        f = out / time.strftime('pdo_%Y%m%d_%H%M%S.json')
        f.write_text(json.dumps(dict(results=results, frames=[f"{t - T0:9.4f} 0x{i:03X} {d.hex(' ')}"
                                                               for t, i, d in log]), indent=1))
        print(f"log: {f}")


if __name__ == '__main__':
    main()
