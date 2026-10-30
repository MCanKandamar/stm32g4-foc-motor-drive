"""CiA 402 hardware-in-the-loop tests (9e) for the BLDC drive, node-ID 13.

Usage (venv with python-can and scrutinydebugger 0.16):
    python tools/hil/test_cia402_v2.py --tests T1,T9,T2,T3,T7,T8
    python tools/hil/test_cia402_v2.py --tests T5,T6 --motor-ok

T4 runs inside T3 (it needs the 0x07 of T3). T5/T6 rotate the motor and only run
with --motor-ok. On every exit the script writes 0x6040 = 0 and 0x2011 = 2.
"""
import argparse
import json
import queue
import threading
import time
from pathlib import Path

import sys

sys.path.insert(0, 'C:/Program Files (x86)/Scrutiny Debugger/scrutinydebugger-nocli-0.16.0-py3-none-any.whl')

import can

PORT = 'COM15'
SDO_TX, SDO_RX, EMCY, HB = 0x60D, 0x58D, 0x08D, 0x70D
SPEED_REF_RPM = 300          # rule: <= 300 rpm
RUN_TIME_S = 8.0             # rule: <= 8 s per test
SPEED_ABORT_RPM = 600        # safety: unexpected speed -> stop at once

bus = None
rxq = queue.Queue()          # (t, id, bytes) from the RX thread
log = []                     # every frame received, (t, id, bytes)
_stop_rx = threading.Event()
T_START = time.perf_counter()


class SafetyStop(Exception):
    pass


def _rx_thread():
    while not _stop_rx.is_set():
        m = bus.recv(timeout=0.05)
        if m is None:
            continue
        rec = (time.perf_counter(), m.arbitration_id, bytes(m.data))
        log.append(rec)
        rxq.put(rec)


def _drain():
    while True:
        try:
            rxq.get_nowait()
        except queue.Empty:
            return


def _wait_sdo(index, sub, timeout=0.5):
    t_end = time.perf_counter() + timeout
    while time.perf_counter() < t_end:
        try:
            t, cid, data = rxq.get(timeout=0.01)
        except queue.Empty:
            continue
        # answer must match the request (index + sub), otherwise it is a stale one
        if cid == SDO_RX and data[1] == (index & 0xFF) and data[2] == (index >> 8) and data[3] == sub:
            return data
    raise TimeoutError(f"no SDO answer for 0x{index:04X}:{sub}")


def sdo_write(index, sub, value, size):          # size: 1, 2 or 4 bytes
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
    data = [cmd, index & 0xFF, index >> 8, sub] + list((value & 0xFFFFFFFF).to_bytes(4, 'little'))
    _drain()
    bus.send(can.Message(arbitration_id=SDO_TX, data=data, is_extended_id=False))
    r = _wait_sdo(index, sub)
    if r[0] == 0x80:
        return ('abort', int.from_bytes(r[4:8], 'little'))
    return ('ok', None)


def sdo_read(index, sub=0, signed=False):
    data = [0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0]
    _drain()
    bus.send(can.Message(arbitration_id=SDO_TX, data=data, is_extended_id=False))
    r = _wait_sdo(index, sub)
    if r[0] == 0x80:
        return ('abort', int.from_bytes(r[4:8], 'little'))
    n = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(r[0], 4)
    return ('ok', int.from_bytes(r[4:4 + n], 'little', signed=signed))


def rd(index, sub=0, signed=False):
    st, v = sdo_read(index, sub, signed)
    return v if st == 'ok' else f"abort 0x{v:08X}"


def cw(value):                                    # write controlword, return start time
    t0 = time.perf_counter()
    sdo_write(0x6040, 0, value, 2)
    return t0


def wait_sw(expected, t0, timeout=0.2):           # poll 0x6041 every 2 ms
    while time.perf_counter() - t0 < timeout:
        st, sw = sdo_read(0x6041)
        if sw == expected:
            return (time.perf_counter() - t0) * 1000.0
        time.sleep(0.002)
    return None


def listen(duration):
    time.sleep(duration)


def frames(id_, t_from, t_to=None):
    t_to = t_to if t_to is not None else time.perf_counter()
    return [(t, d) for (t, i, d) in list(log) if i == id_ and t_from <= t <= t_to]


def fmt_frames(t_from, t_to=None):
    t_to = t_to if t_to is not None else time.perf_counter()
    return [f"{t - T_START:9.4f}  0x{i:03X}  {d.hex(' ')}" for (t, i, d) in list(log) if t_from <= t <= t_to]


def safe_state():
    for idx, val, size in ((0x6040, 0x0000, 2), (0x2011, 2, 1)):
        for _ in range(3):
            try:
                sdo_write(idx, 0, val, size)
                break
            except Exception as e:                # keep trying, then report
                print(f"  safe_state: 0x{idx:04X} write failed: {e}")


# --- Scrutiny ---------------------------------------------------------------

class Scr:
    def __init__(self):
        self.client = None
        self.paths = {}
        self.handles = {}

    def connect(self):
        from scrutiny.sdk.client import ScrutinyClient
        self.client = ScrutinyClient()
        self.client.connect('localhost', 8765)
        self.client.wait_device_ready(5)
        req = self.client.download_watchable_list()
        req.wait_for_completion(30)
        d = req.get()
        allp = list(d.var) + list(d.alias) + list(d.rpv)
        for f in d.var_factory.values():          # arrays: one path per element
            allp += [p for p, _ in f.iterate_possible_paths()]
        print("Scrutiny paths matching amplifier_offset_raw / offset_valid / fault / cmd:")
        for p in sorted(allp):
            if any(k in p for k in ('amplifier_offset_raw', 'offset_valid', 'fault', 'cmd')):
                print("   ", p)

        def pick(pred, name):
            hits = [p for p in allp if pred(p)]
            if len(hits) != 1:
                raise RuntimeError(f"Scrutiny path for {name}: {len(hits)} matches {hits}")
            return hits[0]

        last = lambda p: p.rsplit('/', 1)[-1]
        for i in range(3):
            self.paths[f'off{i}'] = pick(lambda p, i=i: last(p) == f'amplifier_offset_raw[{i}]', f'amplifier_offset_raw[{i}]')
        self.paths['offset_valid'] = pick(lambda p: last(p) == 'offset_valid', 'offset_valid')
        self.paths['fault'] = pick(lambda p: p == '/global/fault', 'fault')
        for k, p in self.paths.items():
            self.handles[k] = self.client.watch(p)
        print("Scrutiny using:", json.dumps(self.paths, indent=1))

    def read_fresh(self, key, timeout=1.0):
        h = self.handles[key]
        h.wait_update(timeout, previous_counter=h.update_counter)
        return h.value

    def write(self, key, value):
        self.handles[key].value = value

    def device_ok(self):
        try:
            self.client.wait_server_status_update(1.0)
        except Exception:
            pass
        st = self.client.get_latest_server_status()
        return str(st.device_comm_state)


scr = None
results = []          # dict(test, expected, measured, verdict)
raw = {}              # test -> list of frame strings


def record(test, expected, measured, ok, t_from=None, skipped=False):
    verdict = 'SKIPPED' if skipped else ('PASS' if ok else 'FAIL')
    results.append(dict(test=test, expected=expected, measured=measured, verdict=verdict))
    print(f"[{verdict}] {test}: expected {expected} | measured {measured}")
    if verdict == 'FAIL' and t_from is not None:
        raw[test] = fmt_frames(t_from)


def hb_intervals(t_from, t_to=None):
    ts = [t for t, d in frames(HB, t_from, t_to)]
    return [(b - a) * 1000.0 for a, b in zip(ts, ts[1:])]


# --- tests ------------------------------------------------------------------

def wait_bootup(timeout=60.0):
    """Wait for the 0x70D boot-up frame (data 00). Returns its time or None."""
    t_end = time.perf_counter() + timeout
    while time.perf_counter() < t_end:
        try:
            t, cid, data = rxq.get(timeout=0.05)
        except queue.Empty:
            continue
        if cid == HB and data == b'\x00':
            return t
    return None


def t1():
    print("\n>>> T1: reset the board within 60 s (reset button)", flush=True)
    t_from = time.perf_counter() - 0.5
    _drain()
    tb = wait_bootup(60.0)
    if tb is None:
        record('T1', 'boot-up 0x70D 00 seen', 'no boot-up in 60 s', False, t_from)
        return
    seq = []
    t_ok = None
    while time.perf_counter() - tb < 1.0:
        sw = rd(0x6041)
        seq.append((round((time.perf_counter() - tb) * 1000, 1), sw))
        if sw == 0x0250:
            t_ok = (time.perf_counter() - tb) * 1000.0
            break
        time.sleep(0.002)
    listen(max(0.0, 3.0 - (time.perf_counter() - tb)))
    ms, ec = rd(0x2002), rd(0x603F)
    sws = ', '.join(f"{t}ms:{v:#06x}" if isinstance(v, int) else f"{t}ms:{v}" for t, v in seq[:6])
    meas = f"0x0250 after {t_ok:.1f} ms" if t_ok else "0x0250 not reached in 1 s"
    meas += f" (first reads {sws}); 0x2002={ms}; 0x603F={ec}"
    ok = t_ok is not None and t_ok <= 50.0 and ms == 1 and ec == 0
    record('T1', '0x6041=0x0250 <=50 ms after boot-up, 0x2002=1, 0x603F=0', meas, ok, t_from)


def t9(nmt_reset=False):
    t_from = time.perf_counter() - 0.5
    _drain()
    if nmt_reset:                                 # NMT Reset Node -> HAL_NVIC_SystemReset()
        print("\n>>> T9: NMT Reset Node (0x000: 81 0D)", flush=True)
        bus.send(can.Message(arbitration_id=0x000, data=[0x81, 0x0D], is_extended_id=False))
    else:
        print("\n>>> T9: reset the board AGAIN within 60 s (0x06 is sent right after boot-up)", flush=True)
    tb = wait_bootup(60.0)
    if tb is None:
        record('T9', 'boot-up seen, then 06 ignored in INIT', 'no boot-up in 60 s', False, t_from)
        return
    t_cw = cw(0x0006)
    seq = []
    while time.perf_counter() - t_cw < 0.5:
        seq.append((round((time.perf_counter() - tb) * 1000, 1), rd(0x6041), rd(0x2002)))
        time.sleep(0.005)
    cw(0x0000)
    first = seq[0]
    saw_0231 = any(s == 0x0231 for _, s, _ in seq)
    saw_0231_in_init = any(s == 0x0231 and m == 0 for _, s, m in seq)
    t231 = next((t for t, s, _ in seq if s == 0x0231), None)
    t_init_end = next((t for t, _, m in seq if m != 0), None)
    meas = (f"06 sent {(t_cw - tb) * 1000:.1f} ms after boot-up; first read {first[0]} ms: sw={first[1]:#06x} 0x2002={first[2]}; "
            f"INIT ended ~{t_init_end} ms; 0x0231 seen: {saw_0231} (at {t231} ms, during INIT: {saw_0231_in_init}); "
            f"last sw={seq[-1][1]:#06x}")
    record('T9', 'sw stays 0x0250 (never 0x0231) after 06 sent during INIT', meas, not saw_0231, t_from)
    raw['T9 sequence (ms_after_bootup, 0x6041, 0x2002)'] = [f"{t} {s:#06x} {m}" for t, s, m in seq]


def t2():
    t_from = time.perf_counter()
    hb = rd(0x1017)
    listen(5.0)
    iv = hb_intervals(t_from)
    if not isinstance(hb, int) or not iv:
        record('T2', 'max HB interval < 2 x 0x1017', f"0x1017={hb}, intervals={len(iv)}", False, t_from)
        return hb
    meas = f"0x1017={hb} ms; {len(iv)} intervals, min {min(iv):.1f} / avg {sum(iv) / len(iv):.1f} / max {max(iv):.1f} ms"
    record('T2', f'max interval < {2 * hb} ms', meas, max(iv) < 2 * hb, t_from)
    return hb


def step(value, expected, timeout=0.2):
    t0 = cw(value)
    return wait_sw(expected, t0, timeout)


def t3_t4(hb):
    t_from = time.perf_counter()
    cw(0x0000)
    if wait_sw(0x0250, time.perf_counter(), 0.3) is None:
        record('T3', 'start from 0x0250', f"sw={rd(0x6041)}", False, t_from)
        return []
    lat = {}
    lat['06'] = step(0x06, 0x0231)
    t07 = time.perf_counter()
    lat['07'] = step(0x07, 0x0233)
    # T4: 50 ms after 0x07
    if scr is not None:
        try:
            time.sleep(max(0.0, 0.05 - (time.perf_counter() - t07)))
            offs = [scr.read_fresh(f'off{i}') for i in range(3)]
            valid = scr.read_fresh('offset_valid')
            dev = scr.device_ok()
            ok4 = all(1900 <= o <= 2200 for o in offs) and int(valid) == 1 and dev.endswith('ConnectedReady')
            record('T4', 'offsets[0..2] in 1900..2200, offset_valid=1, link kept',
                   f"offsets={offs}, offset_valid={valid}, device={dev}", ok4, t07)
        except Exception as e:
            record('T4', 'offsets in 1900..2200, offset_valid=1', f"error: {e}", False, t07)
    else:
        record('T4', 'Scrutiny offsets', 'Scrutiny not available', False, skipped=True)
    # heartbeat must keep running while in Switched On
    listen(max(0.5, 2.5 * hb / 1000.0) if isinstance(hb, int) else 0.5)
    t_hb_end = time.perf_counter()
    lat['06b'] = step(0x06, 0x0231)
    lat['00'] = step(0x00, 0x0250)
    iv = hb_intervals(t07 - (hb / 1000.0 if isinstance(hb, int) else 1.0), t_hb_end)
    hb_ok = bool(iv) and isinstance(hb, int) and max(iv) < 2 * hb
    lat_ok = all(v is not None and v < 10.0 for v in lat.values())
    meas = ', '.join(f"{k}: {'timeout' if v is None else f'{v:.1f} ms'}" for k, v in lat.items())
    meas += f"; HB intervals around 07: {[round(x, 1) for x in iv]}"
    record('T3', 'each step < 10 ms; HB interval < 2 x 0x1017 during 07', meas, lat_ok and hb_ok, t_from)
    return [v for v in lat.values() if v is not None]


def t7():
    if scr is None:
        record('T7', 'fault via Scrutiny', 'Scrutiny not available', False, skipped=True)
        return
    t_from = time.perf_counter()
    cw(0x00)
    wait_sw(0x0250, time.perf_counter(), 0.3)
    a = step(0x06, 0x0231)
    b = step(0x07, 0x0233)
    t_f = time.perf_counter()
    scr.write('fault', 5)
    listen(0.2)
    sw, ec, er, ms = rd(0x6041), rd(0x603F), rd(0x1001), rd(0x2002)
    em = [d.hex(' ') for t, d in frames(EMCY, t_f)]
    want_em = '20 32 81 45 00 00 00 00'
    ok = (a is not None and b is not None and sw == 0x0218 and ec == 0x3220 and er == 0x81 and ms == 5 and want_em in em)
    meas = (f"sw={sw:#06x} 0x603F={ec:#06x} 0x1001={er:#04x} 0x2002={ms} EMCY={em}"
            if all(isinstance(x, int) for x in (sw, ec, er)) else f"sw={sw} 0x603F={ec} 0x1001={er} 0x2002={ms} EMCY={em}")
    record('T7', f'sw=0x0218, 0x603F=0x3220, 0x1001=0x81, 0x2002=5, EMCY {want_em}', meas, ok, t_from)


def t8():
    if scr is None:
        record('T8', 'fault reset via cw bit 7', 'Scrutiny not available', False, skipped=True)
        return
    # a) rising edge 0x07 -> 0x80 resets the fault
    t_from = time.perf_counter()
    cw(0x0080)
    listen(0.2)
    sw, ec = rd(0x6041), rd(0x603F)
    em = [d.hex(' ') for t, d in frames(EMCY, t_from)]
    want_em = '00 00 00 45 00 00 00 00'
    ok = sw == 0x0250 and ec == 0 and want_em in em
    record('T8a', f'cw 0x80 edge: sw=0x0250, 0x603F=0, EMCY {want_em}', f"sw={sw:#06x} 0x603F={ec} EMCY={em}"
           if isinstance(sw, int) else f"sw={sw} 0x603F={ec} EMCY={em}", ok, t_from)
    # b) cw stays 0x80, new fault: no edge, no reset
    t_b = time.perf_counter()
    scr.write('fault', 5)
    listen(0.2)
    sw = rd(0x6041)
    record('T8b', 'cw held 0x80 + fault=5: sw stays 0x0218', f"sw={sw:#06x}" if isinstance(sw, int) else f"sw={sw}",
           sw == 0x0218, t_b)
    # c) 00 -> 80 edge resets again
    t_c = time.perf_counter()
    cw(0x0000)
    cw(0x0080)
    listen(0.2)
    sw = rd(0x6041)
    record('T8c', '00 -> 80: sw=0x0250', f"sw={sw:#06x}" if isinstance(sw, int) else f"sw={sw}", sw == 0x0250, t_c)
    cw(0x0000)


def run_and_sample(test, duration):
    """Sample 0x2001/0x2002 every 0.5 s for `duration` s. Stops on unexpected speed."""
    samples = []
    t0 = time.perf_counter()
    k = 1
    while True:
        t_next = t0 + 0.5 * k
        if t_next - t0 > duration + 1e-6:
            break
        time.sleep(max(0.0, t_next - time.perf_counter()))
        sp, ms = rd(0x2001, signed=True), rd(0x2002)
        samples.append((round(time.perf_counter() - t0, 2), sp, ms))
        print(f"   {test} t={samples[-1][0]:4.1f} s  speed={sp} rpm  state={ms}")
        if isinstance(sp, int) and abs(sp) > SPEED_ABORT_RPM:
            raise SafetyStop(f"{test}: speed {sp} rpm > {SPEED_ABORT_RPM} rpm")
        k += 1
    return samples


def states_in_order(samples):
    seq = []
    for _, _, m in samples:
        if not seq or seq[-1] != m:
            seq.append(m)
    return seq


def t5():
    t_from = time.perf_counter()
    assert SPEED_REF_RPM <= 300
    sdo_write(0x2010, 0, SPEED_REF_RPM, 4)
    cw(0x00)
    wait_sw(0x0250, time.perf_counter(), 0.3)
    lat = {'06': step(0x06, 0x0231), '07': step(0x07, 0x0233), '0F': step(0x0F, 0x0237)}
    samples = run_and_sample('T5', RUN_TIME_S)
    lat['07 stop'] = step(0x07, 0x0233)
    seq = states_in_order(samples)
    last2 = [sp for t, sp, _ in samples if t >= RUN_TIME_S - 2.0 - 0.01]
    order_ok = [s for s in seq if s in (2, 3, 4)] == [2, 3, 4]
    speed_ok = bool(last2) and all(isinstance(s, int) and 250 <= s <= 350 for s in last2)
    lat_ok = all(v is not None and v < 10.0 for v in lat.values())
    meas = (', '.join(f"{k}: {'timeout' if v is None else f'{v:.1f} ms'}" for k, v in lat.items())
            + f"; states {seq}; last 2 s speed {last2}")
    record('T5', '06/07/0F < 10 ms; 0x2002 2->3->4; last 2 s 250..350 rpm', meas, lat_ok and order_ok and speed_ok, t_from)
    raw['T5 samples (t_s, rpm, state)'] = [str(s) for s in samples]
    return [v for v in lat.values() if v is not None]


def t6():
    t_from = time.perf_counter()
    t0 = cw(0x0F)
    sdo_ms = (time.perf_counter() - t0) * 1000.0
    l0f = wait_sw(0x0237, t0)
    samples = run_and_sample('T6', RUN_TIME_S)
    l00 = step(0x00, 0x0250)
    ok = l0f is not None and l0f < 10.0 and sdo_ms < 10.0 and l00 is not None
    meas = (f"0F SDO answer {sdo_ms:.1f} ms, 0x0237 after {'timeout' if l0f is None else f'{l0f:.1f} ms'}; "
            f"00 -> 0x0250 {'timeout' if l00 is None else f'{l00:.1f} ms'}; states {states_in_order(samples)}; "
            f"last speed {samples[-1][1] if samples else None}")
    record('T6', '0F -> 0x0237 < 10 ms (no lock-up); 8 s later 00 -> 0x0250', meas, ok, t_from)
    raw['T6 samples (t_s, rpm, state)'] = [str(s) for s in samples]
    return [v for v in (l0f, l00) if v is not None]


# --- main -------------------------------------------------------------------

def main():
    global bus, scr
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default='T1,T9,T2,T3,T7,T8')
    ap.add_argument('--motor-ok', action='store_true', help='allow T5/T6 (motor rotates)')
    ap.add_argument('--no-scrutiny', action='store_true')
    ap.add_argument('--nmt-reset', action='store_true', help='T9: reset the board with NMT Reset Node')
    args = ap.parse_args()
    tests = [t.strip().upper() for t in args.tests.split(',')]

    bus = can.Bus(interface='slcan', channel=PORT, bitrate=500000)
    rx = threading.Thread(target=_rx_thread, daemon=True)
    rx.start()
    latencies = []
    hb = None
    try:
        if not args.no_scrutiny and any(t in tests for t in ('T3', 'T7', 'T8')):
            try:
                scr = Scr()
                scr.connect()
            except Exception as e:
                print(f"Scrutiny not available: {e}")
                scr = None
        for t in tests:
            try:
                if t == 'T1':
                    t1()
                elif t == 'T9':
                    t9(args.nmt_reset)
                elif t == 'T2':
                    hb = t2()
                elif t == 'T3':
                    if hb is None:
                        hb = rd(0x1017)
                    latencies += t3_t4(hb)
                elif t == 'T7':
                    t7()
                elif t == 'T8':
                    t8()
                elif t in ('T5', 'T6'):
                    if not args.motor_ok:
                        record(t, 'motor test', 'not run: --motor-ok missing', False, skipped=True)
                        continue
                    latencies += t5() if t == 'T5' else t6()
            except SafetyStop:
                raise
            except Exception as e:
                safe_state()
                record(t, '-', f"exception: {e!r}", False, time.perf_counter() - 1.0)
    except (SafetyStop, KeyboardInterrupt) as e:
        print(f"\n!!! STOPPED: {e!r}")
        results.append(dict(test='STOP', expected='-', measured=repr(e), verdict='FAIL'))
    finally:
        safe_state()
        print("safe state written: 0x6040=0x0000, 0x2011=2")
        _stop_rx.set()
        time.sleep(0.1)
        if scr is not None and scr.client is not None:
            try:
                scr.client.disconnect()
            except Exception:
                pass
        bus.shutdown()

        print("\n=== RESULTS ===")
        for r in results:
            print(f"{r['test']:5} | {r['verdict']:7} | {r['expected']} | {r['measured']}")
        if latencies:
            print(f"max latency: {max(latencies):.1f} ms")
        out = Path(__file__).parent / 'logs'
        out.mkdir(exist_ok=True)
        f = out / time.strftime('cia402_%Y%m%d_%H%M%S.json')
        f.write_text(json.dumps(dict(results=results, raw=raw, max_latency_ms=max(latencies) if latencies else None,
                                     all_frames=fmt_frames(T_START)), indent=1))
        print(f"log: {f}")


if __name__ == '__main__':
    main()
