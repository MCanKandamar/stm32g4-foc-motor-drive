"""CANopen phase 7 hardware test: heartbeat consumer 0x1016, node-ID 13.

The PC plays a master with node-ID 1: it sends 0x701 [05] every 100 ms.
Usage (python-can, SLCAN adapter on test_pdo.PORT):
    python tools/hil/test_heartbeat.py
    python tools/hil/test_heartbeat.py --tests H,M,O --motor-ok

H: motor off, config / timeout / recovery / master reset
M: sensorless pv at 1000 rpm, the master goes silent, 0x6007 = 1 must stop the motor (--motor-ok only)
O: 0x6007 write guard, then values 2, 3 and 0 with NMT pre-op at 1000 rpm (--motor-ok only)
On every exit: 0x1016:01 = 0, 0x6007 = 1, NMT start, 0x6040 = 0, 0x2011 = 2, 0x6060/0x2012 restored.
0x1016 stays in RAM: a real master must write it after each boot of the drive.
Log: tools/hil/logs/heartbeat_<date>_<time>.json
"""
import argparse
import json
import threading
import time
from pathlib import Path

import can
import test_pdo as t

MASTER_HB = 0x701
HB_PERIOD_S = 0.1
CONSUMER_MS = 500
ENTRY = (1 << 16) | CONSUMER_MS              # node-ID in bits 16..23, time in bits 0..15
EMC_HEARTBEAT = 0x8130
ERR_TIMEOUT, ERR_REMOTE_RESET = 0x1B, 0x1C   # byte 3 of the EMCY frame
NMT_OPERATIONAL, NMT_PRE_OP = 0x05, 0x7F
ERR_REG_COMM = 0x10
FAULT_COMM_LOST = 10
SW_FAULT = 0x0218
PV_RPM = 1000


class Master:
    """Heartbeat of the simulated master. start() and stop() can be called again."""

    def __init__(self):
        self.stop_flag = threading.Event()
        self.thread = None
        self.last_sent = None

    def _run(self):
        while not self.stop_flag.is_set():
            t.send(MASTER_HB, [NMT_OPERATIONAL])
            self.last_sent = t.now()
            self.stop_flag.wait(HB_PERIOD_S)

    def start(self):
        if self.thread is None:
            self.stop_flag.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def stop(self):
        if self.thread is not None:
            self.stop_flag.set()
            self.thread.join()
            self.thread = None
        return self.last_sent


def nmt_state():
    hb = t.frames(t.HB, 0.0)                 # the node also sends one on every state change
    return hb[-1][1][0] if hb else None


def wait_nmt(state, timeout=1.5):
    t_end = t.now() + timeout
    while t.now() < t_end and nmt_state() != state:
        time.sleep(0.005)
    return nmt_state() == state


def hb_emcys(t_from, bit):
    return [tm for tm, d in t.frames(t.EMCY, t_from)
            if int.from_bytes(d[0:2], 'little') == EMC_HEARTBEAT and d[3] == bit]


def wait_emcy(t_from, bit, timeout=1.0):
    t_end = t.now() + timeout
    while t.now() < t_end:
        found = hb_emcys(t_from, bit)
        if found:
            return found[0]
        time.sleep(0.005)
    return None


def wait_comm_error_clear(timeout=1.0):
    t_end = t.now() + timeout
    reg = None
    while t.now() < t_end:
        reg = t.sdo_read(0x1001)
        if (reg & ERR_REG_COMM) == 0:
            break
        time.sleep(0.02)
    return reg


def nmt_start():
    t.send(0x000, [0x01, 0x0D])
    return wait_nmt(NMT_OPERATIONAL)


# --- H: motor off -------------------------------------------------------------

def test_config():
    count = t.sdo_read(0x1016, 0)
    t.sdo_write(0x1016, 1, ENTRY, 4)
    back = t.sdo_read(0x1016, 1)
    t.record('H1', f'0x1016:00 = 8, write and read back 0x1016:01 = {ENTRY:#010x}',
             f"0x1016:00 = {count}, read back {back:#010x}", count == 8 and back == ENTRY)

    abort = None
    try:
        t.sdo_write(0x1016, 2, ENTRY, 4)
    except RuntimeError as e:
        abort = str(e)
    if abort is None:
        t.sdo_write(0x1016, 2, 0, 4)
    t.record('H2', 'same node in a second entry: SDO abort 0x06040043', f"{abort}",
             abort is not None and '0x06040043' in abort)


def test_no_master():
    t_from = t.now()
    time.sleep(1.2 * CONSUMER_MS / 1000.0)
    bad = hb_emcys(t_from, ERR_TIMEOUT)
    t.record('H3', 'no heartbeat seen yet: no monitoring, no EMCY 0x8130, NMT 05',
             f"EMCY {len(bad)}, NMT {nmt_state():#04x}", not bad and nmt_state() == NMT_OPERATIONAL)


def test_master_alive(master):
    t_from = t.now()
    master.start()
    time.sleep(1.5)
    bad = hb_emcys(t_from, ERR_TIMEOUT)
    t.record('H4', 'heartbeat every 100 ms for 1.5 s: no EMCY 0x8130, NMT 05',
             f"EMCY {len(bad)}, NMT {nmt_state():#04x}", not bad and nmt_state() == NMT_OPERATIONAL)


def test_timeout(master):
    t_from = t.now()
    last = master.stop()
    t_emcy = wait_emcy(t_from, ERR_TIMEOUT)
    pre_op = wait_nmt(NMT_PRE_OP)
    reg = t.sdo_read(0x1001)
    fault = t.sdo_read(0x2003)
    delay_ms = None if t_emcy is None else round((t_emcy - last) * 1000.0)
    t.record('H5', f'silent master: EMCY 0x8130 bit 0x1B after {CONSUMER_MS} ms (450..650), NMT 7F, '
                   '0x1001 bit 4 set, no motor fault (the motor is off)',
             f"delay {delay_ms} ms, NMT {nmt_state():#04x}, 0x1001 = {reg:#04x}, 0x2003 = {fault}",
             delay_ms is not None and 450 <= delay_ms <= 650 and pre_op and (reg & ERR_REG_COMM) != 0 and fault == 0)


def test_recovery(master):
    master.start()
    reg = wait_comm_error_clear()
    time.sleep(0.2)
    stays = nmt_state()
    started = nmt_start()
    t.record('H6', 'master back: 0x1001 bit 4 clears, NMT stays 7F, NMT start gives 05',
             f"0x1001 = {reg:#04x}, NMT before start {stays:#04x}, after {nmt_state():#04x}",
             (reg & ERR_REG_COMM) == 0 and stays == NMT_PRE_OP and started)


def test_master_reset(master):
    t_from = t.now()
    t.send(MASTER_HB, [0x00])                # boot-up frame: the master has restarted
    t_emcy = wait_emcy(t_from, ERR_REMOTE_RESET)
    pre_op = wait_nmt(NMT_PRE_OP)
    reg = wait_comm_error_clear()
    started = nmt_start()
    t.record('H7', 'boot-up from the master: EMCY 0x8130 bit 0x1C, NMT 7F, next heartbeat clears it, NMT start gives 05',
             f"EMCY {'seen' if t_emcy else 'none'}, NMT 7F {pre_op}, 0x1001 = {reg:#04x}, NMT {nmt_state():#04x}",
             t_emcy is not None and pre_op and (reg & ERR_REG_COMM) == 0 and started)


def test_h(master):
    t.to_switch_on_disabled()
    test_config()
    test_no_master()
    test_master_alive(master)
    test_timeout(master)
    test_recovery(master)
    test_master_reset(master)


# --- M: the motor runs when the master goes silent ------------------------------

def start_pv(t_from):
    t.sdo_write(0x6060, 0, 3, 1)
    t.sdo_write(0x2012, 0, 0, 1)
    t.rpdo1(0x06, PV_RPM)
    t.wait_sw(0x0231)
    t.rpdo1(0x07, PV_RPM)
    t.wait_sw(0x0233)
    t.rpdo1(0x0F, PV_RPM)
    t.wait_sw(0x0027, mask=0x006F)
    t.wait_state(t.ST_CL, 10.0, t_from)
    time.sleep(1.0)


def test_m(master):
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6007, 0, 1, 2)
    t.sdo_write(0x1016, 1, ENTRY, 4)
    master.start()
    time.sleep(0.3)
    nmt_start()                              # a failed H test can leave the node in pre-operational
    try:
        start_pv(t_from)

        t_stop = t.now()
        last = master.stop()
        t_emcy = wait_emcy(t_stop, ERR_TIMEOUT)
        sw = t.wait_sw(SW_FAULT, 1.0)
        t_fault = t.now()
        err_code, fault, state = t.sdo_read(0x603F), t.sdo_read(0x2003), t.sdo_read(0x2002)
        delay_ms = None if t_emcy is None else round((t_emcy - last) * 1000.0)
        t.record('M1', f'silent master with the motor at {PV_RPM} rpm: EMCY 0x8130 after ~{CONSUMER_MS} ms, '
                       'sw 0x0218, 0x603F = 0x8100, 0x2003 = 10, motor state FAULT',
                 f"delay {delay_ms} ms, sw {sw:#06x} {round((t_fault - last) * 1000.0)} ms after the last heartbeat, "
                 f"0x603F = {err_code:#06x}, 0x2003 = {fault}, state {state}",
                 delay_ms is not None and 450 <= delay_ms <= 650 and err_code == 0x8100
                 and fault == FAULT_COMM_LOST and state == t.ST_FAULT)
    finally:
        t.safe_state()                       # SDO still works in pre-operational, RPDO does not

    master.start()
    reg = wait_comm_error_clear()
    started = nmt_start()
    t.coast(False)
    for cw in (0x0000, 0x0080, 0x0000):
        t.sdo_write(0x6040, 0, cw, 2)
        time.sleep(0.05)
    t.wait_state(t.ST_IDLE, 8.0, t.now())
    t.record('M2', 'master back, NMT start, fault reset: motor IDLE, 0x2003 = 0',
             f"0x1001 = {reg:#04x}, NMT {nmt_state():#04x}, state {t.sdo_read(0x2002)}, 0x2003 = {t.sdo_read(0x2003)}",
             (reg & ERR_REG_COMM) == 0 and started and t.sdo_read(0x2002) == t.ST_IDLE and t.sdo_read(0x2003) == 0)


# --- O: the other 0x6007 reactions, NMT pre-op is the comm loss here ------------

def test_o():
    abort = None
    try:
        t.sdo_write(0x6007, 0, 7, 2)
    except RuntimeError as e:
        abort = str(e)
    back = t.sdo_read(0x6007, signed=True)
    t.record('O1', 'write 7 to 0x6007: SDO abort 0x06090030, value stays 1', f"{abort}, read back {back}",
             abort is not None and '0x06090030' in abort and back == 1)

    for name, option, expected, stops in (('O2', 2, 'disable voltage: sw 0x0250, motor IDLE, no fault', True),
                                          ('O3', 3, 'quick stop: ends at sw 0x0250, motor IDLE, no fault', True),
                                          ('O4', 0, 'no reaction: stays in OE, motor CLOSED_LOOP', False)):
        t_from = t.now()
        t.to_switch_on_disabled()
        t.sdo_write(0x6007, 0, option, 2)
        try:
            start_pv(t_from)
            t.send(0x000, [0x80, 0x0D])
            pre_op = wait_nmt(NMT_PRE_OP)
            if stops:
                sw = t.wait_sw(0x0250, 3.0)
            else:
                time.sleep(1.0)
                sw = t.sdo_read(0x6041)
            state, fault = t.sdo_read(0x2002), t.sdo_read(0x2003)
            ok = pre_op and fault == 0 and (state == t.ST_IDLE if stops else
                                            (state == t.ST_CL and (sw & 0x006F) == 0x0027))
            t.record(name, f'0x6007 = {option}, NMT pre-op at {PV_RPM} rpm: {expected}',
                     f"NMT {nmt_state():#04x}, sw {sw:#06x}, state {state}, 0x2003 = {fault}", ok)
        finally:
            t.safe_state()
            nmt_start()
        t.coast(False)


# --- main ---------------------------------------------------------------------

def clean_up(master):
    try:
        t.sdo_write(0x1016, 1, 0, 4)         # first: a stopped master must not cause a timeout now
        t.sdo_write(0x1016, 2, 0, 4)
        t.sdo_write(0x6007, 0, 1, 2)
    except Exception as e:
        print(f"  clean up: write failed: {e}")
    master.stop()
    t.send(0x000, [0x01, 0x0D])
    time.sleep(0.1)
    t.safe_state()
    for idx, key in ((0x6060, '6060'), (0x2012, '2012')):
        if key in t.orig:
            try:
                t.sdo_write(idx, 0, t.orig[key], 1)
            except Exception as e:
                print(f"  restore 0x{idx:04X} failed: {e}")
    print("safe state: 0x1016:01 = 0, NMT start, 0x6040 = 0, 0x2011 = 2, 0x6060/0x2012 restored", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default='H')
    ap.add_argument('--log', default=str(Path(__file__).parent / 'logs'))
    ap.add_argument('--motor-ok', action='store_true', help='allow M (motor rotates)')
    args = ap.parse_args()
    tests = [x.strip().upper() for x in args.tests.split(',')]

    master = Master()
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if t.preflight():
            for name in tests:
                if name in ('M', 'O') and not args.motor_ok:
                    t.record(name, 'motor test', 'not run: --motor-ok missing', False, skipped=True)
                    continue
                try:
                    if name == 'H':
                        test_h(master)
                    elif name == 'M':
                        test_m(master)
                    elif name == 'O':
                        test_o()
                except t.SafetyStop:
                    raise
                except Exception as e:
                    t.record(name, '-', f"exception: {e!r}", False)
                    t.safe_state()
                    master.stop()
                    time.sleep(1.0)
    except (t.SafetyStop, KeyboardInterrupt) as e:
        print(f"\n!!! STOPPED: {e!r}", flush=True)
        t.results.append(dict(test='STOP', expected='-', measured=repr(e), verdict='FAIL'))
    finally:
        clean_up(master)
        t._stop_rx.set()
        time.sleep(0.1)
        t.bus.shutdown()
        print("\n=== RESULTS ===")
        for r in t.results:
            print(f"{r['test']:4} | {r['verdict']:4} | {r['expected']} | {r['measured']}")
        out = Path(args.log)
        out.mkdir(exist_ok=True)
        f = out / time.strftime('heartbeat_%Y%m%d_%H%M%S.json')
        f.write_text(json.dumps(dict(results=t.results, frames=[f"{tm - t.T0:9.4f} 0x{i:03X} {d.hex(' ')}"
                                                                 for tm, i, d in t.log]), indent=1))
        print(f"log: {f}")


if __name__ == '__main__':
    main()
