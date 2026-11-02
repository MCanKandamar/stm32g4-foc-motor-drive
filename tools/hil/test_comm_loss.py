"""CANopen phase 7 hardware test: communication loss, node-ID 13.

The PC plays a master with node-ID 1: it sends 0x701 [05] every 100 ms.
Usage (python-can, SLCAN adapter on test_pdo.PORT):
    python tools/hil/test_comm_loss.py --tests H,R
    python tools/hil/test_comm_loss.py --tests B,C --motor-ok    (B and C need you at the bus)

H: heartbeat consumer 0x1016, motor off: config / timeout / recovery / master reset
M: sensorless pv at 1000 rpm, the master goes silent, 0x6007 = 1 must stop the motor (--motor-ok only)
O: 0x6007 write guard, then values 2, 3 and 0 with NMT pre-op at 1000 rpm (--motor-ok only)
R: RPDO timeout 0x1400:05 and 0x1401:05, motor off
S: sensorless pv at 1000 rpm, the RPDO stream stops, 0x6007 = 1 must stop the motor (--motor-ok only)
B: real bus-off with the motor off: the script waits, you short CAN_H to CAN_L for about 1 s
C: the same bus-off at 1000 rpm, 0x6007 = 1 must stop the motor (--motor-ok only)
Do not force bus-off with a wrong adapter bitrate: it worked once, then this adapter stopped reading USB twice.
On every exit: 0x1016:01 = 0, 0x1400:05 = 0, 0x1401:05 = 0, 0x6007 = 1, NMT start, 0x6040 = 0, 0x2011 = 2,
0x6060/0x2012 restored. These objects stay in RAM: a real master must write them after each boot of the drive.
Log: tools/hil/logs/comm_loss_<date>_<time>.json
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
RPDO_PERIOD_MS, RPDO_TIMEOUT_MS = 100, 300
EMC_HEARTBEAT, EMC_RPDO_TIMEOUT = 0x8130, 0x8250
ERR_TIMEOUT, ERR_REMOTE_RESET = 0x1B, 0x1C   # byte 3 of the EMCY frame
ERR_RPDO_TIMEOUT = 0x17
NMT_OPERATIONAL, NMT_PRE_OP = 0x05, 0x7F
ERR_REG_COMM = 0x10
FAULT_COMM_LOST = 10
SW_FAULT = 0x0218
PV_RPM = 1000
EMC_BUS_OFF, ERR_BUS_OFF = 0x8140, 0x12
SHORT_WAIT_S = 60


class Cyclic:
    """One frame sent at a fixed period. start() and stop() can be called again."""

    def __init__(self, cid, data, period_s):
        self.cid, self.data, self.period_s = cid, data, period_s
        self.stop_flag = threading.Event()
        self.thread = None
        self.last_sent = None

    def _run(self):
        while not self.stop_flag.is_set():
            t.send(self.cid, self.data)
            self.last_sent = t.now()
            self.stop_flag.wait(self.period_s)

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


def emcys(t_from, bit, code):
    return [tm for tm, d in t.frames(t.EMCY, t_from)
            if int.from_bytes(d[0:2], 'little') == code and d[3] == bit]


def hb_emcys(t_from, bit):
    return emcys(t_from, bit, EMC_HEARTBEAT)


def rpdo_emcys(t_from):
    return emcys(t_from, ERR_RPDO_TIMEOUT, EMC_RPDO_TIMEOUT)


def wait_emcy(t_from, bit, timeout=1.0, code=EMC_HEARTBEAT):
    t_end = t.now() + timeout
    while t.now() < t_end:
        found = emcys(t_from, bit, code)
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
    record_recovery('M2', 'master back')


def record_recovery(name, what):
    reg = wait_comm_error_clear()
    started = nmt_start()
    t.coast(False)
    for cw in (0x0000, 0x0080, 0x0000):
        t.sdo_write(0x6040, 0, cw, 2)
        time.sleep(0.05)
    t.wait_state(t.ST_IDLE, 8.0, t.now())
    t.record(name, f'{what}, NMT start, fault reset: motor IDLE, 0x2003 = 0',
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


# --- R: RPDO timeout, motor off -------------------------------------------------

def rpdo1_frame(cw, rpm):
    return cw.to_bytes(2, 'little') + (rpm & 0xFFFFFFFF).to_bytes(4, 'little')


def delay_ok(delay_ms):
    return delay_ms is not None and RPDO_TIMEOUT_MS - 30 <= delay_ms <= RPDO_TIMEOUT_MS + 150


def rpdo_delay_ms(t_from, last):
    t_emcy = wait_emcy(t_from, ERR_RPDO_TIMEOUT, code=EMC_RPDO_TIMEOUT)
    return None if t_emcy is None else round((t_emcy - last) * 1000.0)


def test_r():
    t.to_switch_on_disabled()
    t.sdo_write(0x1400, 5, RPDO_TIMEOUT_MS, 2)
    t.sdo_write(0x1401, 5, RPDO_TIMEOUT_MS, 2)
    back = t.sdo_read(0x1400, 5), t.sdo_read(0x1401, 5)
    t_from = t.now()
    time.sleep(2.0 * RPDO_TIMEOUT_MS / 1000.0)
    bad = rpdo_emcys(t_from)
    t.record('R1', f'0x1400:05 and 0x1401:05 = {RPDO_TIMEOUT_MS}; no RPDO seen yet: no EMCY 0x8250, NMT 05',
             f"read back {back}, EMCY {len(bad)}, NMT {nmt_state():#04x}",
             back == (RPDO_TIMEOUT_MS, RPDO_TIMEOUT_MS) and not bad and nmt_state() == NMT_OPERATIONAL)

    rpdo = Cyclic(t.RPDO1, rpdo1_frame(0x0000, 0), RPDO_PERIOD_MS / 1000.0)
    t_from = t.now()
    rpdo.start()
    time.sleep(1.5)
    bad = rpdo_emcys(t_from)
    t.record('R2', f'RPDO1 every {RPDO_PERIOD_MS} ms for 1.5 s: no EMCY 0x8250, NMT 05',
             f"EMCY {len(bad)}, NMT {nmt_state():#04x}", not bad and nmt_state() == NMT_OPERATIONAL)

    t_from = t.now()
    last = rpdo.stop()
    delay_ms = rpdo_delay_ms(t_from, last)
    pre_op = wait_nmt(NMT_PRE_OP)
    reg = wait_comm_error_clear()
    fault = t.sdo_read(0x2003)
    t.record('R3', f'RPDO1 stops: EMCY 0x8250 bit 0x17 after ~{RPDO_TIMEOUT_MS} ms, NMT 7F, '
                   'error clears in pre-op, no motor fault (the motor is off)',
             f"delay {delay_ms} ms, NMT {nmt_state():#04x}, 0x1001 = {reg:#04x}, 0x2003 = {fault}",
             delay_ok(delay_ms) and pre_op and (reg & ERR_REG_COMM) == 0 and fault == 0)

    started = nmt_start()
    t_from = t.now()
    time.sleep(1.0)
    bad = rpdo_emcys(t_from)
    t.record('R4', 'NMT start, no RPDO for 1 s: stays NMT 05, watching restarts at the next RPDO',
             f"EMCY {len(bad)}, NMT {nmt_state():#04x}", started and not bad and nmt_state() == NMT_OPERATIONAL)

    t_from = t.now()
    t.rpdo2(0x0000, 0)
    delay_ms = rpdo_delay_ms(t_from, t_from)
    pre_op = wait_nmt(NMT_PRE_OP)
    reg = wait_comm_error_clear()
    started = nmt_start()
    t.record('R5', f'one RPDO2 frame: EMCY 0x8250 bit 0x17 after ~{RPDO_TIMEOUT_MS} ms, NMT 7F, NMT start gives 05',
             f"delay {delay_ms} ms, NMT 7F {pre_op}, 0x1001 = {reg:#04x}, NMT {nmt_state():#04x}",
             delay_ok(delay_ms) and pre_op and (reg & ERR_REG_COMM) == 0 and started)
    t.sdo_write(0x1400, 5, 0, 2)
    t.sdo_write(0x1401, 5, 0, 2)


# --- S: the motor runs when the RPDO stream stops -------------------------------

def test_s():
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6007, 0, 1, 2)
    nmt_start()
    rpdo = Cyclic(t.RPDO1, rpdo1_frame(0x0F, PV_RPM), RPDO_PERIOD_MS / 1000.0)
    try:
        start_pv(t_from)
        rpdo.start()                         # the motor start is slower than the timeout, so arm it after the start
        time.sleep(0.3)
        t.sdo_write(0x1400, 5, RPDO_TIMEOUT_MS, 2)
        time.sleep(0.5)

        t_stop = t.now()
        last = rpdo.stop()
        delay_ms = rpdo_delay_ms(t_stop, last)
        sw = t.wait_sw(SW_FAULT, 1.0)
        t_fault = t.now()
        err_code, fault, state = t.sdo_read(0x603F), t.sdo_read(0x2003), t.sdo_read(0x2002)
        t.record('S1', f'RPDO1 stops with the motor at {PV_RPM} rpm: EMCY 0x8250 after ~{RPDO_TIMEOUT_MS} ms, '
                       'sw 0x0218, 0x603F = 0x8100, 0x2003 = 10, motor state FAULT',
                 f"delay {delay_ms} ms, sw {sw:#06x} {round((t_fault - last) * 1000.0)} ms after the last RPDO, "
                 f"0x603F = {err_code:#06x}, 0x2003 = {fault}, state {state}",
                 delay_ok(delay_ms) and err_code == 0x8100 and fault == FAULT_COMM_LOST and state == t.ST_FAULT)
    finally:
        rpdo.stop()
        t.safe_state()
        t.sdo_write(0x1400, 5, 0, 2)
    record_recovery('S2', 'RPDO timeout off')


# --- B, C: real bus-off, the operator shorts CAN_H to CAN_L ---------------------

def wait_bus_off():
    """No frame gets through during the short. The EMCY comes out after it, so wait for that."""
    t_from = t.now()
    print(f">>> short CAN_H to CAN_L for about 1 s, then remove it (waiting {SHORT_WAIT_S} s)", flush=True)
    seen = wait_emcy(t_from, ERR_BUS_OFF, timeout=SHORT_WAIT_S, code=EMC_BUS_OFF) is not None
    time.sleep(1.0)
    return seen


def test_b():
    t.to_switch_on_disabled()
    seen = wait_bus_off()
    reg = wait_comm_error_clear()
    fault = t.sdo_read(0x2003)
    t.record('B1', 'bus-off: EMCY 0x8140 bit 0x12, restarts by itself, error clears, NMT 7F, no motor fault',
             f"seen {seen}, 0x1001 = {reg:#04x}, NMT {nmt_state():#04x}, 0x2003 = {fault}",
             seen and (reg & ERR_REG_COMM) == 0 and nmt_state() == NMT_PRE_OP and fault == 0)

    started = nmt_start()
    t_tpdo = t.now()
    time.sleep(1.0)
    n = len(t.frames(t.TPDO1, t_tpdo))
    t.record('B2', 'NMT start gives 05, TPDO1 ~10 in 1 s', f"NMT {nmt_state():#04x}, TPDO1 {n}", started and 9 <= n <= 11)


def test_c():
    t_from = t.now()
    t.to_switch_on_disabled()
    t.sdo_write(0x6007, 0, 1, 2)
    nmt_start()
    try:
        start_pv(t_from)
        seen = wait_bus_off()
        sw, err_code = t.sdo_read(0x6041), t.sdo_read(0x603F)
        fault, state = t.sdo_read(0x2003), t.sdo_read(0x2002)
        t.record('C1', f'bus-off with the motor at {PV_RPM} rpm: EMCY 0x8140 bit 0x12, sw 0x0218, 0x603F = 0x8100, '
                       '0x2003 = 10, motor state FAULT',
                 f"seen {seen}, sw {sw:#06x}, 0x603F = {err_code:#06x}, 0x2003 = {fault}, state {state}",
                 seen and sw == SW_FAULT and err_code == 0x8100 and fault == FAULT_COMM_LOST and state == t.ST_FAULT)
    finally:
        t.safe_state()
    record_recovery('C2', 'bus-off over')


# --- main ---------------------------------------------------------------------

def clean_up(master):
    try:
        t.sdo_write(0x1016, 1, 0, 4)         # first: a stopped master must not cause a timeout now
        t.sdo_write(0x1016, 2, 0, 4)
        t.sdo_write(0x1400, 5, 0, 2)
        t.sdo_write(0x1401, 5, 0, 2)
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
    print("safe state: 0x1016:01, 0x1400:05, 0x1401:05 = 0, 0x6007 = 1, NMT start, 0x6040 = 0, 0x2011 = 2, "
          "0x6060/0x2012 restored", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default='H,R')
    ap.add_argument('--log', default=str(Path(__file__).parent / 'logs'))
    ap.add_argument('--motor-ok', action='store_true', help='allow M (motor rotates)')
    args = ap.parse_args()
    tests = [x.strip().upper() for x in args.tests.split(',')]

    master = Cyclic(MASTER_HB, [NMT_OPERATIONAL], HB_PERIOD_S)
    t.bus = can.Bus(interface='slcan', channel=t.PORT, bitrate=500000)
    threading.Thread(target=t._rx_thread, daemon=True).start()
    try:
        if t.preflight():
            for name in tests:
                if name in ('M', 'O', 'S', 'C') and not args.motor_ok:
                    t.record(name, 'motor test', 'not run: --motor-ok missing', False, skipped=True)
                    continue
                try:
                    if name == 'H':
                        test_h(master)
                    elif name == 'M':
                        test_m(master)
                    elif name == 'O':
                        test_o()
                    elif name == 'R':
                        test_r()
                    elif name == 'S':
                        test_s()
                    elif name == 'B':
                        test_b()
                    elif name == 'C':
                        test_c()
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
        f = out / time.strftime('comm_loss_%Y%m%d_%H%M%S.json')
        f.write_text(json.dumps(dict(results=t.results, frames=[f"{tm - t.T0:9.4f} 0x{i:03X} {d.hex(' ')}"
                                                                 for tm, i, d in t.log]), indent=1))
        print(f"log: {f}")


if __name__ == '__main__':
    main()
