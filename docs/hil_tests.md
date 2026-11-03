# Hardware-in-the-loop tests

The scripts in [tools/hil/](../tools/hil) test a real board over CAN and Scrutiny. Back to the [README](../README.md).

## Setup

- Python packages: `python-can`, `pyserial`, and an SLCAN adapter. Set the adapter port in `PORT` in `test_pdo.py` and `test_cia402.py`. `test_comm_loss.py` takes it from `test_pdo.py`.
- Scripts that use Scrutiny also need `scrutinydebugger` 0.16 and a running Scrutiny server.
- Any test that turns the motor needs the `--motor-ok` flag.
- Logs and CSV files go to `tools/hil/logs/`, which is not in git.

| Script | What it does |
|---|---|
| `test_cia402.py` | CiA 402 state machine, heartbeat, fault injection and reset. Sends NMT start first |
| `test_pdo.py` | PDOs, SYNC, profile velocity and profile torque over RPDOs |
| `test_comm_loss.py` | Heartbeat consumer, RPDO timeout, master reset, bus-off, cable pull |
| `speed_step.py` | Speed-step or speed-ramp response from a Scrutiny datalog, with rise time and overshoot |
| `sensorless_capture.py` | Scrutiny datalog of the sensorless start-up, the SMO angle against the Hall angle, or the SMO back-EMF and PWM duties |
| `hall_speed_capture.py` | Scrutiny datalog of the Hall and SMO speeds during a Hall run |
| `plot_readme.py` | Draws the README plots (SVG) from the newest logs |
| `drive_util.py` | Status read-out, fault reset, Hall calibration |

## Commands

T1 and T9 wait for a board reset. With `--nmt-reset`, the script sends the reset itself:

```shell
python tools/hil/test_cia402.py --tests T1,T9,T2,T3,T7,T8 --nmt-reset
python tools/hil/test_cia402.py --tests T5,T6 --motor-ok
python tools/hil/test_pdo.py --tests D,A,B,C --motor-ok
```

Communication loss. For the bus-off tests (B, C) you short CAN_H to CAN_L by hand, and in test K you pull the CAN cable:

```shell
python tools/hil/test_comm_loss.py --tests H,R
python tools/hil/test_comm_loss.py --tests H,M,O,R,S --motor-ok
python tools/hil/test_comm_loss.py --tests B
python tools/hil/test_comm_loss.py --tests C --motor-ok
python tools/hil/test_comm_loss.py --tests K --motor-ok
```

The README plots:

```shell
python tools/hil/speed_step.py --motor-ok
python tools/hil/speed_step.py --sensorless --motor-ok
python tools/hil/sensorless_capture.py --startup --motor-ok
python tools/hil/sensorless_capture.py --angle --motor-ok
python tools/hil/sensorless_capture.py --waves --motor-ok
python tools/hil/plot_readme.py
```

## Results of 2026-11-03: README plots

One board, firmware of commit `92cc29e`, 24 V, no load on the motor shaft. Each run ran once.

| Test | Result |
|---|---|
| Hall speed step, 1000 → 1300 rpm on `0x2010` | 18 ms rise (10 → 90 %), 20.1 % overshoot, 246 ms to stay within 5 %, peak iq reference 0.96 A |
| Sensorless speed step, 1000 → 1300 rpm on `0x2010` | 20 ms rise, 15.9 % overshoot, 180 ms to stay within 5 %, peak iq reference 0.95 A |
| Sensorless start-up to 1000 rpm | ALIGN → OPEN_LOOP → CLOSED_LOOP, 1000 rpm about 6.9 s after the start command |
| SMO angle against the Hall PLL angle at 1000 rpm, sensorless control | mean −0.8°, standard deviation 0.8°, largest 3.1° (electrical), over 0.34 s |
| SMO back-EMF and PWM duties at 1000 rpm, sensorless control | back-EMF about 4.2 V peak, duty cycles 0.35 to 0.65, over 0.2 s |

The Hall PLL angle is used as the reference, but it is an estimate too, so the angle result shows how well the two agree, not the true error of either one.

## Results of 2026-11-02: communication loss

| Test | Result |
|---|---|
| Heartbeat consumer, 500 ms setting | Timeout after 503 ms, fault 510 ms after the last frame |
| RPDO timeout, 300 ms setting | Timeout after 303 ms, fault 309 ms after the last frame |
| Bus-off with a CAN_H to CAN_L short | Recovered, motor off and at 1000 rpm |
| Cable pull at 1000 rpm | Fault, but the time from the pull to motor stop was not measured |

## Results of 2026-10-30: CiA 402 and PDOs

These tests passed on one board with the FreeRTOS firmware, in two runs. Run A used a local Release build, Scrutiny firmware ID `08c84ea7…`. Run B added the tests that need Scrutiny, but its log holds no firmware ID, so its build is not confirmed. The run logs are not in this repository.

| Test | What it checks | Result | Run |
|---|---|---|---|
| T1 | Start-up after an NMT reset | `0x6041` = `0x0250` after 7.5 ms, no fault | A |
| T9 | Controlword `0x06` right after boot-up | First read `0x0231` at 10.5 ms, none during `INIT` | A |
| T2 | Heartbeat period | 999.3 to 1000.5 ms | A |
| T3 | Controlword steps and heartbeat during the current-offset refresh | Each step 4.0 to 4.3 ms in A, 4.1 to 6.0 ms in B, limit 10 ms | A, B |
| T4 | Current offsets | 2045, 2045 and 2046 counts | B |
| T7 | Fault injected through Scrutiny | Statusword, `0x603F`, `0x1001` and EMCY as expected | B |
| T8a to T8c | Fault reset on the rising edge of bit 7 | As expected | B |
| T5 | Sensorless start to 300 rpm | ALIGN → OPEN_LOOP → CLOSED_LOOP, 312 to 319 rpm from 6.5 s | A |
| T6 | Enable again right after T5, with the rotor still coasting: no lock-up | `0x0237` after 3.9 ms, `0x0250` after 4.0 ms | A |
| PDO D | SYNC and TPDO types | One TPDO1 per SYNC, every second one on type 2, none without SYNC | A |
| PDO A | Sensorless profile velocity over RPDO1 | 1004 rpm mean, 1000 → 1500 rpm stays in Operation Enabled | A |
| PDO B | Hall profile velocity over RPDO1 | 1005 rpm mean, 1000 → 1500 rpm stays in Operation Enabled | A |
| PDO C | Hall profile torque over RPDO2, 80 ‰ for 0.45 s | `0x6077` mean 74.0 ‰, limit 65 to 95 ‰ | A |

Each test ran once, except T3, which ran twice. T6 has not been run from rest.

Read these limits with the table:

- T1 and T9 used a software reset, not a power cycle, so the real power-up path is not covered.
- T9 passes almost by design, because `INIT` is already over at the first read. The unit test `ControlwordIsIgnoredDuringInit` covers this case.
- T5 checks the speed only from 6.5 s after the start, because before that the rotor is still on the ramp from the 500 rpm hand-over.
- Timing depends on the PC: `python-can` reads SLCAN byte by byte, so busy Python code in the same process delays reception. The CiA 402 script waits 2 s after connecting to Scrutiny for this reason. A late frame in these tests can be a PC delay, not a firmware delay.
