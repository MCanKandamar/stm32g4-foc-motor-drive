# Work
<!-- agent-lite v5. State, not instructions. Target ≤ 80 lines and ≤ 4 KB; git keeps the history.
Every item is one short line. Done keeps only the newest 5 items.
Never delete the active task, its plan, step status or evidence to meet the target.
When over the target, do all of these, in order: move lasting lessons and constraints from
Dead ends to PROJECT.md › Decisions & constraints (as "Constraint:"); drop obsolete dead ends; trim
Done to the newest items (git keeps history); shorten Next to one line per item; remove answered questions. -->

## Now
<!-- Task · branch · started date, then the plan. Mark the current step "(in progress)". Example:
Task: Password reset · branch: feature/reset · started: 2026-10-25
- [x] Token table + migration — verified: `npm test` ✓
- [ ] (in progress) Send reset e-mail
- [ ] Reset form -->
(empty)

## Next
<!-- Order set by the user 2026-10-30; CANopen phase 8 dropped 2026-11-02; Hall calibration in flash dropped 2026-11-03 -->
(empty: choose from the "v1:" items in Open questions)
Parked by the user: USB command refactor steps 4–6; DC-bus sag control (regen limit is done).
Later: commit e-mail in history; Docker image for CI; rename the working folder (steps in PROJECT.md).
Release: `v0.6.0` released on b4461e7; `v0.7.0` later (user tags it) with 74292f4 EDS + 5acaeb2 regen; `v1.0.0` waits for the "v1:" items in Open questions.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. Older items: git log. -->
- 2026-11-03 DRV8350S register values (user's OK): IDRIVE 100/200 mA (~60/30 ns for Qgd 6 nC), dead time 100 ns, OCP latch + all half-bridges off, VDS 0.4 V (~67 A cold / ~40 A hot), deglitch 2 µs — built ✓ (Debug 163.8, Release 97.9/512 KB, RAM 34.4/128 KB), `ctest` 186/186 ✓, HW verified (agent, 2026-11-03): read-back OK (motor starts), `test_pdo` D,A,B,C ✓, `test_cia402` T5,T6 ✓, `regen_test` Hall/SL max 28.5 V no fault; not measured: switch-node edges (needs a scope), Scrutiny speed step (Scrutiny USB not plugged)
- 2026-11-03 DRV8350S over SPI: hardware-free `motor_control/drv8350s` (configure + read-back, status), `powerOn()` configures after every ENABLE high (fail → `DRIVER_FAULT`), nFAULT → MOE off, `drvFaultPoll()` reads the cause (→ `DRIVER_OVER_CURRENT` / `DRIVER_OVER_TEMP`, `drv_status`), then ENABLE low; software SPI on PB13/14/15 (SPI2 removed in CubeMX, 200 kHz) because SDO has no pull-up; register values = chip reset values (65b084e) — built ✓ (Debug 163.8, Release 97.9/512 KB, RAM 34.4/128 KB), `ctest` 186/186 ✓, HW verified (agent, 2026-11-03): 100/100 configure OK at 1, 2.5, 5 µs half clock (100/100 fail at 0.25 and 0.5 µs), read-back 0x03 0x3FF, 0x04 0x7FF, 0x05 0x169, status 0, configure 1.2 ms; `test_pdo` D,A,B,C ✓, `test_cia402` T5,T6 ✓; nFAULT cause path not tested (needs a real driver fault)
- 2026-11-03 Regen limit: Vdc filtered in the FOC ISR (`meas.vdc`), OV trip there, braking iq × `meas.regen_scale` (27 → 29 V), speed PI braking limit (no wind-up); `tools/hil/regen_test.py` — built ✓ (Debug 163.5, Release 97.6/512 KB, RAM 34.5/128 KB), `ctest` 180/180 ✓, HW verified (agent, 2026-11-03): 3000 → 500 rpm at 100000 rpm/s, old fw OV trip at 30.8 V in 37 ms, new max 28.5 V no fault (Hall, SL); `test_pdo` D,A,B,C ✓, `stop_test --can-only` 908 ms; Scrutiny trace not taken
- 2026-11-03 EDS clean-up: 0x1010/0x1011 read 0 and read-only, 0x1018:03 = 0 (= EDS RevisionNumber), header filled (74292f4) — built ✓, HW verified (agent, 2026-11-03): SDO reads 0, `save`/`load` write → abort 0x06010002
- 2026-11-03 Soft stop: 0x06/0x07, 0x2011 = 2, Scrutiny `cmd.stop` ramp on 0x6084 to 100 rpm (Hall, `HALL_STOP_RPM`, user's choice) / 300 rpm (SL), then PWM off; 0x00/0x02/fault/comm loss at once — built ✓ (Debug 163.0, Release 97.2/512 KB, RAM 34.5/128 KB), `ctest` 173/173 ✓, HW verified (agent, 2026-11-03): Hall end 100 rpm → IDLE 907 ms, SL 709 ms, 0x00 11 ms, `test_pdo` ✓ ×2; earlier ramp-to-0 runs: CAN Hall 1014 ms, 500 rpm/s 2020 ms, SL 747 ms, 0x00 57 ms, `test_pdo` D,A,B,C ✓, T5,T6 ✓; Vdc ≤ 23.9 V

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->
- 2026-11-03 · DRV8350S on the SPI2 peripheral with the PB14 internal pull-up · SDO is open-drain with no pull-up resistor, the line rises too slowly: configure failed 20/20 at 1.33 / 2.66 MHz, 7/20 at 664 kHz (slowest prescaler) · retry only on a board with an SDO pull-up
- 2026-11-03 · Field weakening, cancelled by the user · branch `feature/field-weakening` (0ffd667) kept local only, never push · findings:
  max 3960 rpm at id −5 A (24 V, was ~3560); speed swings above |id| ≈ 2.3 A (Hall and SMO): PLL lag (30 Hz) + negative id acts as
  negative inertia; a fault (PWM off) at 3800 rpm with id < 0 sent Vdc to 42.6 V · retry with faster PLLs or lag compensation.
  The branch also has a Hall 000/111 fix (angle runs on, resync on return) in cfb5134; cherry-pick it if Hall dropouts matter
- 2026-11-02 · 0x1010 storage in MCU flash · rejected by the user: flash is risky, an EEPROM is the normal place, not needed here · retry only on a board with an EEPROM
- 2026-11-02 · Forcing bus-off by flooding from the SLCAN adapter · COM15 refused writes after 3 runs and needed a replug, cause not proven · retry with another adapter, or short CAN_H to CAN_L

## Open questions
<!-- question · impact · default used meanwhile -->
- Fault reaction at high speed: active short (all low-side on) instead of PWM off? · PWM off lets back-EMF and phase current pump Vdc (42.6 V seen with FW); user finds it sensible; bus caps maybe 100 V, not read · PWM off
- Motor flux λ: SMO back-EMF read ~4.2 V peak at 1000 rpm (2026-11-03) → ~10 mWb if the SMO scale is right, vs 6 mWb given · affects SMO gain margin k vs E_max · 6 mWb in motor_parameters
- NTC B value is assumed (3950): R24 part number unknown · reading may be off by a few °C away from 25 °C · 3950
- v1: what load and run time for the load test? · every board test so far is unloaded · not tested
- v1: rerun on the final firmware: power cycle, bus-off (B, C), cable pull (K) · not run on this firmware
- Why does 0x6077 read 7–12 % under target during acceleration? · not measured · pdo C passes with ~8 ‰ of margin
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547) · cause not found · rerun once and report it
- CANdamar CiA 402 panel: does it resend 06/07 on a target change? · user's Trace check is open · none
