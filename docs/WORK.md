# Work
<!-- agent-lite v2. State, not instructions. Target ≤ 80 lines.
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
Task: Next fixes A + B (user 2026-10-30: "do 1 and 6") · branch: main · started: 2026-10-30
- [x] A. Standstill speed (user 2026-10-27): with sensorless selected, 0x606C/0x2001 show the SMO/PLL estimate while stopped (e.g. −3808 rpm), so pv bit 12 is never set. Cause: in IDLE `angle_src` = FIXED, and the bridge uses SMO speed for every source except HALL. Fix: SMO speed only when `angle_src` = SMO, else Hall speed (0 after the Hall timeout). Also 0x6077 keeps the last iq when the motor is off (seen 2026-10-30: constant 43 / 21 / 54 ‰ in IDLE) because `foc.update()` does not run with `LoopMode::OFF` → report 0 then (`canopen_bridge.cpp`, `CanopenBridgeUpdate`) — built ✓ (Debug, no warnings; flash 177.4/512 KB, RAM 66.5/128 KB), needs HW test
- [x] B. Hall speed-loop oscillation below ~1000 rpm (found 2026-10-30, Scrutiny datalog 250 Hz on a pv Hall 0 → 1000 rpm run): `speed_rpm_hall` and `speed_rpm_smo` both swing ±400 rpm at ~6 Hz, Hall ~30–40 ms behind SMO, `meas.idq_ref.iq_ref` −0.9…+1.2 A in antiphase; damps after the ramp, gone at 1500 rpm, none on sensorless at 1000. Likely cause: the 24-edge (1 mechanical turn) average in `HallSensor::onEdge` delays the feedback by T/2 (30 ms at 1000 rpm, 60 ms at 500). User chose (2026-10-30): Hall PLL speed (`pll.get_speed()` → `meas.speed_rpm_hall_pll`) as the Hall speed-loop feedback in `isr.cpp`; the user's uncommitted speed-PI change Kp = Ki = 0.003 → 0.001 (`app.cpp`) goes in the same commit — built ✓ (Debug, no warnings; flash 177.4/512 KB, RAM 66.5/128 KB), needs HW test

## Next
<!-- Order set by the user 2026-10-30 -->
1. Host build for the `tests/` GoogleTest suite, then run it in CI (see Open questions)
2. CANopen phase 7: robustness — heartbeat consumer / 0x6007, bus-off recovery, NMT return to operational after comm errors, error register 0x1001 (seen 2026-10-30: SLCAN channel closed → no ACK → EMCY 0x8120/0x8110 TX overflow → node goes pre-op and stays there; an NMT start while the error is still active is undone; under-voltage fault latched at power-up when the MCU starts before the 24 V supply, needs a 0x80 fault reset)
3. CANopen phase 8: EDS export and 0x1010 store parameters (needs flash storage; it was planned to come from item 4, which now comes after it)
4. Persistent calibration (user 2026-10-26, last by the user's order 2026-10-30): Hall edge table in a reserved flash page, loaded at boot; needs approval for the linker-script region
Parked by the user: USB command refactor steps 4–6 (command queue, USB parser, telemetry); DC-bus sag and regen/over-voltage control.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. -->
- 2026-10-30 CANopen phase 6 — PDO mapping: RPDO1 0x20D (0x6040 + 0x60FF), RPDO2 0x30D (0x6040 + 0x6071), TPDO1 0x18D (0x6041 + 0x606C), TPDO2 0x28D (0x6041 + 0x6077 + 0x6061), type 0xFE, event timer 100 ms; SYNC consumer (synchronous TPDO by 0x18xx:02 = 1..240); DBC `canopen/od/bldc_drive.dbc` (b0dc005) — built ✓ (Debug, no warnings; flash 177.4/512 KB, RAM 66.5/128 KB), HW verified (user 2026-10-27: TPDO 10/s, DBC in CANdamar, pv sensorless; agent on-target test approved by the user, 2026-10-30: SYNC type 1 → one TPDO1 per SYNC, type 2 → every 2nd, none without SYNC; pv sensorless and Hall via RPDO1, target change with cw 0x0F never leaves OE; tq via RPDO2, mode display 4, 0x6077 ≈ target; results accepted by the user 2026-10-30). Open (user): CANdamar CiA 402 panel Trace check (does it resend 06/07 on a target change?). Findings → Next 1, 2, 6
- 2026-10-30 Code comments translated from Turkish to simple English (32 own files, comments only, line counts kept); future-comment rule in AGENTS.md › Conventions — verified: comment-stripped sources equal HEAD (26 C/C++ via `g++ -fpreprocessed -E`, 6 CMake/config) ✓, Debug .bin sha256 unchanged (807f8cc4…) ✓, built ✓ (Debug, no warnings; flash 177.4/512 KB, RAM 66.5/128 KB); afterwards the Turkish text inside code (CMake `message()`, `tools/hil/test_cia402_v2.py` prompts) translated too — verified: `python -m py_compile` ✓, built ✓
- 2026-10-27 CANopen phase 5 — modes of operation 0x6060/0x6061: Profile Velocity (3, SPEED + `motor_control::Ramp`, 0x6083/0x6084) and Profile Torque (4, CURRENT, iq = T / kt), sensor mode 0x2012, 0x6060/0x2012 write guard (SDO abort 0x08000022 in OE), statusword bits 10/12 (7927f2f … 046e8e6) — built ✓ (CI run 26, Debug + Release, Debug flash 177.5/512 KB, RAM 66.5/128 KB), verified: test_ramp 6/6 ✓ (host, gtest stand-in), HW verified (user, 2026-10-27: pv ramp on Hall and sensorless, tq on Hall, 0x6060/0x2012 abort in OE, 0x2010 jump)
- 2026-10-27 CANopen phase 4 — CiA 402 state machine (0x6040/0x6041), fault → 0x603F + EMCY, INIT state ↔ Not Ready to Switch On, non-blocking current offset in the ADC ISR, NMT stays operational on motor faults (318c03f … 0d3fb2f) — built ✓ (CI run 18, Debug flash 175.5/512 KB, RAM 66.3/128 KB), HW verified (user + local agent, 2026-10-26/27: T0–T13, fault/EMCY, 0x80 edge, NMT 05 kept on fault = 5)
- 2026-10-26 GitHub + CI/CD: private repo `MCanKandamar/stm32g4-foc-motor-drive` (MIT), `build.yml` (Debug+Release matrix, size, .elf/.map artifacts) via PR #1, `release.yml` (`v*` tag → GitHub Release with .elf/.hex/.bin) via PR #2, tag `v0.1.0` on main (dd47c8a); main merged into this branch (a5674eb) — runs reported green by the user (not seen by the agent: private repo, no `gh`)
- 2026-10-25 agent-lite installed (AGENTS.md, CLAUDE.md, docs/WORK.md, docs/PROJECT.md) — built ✓ (Debug: flash 175.4/512 KB, RAM 66.3/128 KB), verified: `cmake --build --preset Debug` ✓ from bash with the bundle PATH

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->

## Open questions
<!-- question · impact · default used meanwhile -->
- How should the `tests/` GoogleTest suite be built on the PC (host CMake project or preset)? · motor_control changes cannot be unit-tested by agents · meanwhile: report `not run (no host test build)`
