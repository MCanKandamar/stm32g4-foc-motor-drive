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
Task: KISS/YAGNI trim, list approved by the user · branch: main · started: 2026-11-02
Kept by the user's choice: `PidController`, OD 0x1010/0x1011, PDO 3–4, all `motor_parameters.hpp` fields.
- [x] Dead code, repeated constants, CMake leftovers (A2–A7) — built ✓ (Debug flash 167.4/512 KB, RAM 34.3/128 KB), verified: `ctest --preset HostTests` 164/164 ✓, needs HW test
- [x] CANopen port: storage files, old comment, `log_printf` off (B1–B3) — built ✓ (Debug flash 166.5/512 KB, RAM 34.3/128 KB), needs HW test
- [x] `status.rpm`/`theta_deg`, divider array, `test_cia402.py` rename (C2, C3, C5) — built ✓ (Debug flash 166.4/512 KB, RAM 34.3/128 KB), verified: `ctest --preset HostTests` 164/164 ✓, needs HW test
- [ ] (in progress) `bldc_drive.xdd`: drop the 22 disabled objects, fill empty descriptions, export OD and EDS

## Next
<!-- Order set by the user 2026-10-30; CANopen phase 8 dropped 2026-11-02 -->
1. Persistent Hall calibration in a reserved flash page: see Open questions
Parked by the user: USB command refactor steps 4–6; DC-bus sag and regen/over-voltage control.
Later: commit e-mail in history; Docker image for CI; rename the working folder (steps in PROJECT.md).
Release: `v0.4.0` is the latest; next is `v1.0.0`, the first stable release.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. Older items: git log. -->
- 2026-11-02 README brought up to date (no stored parameters, Status, limits, agent-lite, licence note); isr.cpp uses `POLE_PAIRS` from the motor data — built ✓ (Debug flash 167.7/512 KB, RAM 34.3/128 KB), verified: `ctest --preset HostTests` 164/164 ✓, needs HW test
- 2026-11-02 CANopen phase 7: 0x6007, heartbeat, RPDO timeout, bus-off, one control owner (up to 32728bd) — built ✓ (Debug flash 167.7/512 KB, RAM 34.3/128 KB), verified: `ctest --preset HostTests` 164/164 ✓, HW verified (agent + user, 2026-11-02). Not checked: under-voltage change on the board, cable pull to motor stop time
- 2026-10-30 `v0.4.0` released from 22c9be1 — CI ✓, Release has all 4 files; unverified on the board (this build not flashed)
- 2026-10-30 FreeRTOS + TinyUSB replace ThreadX + USBX — built ✓, verified: `ctest --preset HostTests` 157/157 ✓, HW verified (agent runs reported by the user, 2026-10-30)
- 2026-10-30 CANopen phases 1–6 complete: details in PROJECT.md › Decisions and git log

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->
- 2026-11-02 · 0x1010 storage in MCU flash · rejected by the user: flash is risky, an EEPROM is the normal place, not needed here · retry only on a board with an EEPROM
- 2026-11-02 · Forcing bus-off by flooding from the SLCAN adapter · COM15 refused writes after 3 runs and needed a replug, cause not proven · retry with another adapter, or short CAN_H to CAN_L

## Open questions
<!-- question · impact · default used meanwhile -->
- Is Hall calibration in flash still wanted after the phase 8 decision? · decides Next 1 · kept in Next until the user answers
- ThreadX wording left in `CO_app_STM32.c` line 191 · cosmetic · unchanged
- Why does 0x6077 read 7–12 % under target during acceleration? · not measured · pdo C passes with ~8 ‰ of margin
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547) · cause not found · rerun once and report it
- CANdamar CiA 402 panel: does it resend 06/07 on a target change? · user's Trace check is open · none
