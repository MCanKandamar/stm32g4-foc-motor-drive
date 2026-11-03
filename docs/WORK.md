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
Task: IWDG watchdog (v1) · branch: feature/iwdg · started: 2026-11-03
Design: ~100 ms (PR 32, RL 99); Monitor kicks only when Supervisor, CANopen and Monitor all reported alive; USB and Scrutiny not watched; started by our code, not by CubeMX.
- [x] CubeMX: IWDG on, no generated call, not static (user) — built ✓ (Debug flash 166.4/512 KB, RAM 34.3/128 KB), no new warnings
- [ ] (in progress) Start IWDG in Monitor, alive bits, kick — built ✓ (Debug flash 166.8/512 KB, RAM 34.3/128 KB), needs HW test
- [ ] Debug freeze (DBGMCU) and reset cause in Scrutiny
- [ ] Build Debug/Release, HW test plan

## Next
<!-- Order set by the user 2026-10-30; CANopen phase 8 dropped 2026-11-02; Hall calibration in flash dropped 2026-11-03 -->
1. v1: IWDG watchdog (design waits for the user's OK, then `.ioc` change)
Parked by the user: USB command refactor steps 4–6; DC-bus sag and regen/over-voltage control.
Later: commit e-mail in history; Docker image for CI; rename the working folder (steps in PROJECT.md).
Release: `v0.5.0` is the latest; `v1.0.0` waits for the "v1:" items in Open questions.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. Older items: git log. -->
- 2026-11-02 `v0.5.0` released from 8f1e363 — CI ✓, Release has all 4 files; unverified on the board (this CI build not flashed)
- 2026-11-02 KISS/YAGNI trim and XDD clean-up (757b24c..fe785e7) — built ✓ (Debug flash 166.4/512 KB, RAM 34.3/128 KB; Release flash 95.1/512 KB), verified: `ctest --preset HostTests` 164/164 ✓, HW verified (agent, user flashed Release 8efa2303…, 2026-11-02). Not run: comm loss B, C, K, power cycle
- 2026-11-02 README brought up to date; isr.cpp uses `POLE_PAIRS` — built ✓, verified: `ctest --preset HostTests` 164/164 ✓
- 2026-11-02 CANopen phase 7: 0x6007, heartbeat, RPDO timeout, bus-off, one control owner (up to 32728bd) — HW verified (agent + user, 2026-11-02)
- 2026-10-30 `v0.4.0` released from 22c9be1 — CI ✓; unverified on the board (this build not flashed)

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->
- 2026-11-02 · 0x1010 storage in MCU flash · rejected by the user: flash is risky, an EEPROM is the normal place, not needed here · retry only on a board with an EEPROM
- 2026-11-02 · Forcing bus-off by flooding from the SLCAN adapter · COM15 refused writes after 3 runs and needed a replug, cause not proven · retry with another adapter, or short CAN_H to CAN_L

## Open questions
<!-- question · impact · default used meanwhile -->
- v1: add the IWDG watchdog? (`.ioc` change) · a locked task leaves the FOC ISR running without Monitor · no watchdog
- v1: which DRV8350S registers to write over SPI? · dead time comes from the driver default, TIM8 adds none; `DRIVER_OVER_CURRENT` is never raised · driver defaults
- v1: read the NTC inputs, with which limits? · `MOTOR_OVER_TEMP` and `DRIVER_OVER_TEMP` are never raised · no temperature protection
- v1: what load and run time for the load test? · every board test so far is unloaded · not tested
- v1: limit regen by a deceleration cap or by control? · a fast stop can raise Vdc, checked only every 10 ms · 30 V trip only
- v1: rerun on the final firmware: power cycle, under-voltage at power-up, bus-off (B, C), cable pull (K); finish the EDS · not run on this firmware
- Why does 0x6077 read 7–12 % under target during acceleration? · not measured · pdo C passes with ~8 ‰ of margin
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547) · cause not found · rerun once and report it
- CANdamar CiA 402 panel: does it resend 06/07 on a target change? · user's Trace check is open · none
