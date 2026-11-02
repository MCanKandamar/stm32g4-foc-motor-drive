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

## Next
<!-- Order set by the user 2026-10-30; CANopen phase 8 dropped 2026-11-02 -->
1. README: say the drive stores no parameters (PROJECT.md, phase 8 decision) and update the Status table
2. Persistent Hall calibration in a reserved flash page: see Open questions
Parked by the user: USB command refactor steps 4–6; DC-bus sag and regen/over-voltage control.
Later: commit e-mail in history; Docker image for CI; rename the working folder (steps in PROJECT.md).
Release: `v0.4.0` is the latest; next is `v1.0.0`, the first stable release.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. Older items: git log. -->
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
- Is Hall calibration in flash still wanted after the phase 8 decision? · decides Next 2 · kept in Next until the user answers
- How to clean the git history of the Microsoft-licensed ThreadX/USBX files? · repo is public, tags `v0.1.0`–`v0.3.0` expose 317 files · user decided 2026-10-30 to stay public, clean-up not decided. Findings 2026-11-02: free STM32G4 use covers production and binaries; MS terms 2(a)(iii) and 3(c) still limit source; Microsoft says 6.x is also MIT (repo copy is 6.2.0); gap: `usbx_stm32_device_controllers` files are not in eclipse-threadx/usbx
- ThreadX wording left in `CO_app_STM32.c` line 191 · cosmetic · unchanged
- `motor_parameters.hpp` pole count is not used: `isr.cpp` lines 25–26, `app.hpp` line 39 and `app.cpp` line 64 hard-code 4 · isr.cpp fix asked by the user 2026-11-02 · unchanged
- Why does 0x6077 read 7–12 % under target during acceleration? · not measured · pdo C passes with ~8 ‰ of margin
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547) · cause not found · rerun once and report it
- CANdamar CiA 402 panel: does it resend 06/07 on a target change? · user's Trace check is open · none
