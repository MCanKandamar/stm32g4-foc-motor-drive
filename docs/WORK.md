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
<!-- Order set by the user 2026-10-30; CANopen phase 8 dropped 2026-11-02; Hall calibration in flash dropped 2026-11-03 -->
(empty: choose from the "v1:" items in Open questions)
Parked by the user: USB command refactor steps 4–6; DC-bus sag and regen/over-voltage control.
Later: commit e-mail in history; Docker image for CI; rename the working folder (steps in PROJECT.md).
Release: `v0.5.0` is the latest; `v1.0.0` waits for the "v1:" items in Open questions.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. Older items: git log. -->
- 2026-11-03 Soft stop: 0x06/0x07, 0x2011 = 2, Scrutiny `cmd.stop` ramp on 0x6084 to 0 rpm (Hall) / 300 rpm (SL), then PWM off; 0x00/0x02/fault/comm loss at once — built ✓ (Debug 163.0, Release 97.2/512 KB, RAM 34.5/128 KB), `ctest` 173/173 ✓, HW verified (agent, 2026-11-03): CAN Hall 1014 ms, 500 rpm/s 2020 ms, SL 747 ms, 0x00 57 ms, `test_pdo` D,A,B,C ✓, T5,T6 ✓; Vdc ≤ 23.9 V
- 2026-11-03 README rewritten with board plots (speed step, SL start-up, SMO vs Hall angle, back-EMF/duties), docs moved to docs/ + motor_control/docs/ (checked vs code by side session) — `ctest` 168/168 ✓, Debug built ✓, links ✓; motor runs by agent with user OK; GitHub rendering not seen yet
- 2026-11-03 Board NTC on PC3 (ADC1 IN9) → `status.pcb_temp_c`, `DRIVER_OVER_TEMP` above 85 °C (a8be70c) — built ✓, `ctest` 168/168 ✓, HW verified (agent, 2026-11-03): 24.9 °C at rest; warm-up test skipped by the user, B 3950 assumed
- 2026-11-03 FOC ISR path at -O2 in Debug + `FOC_RATE_ERROR` (0x6100) — built ✓ (Debug 160.9, Release 95.6/512 KB), `ctest` 164/164 ✓, HW verified (agent, 2026-11-03): old Debug → 0x6100, new 20 kHz, `test_pdo.py` ✓ both
- 2026-11-03 IWDG watchdog 100 ms, fed by Supervisor/CANopen/Monitor alive flags, `reset_by_iwdg` for Scrutiny (feature/iwdg, merged) — built ✓ (Release flash 95.4/512 KB, RAM 34.3/128 KB), HW verified (agent, 2026-11-03): halt → IWDG reset, `reset_by_iwdg` 1/0, `test_pdo.py` D,A,B,C ✓

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->
- 2026-11-03 · Field weakening, cancelled by the user · branch `feature/field-weakening` (0ffd667) kept local only, never push · findings:
  max 3960 rpm at id −5 A (24 V, was ~3560); speed swings above |id| ≈ 2.3 A (Hall and SMO): PLL lag (30 Hz) + negative id acts as
  negative inertia; a fault (PWM off) at 3800 rpm with id < 0 sent Vdc to 42.6 V · retry with faster PLLs or lag compensation.
  The branch also has a Hall 000/111 fix (angle runs on, resync on return) in cfb5134; cherry-pick it if Hall dropouts matter
- 2026-11-02 · 0x1010 storage in MCU flash · rejected by the user: flash is risky, an EEPROM is the normal place, not needed here · retry only on a board with an EEPROM
- 2026-11-02 · Forcing bus-off by flooding from the SLCAN adapter · COM15 refused writes after 3 runs and needed a replug, cause not proven · retry with another adapter, or short CAN_H to CAN_L

## Open questions
<!-- question · impact · default used meanwhile -->
- Fault reaction at high speed: active short (all low-side on) instead of PWM off? · PWM off lets back-EMF and phase current pump Vdc (42.6 V seen with FW); user finds it sensible; bus caps maybe 100 V, not read · PWM off
- v1: which DRV8350S registers to write over SPI? · dead time comes from the driver default, TIM8 adds none; `DRIVER_OVER_CURRENT` is never raised · driver defaults
- Hall soft stop: Hall PLL speed swings ±100 rpm below ~150 rpm, iq up to 0.8 A at the end · end the Hall ramp at ~100 rpm and coast? · ramp to 0 rpm
- Motor flux λ: SMO back-EMF read ~4.2 V peak at 1000 rpm (2026-11-03) → ~10 mWb if the SMO scale is right, vs 6 mWb given · affects SMO gain margin k vs E_max · 6 mWb in motor_parameters
- NTC B value is assumed (3950): R24 part number unknown · reading may be off by a few °C away from 25 °C · 3950
- v1: what load and run time for the load test? · every board test so far is unloaded · not tested
- v1: limit regen by a deceleration cap or by control? · a fast stop can raise Vdc, checked only every 10 ms · 30 V trip only
- v1: rerun on the final firmware: power cycle, under-voltage at power-up, bus-off (B, C), cable pull (K); finish the EDS · not run on this firmware
- Why does 0x6077 read 7–12 % under target during acceleration? · not measured · pdo C passes with ~8 ‰ of margin
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547) · cause not found · rerun once and report it
- CANdamar CiA 402 panel: does it resend 06/07 on a target change? · user's Trace check is open · none
