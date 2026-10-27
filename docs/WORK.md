# Work
<!-- agent-lite v1. State, not instructions. Target ≤ 80 lines.
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
Task: CANopen phase 6 — PDO mapping (RPDO1 controlword + target, TPDO1 statusword + actual) and SYNC; DBC file for CANdamar · started: 2026-10-27
Current OD (checked 2026-10-27): 4 RPDO + 4 TPDO, all disabled (COB-ID bit 31 = 1), mappings empty, transmission type 0xFE; SYNC consumer on 0x80, 0x1006 = 0 (no SYNC producer).
- [x] 1. Concept: PDO vs SDO, COB-IDs for node 13, communication (0x14xx/0x18xx) and mapping (0x16xx/0x1Axx) parameters, transmission types, SYNC (0x1005/0x1006) — explained 2026-10-27
- [x] 2. Mapping design — user approved OD changes 2026-10-27, design left to the agent (PROJECT.md › Decisions)
- [x] 3. OD: 0x1400/0x1401 enabled (0x20D, 0x30D), 0x1600/0x1601 mapped, 0x1800/0x1801 enabled (0x18D, 0x28D, event timer 100 ms), 0x1A00/0x1A01 mapped, in OD.c, .eds, .xdd — verified: xdd XML parse ✓; build: step 6
- [x] 4. Bridge: no change needed — CO_CONFIG_PDO default has OD_IO_ACCESS, so RPDO writes go through OD extensions (0x6060/0x2012 guard holds); the bridge reads 0x6040/0x60FF/0x6071 from OD_RAM every 1 ms like SDO writes
- [x] 5. DBC `canopen/od/bldc_drive.dbc`: SYNC, RPDO1/2, TPDO1/2, EMCY, heartbeat
- [x] 6. Build — built ✓ (local Debug, no warnings; flash 177.4/512 KB, RAM 66.5/128 KB)
- [ ] 7. (in progress) HW test (user): TPDO1/2 every 100 ms, RPDO1 pv run, RPDO2 tq run, SYNC mode (0x1800:02 = 1 + 0x080), DBC in CANdamar — built ✓, needs HW test. User 2026-10-27: TPDO1/2 at ~10/s after NMT start ✓, DBC decoding ✓, pv sensorless reaches target (1501/1500 rpm, sw 0x0637) ✓; dip on target change = drive briefly left OE (sw ≈ 0x0231/0x0233), probably the CANdamar CiA 402 panel resending 06/07/0F, check with Trace; not run yet: pv Hall, tq, SYNC; node boots pre-operational (phase 7)
- [ ] 8. WORK.md / PROJECT.md wrap-up
Branch: main (user 2026-10-27: work on main for now; cloud agent commits unsigned in the user's name, the user re-signs and force-pushes). The user is learning CANopen: explain each step from basics with byte/frame examples first.

## Next
<!-- CANopen roadmap (fixed list, agreed with the user 2026-10-22) -->
1. Fix (user 2026-10-27): wrong actual speed at standstill — with sensorless selected, 0x606C/0x2001 show the SMO/PLL estimate while stopped (e.g. −3808 rpm), so pv bit 12 is never set; report Hall speed (or 0) unless SMO is the active angle source (`canopen_bridge.cpp`, `CanopenBridgeUpdate`)
2. CANopen phase 7: robustness — heartbeat consumer / 0x6007, bus-off recovery, NMT return to operational after comm errors, error register 0x1001
3. Persistent calibration (user 2026-10-26, postponed after CiA 402 on 2026-10-27): Hall edge table in a reserved flash page, loaded at boot; needs approval for the linker-script region
4. CANopen phase 8: EDS export and 0x1010 store parameters (uses the flash storage from item 3)
5. Host build for the `tests/` GoogleTest suite, then run it in CI (see Open questions)
Parked by the user: USB command refactor steps 4–6 (command queue, USB parser, telemetry); DC-bus sag and regen/over-voltage control.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. -->
- 2026-10-27 CANopen phase 5 — modes of operation 0x6060/0x6061: Profile Velocity (3, SPEED + `motor_control::Ramp`, 0x6083/0x6084) and Profile Torque (4, CURRENT, iq = T / kt), sensor mode 0x2012, 0x6060/0x2012 write guard (SDO abort 0x08000022 in OE), statusword bits 10/12 (7927f2f … 046e8e6) — built ✓ (CI run 26, Debug + Release, Debug flash 177.5/512 KB, RAM 66.5/128 KB), verified: test_ramp 6/6 ✓ (host, gtest stand-in), HW verified (user, 2026-10-27: pv ramp on Hall and sensorless, tq on Hall, 0x6060/0x2012 abort in OE, 0x2010 jump)
- 2026-10-27 CANopen phase 4 — CiA 402 state machine (0x6040/0x6041), fault → 0x603F + EMCY, INIT state ↔ Not Ready to Switch On, non-blocking current offset in the ADC ISR, NMT stays operational on motor faults (318c03f … 0d3fb2f) — built ✓ (CI run 18, Debug flash 175.5/512 KB, RAM 66.3/128 KB), HW verified (user + local agent, 2026-10-26/27: T0–T13, fault/EMCY, 0x80 edge, NMT 05 kept on fault = 5)
- 2026-10-26 GitHub + CI/CD: private repo `MCanKandamar/stm32g4-foc-motor-drive` (MIT), `build.yml` (Debug+Release matrix, size, .elf/.map artifacts) via PR #1, `release.yml` (`v*` tag → GitHub Release with .elf/.hex/.bin) via PR #2, tag `v0.1.0` on main (dd47c8a); main merged into this branch (a5674eb) — runs reported green by the user (not seen by the agent: private repo, no `gh`)
- 2026-10-25 agent-lite installed (AGENTS.md, CLAUDE.md, docs/WORK.md, docs/PROJECT.md) — built ✓ (Debug: flash 175.4/512 KB, RAM 66.3/128 KB), verified: `cmake --build --preset Debug` ✓ from bash with the bundle PATH

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->

## Open questions
<!-- question · impact · default used meanwhile -->
- How should the `tests/` GoogleTest suite be built on the PC (host CMake project or preset)? · motor_control changes cannot be unit-tested by agents · meanwhile: report `not run (no host test build)`
