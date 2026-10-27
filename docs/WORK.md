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
Task: CANopen phase 5 — modes of operation 0x6060/0x6061: Profile Velocity (pv = 3) and Profile Torque (tq = 4) · branch: main (user 2026-10-27: work on main for now; cloud agent commits unsigned in the user's name, the user re-signs and force-pushes) · started: 2026-10-27
The user is learning CANopen: explain each step from basics with byte/frame examples first. Cloud agent has no Arm toolchain: firmware build = CI on push.
- [x] 1. Concept: 0x6060/0x6061, pv objects (0x60FF, 0x606C, 0x6083/0x6084), tq objects (0x6071 ‰ of 0x6076, 0x6077, 0x6087), statusword bits 10/12, units — explained 2026-10-27
- [x] 2. Mapping design — approved by the user 2026-10-27 (PROJECT.md › Decisions): rpm; pv ↔ SPEED + ramp; tq ↔ CURRENT, iq = T / kt; 0x6060 only outside Operation Enabled; sensor mode 0x2012 (0 = sensorless, 1 = Hall) used by pv only, tq always runs on Hall; 0x2010 kept as a direct speed reference without ramp
- [x] 3. OD: 0x2012, 0x6060 (default 3), 0x6061, 0x606C, 0x6071, 0x6076 (400 mNm, ro), 0x6077, 0x6083/0x6084 (default 1000 rpm/s), 0x6087 (default 1000 ‰/s), 0x60FF in OD.c/h, .eds, .xdd (hand-edited in CANopenEditor format, like 318c03f) — verified: host `gcc -fsyntax-only OD.c` ✓, xdd XML parse ✓; firmware build: see step 6
- [x] 4. Ramp generator `motor_control::Ramp` (inc/ramp.hpp, src/ramp.cpp) + `tests/test_ramp.cpp` — verified: host g++ -Wall -Wextra -Wpedantic -Wdouble-promotion -Wfloat-conversion, no warnings in ramp.cpp ✓; test_ramp 6/6 ✓ (run with a minimal gtest stand-in, real GoogleTest not available in the cloud container)
- [x] 5. Bridge: 0x6060/0x2012 write guard (OD extension, SDO abort 0x08000022), 0x60FF/0x2010/0x6071 → ramp → cmd, 0x6061/0x606C/0x6077 actuals, statusword bits 10/12, `CanopenBridgeInit()` — verified: host `g++ -fsyntax-only -Wall -Wextra -Wdouble-promotion -Wfloat-conversion` on canopen_bridge.cpp and canopen_app.c, no warnings in project files ✓ (HAL headers need -fpermissive on 64-bit host); firmware build: step 6
- [x] 6. CI build — built ✓ (CI run 26 on dc88c01, Debug + Release green, no compiler warnings in changed files; Debug flash 177.5/512 KB, RAM 66.5/128 KB)
- [ ] 7. (in progress) HW test (user): pv Hall/sensorless ramp, tq on Hall, 0x6060/0x2012 abort in OE, 0x2010 jump — built ✓, needs HW test
- [ ] 8. WORK.md / PROJECT.md wrap-up

## Next
<!-- CANopen roadmap (fixed list, agreed with the user 2026-10-22) -->
1. CANopen phase 6: PDO mapping (RPDO1 controlword + target, TPDO1 statusword + actual) and SYNC; DBC file for CANviz decoding
2. CANopen phase 7: robustness — heartbeat consumer / 0x6007, bus-off recovery, NMT return to operational after comm errors, error register 0x1001
3. Persistent calibration (user 2026-10-26, postponed after CiA 402 on 2026-10-27): Hall edge table in a reserved flash page, loaded at boot; needs approval for the linker-script region
4. CANopen phase 8: EDS export and 0x1010 store parameters (uses the flash storage from item 3)
5. Host build for the `tests/` GoogleTest suite, then run it in CI (see Open questions)
Parked by the user: USB command refactor steps 4–6 (command queue, USB parser, telemetry); DC-bus sag and regen/over-voltage control.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. -->
- 2026-10-27 CANopen phase 4 — CiA 402 state machine (0x6040/0x6041), fault → 0x603F + EMCY, INIT state ↔ Not Ready to Switch On, non-blocking current offset in the ADC ISR, NMT stays operational on motor faults (318c03f … 0d3fb2f) — built ✓ (CI run 18, Debug flash 175.5/512 KB, RAM 66.3/128 KB), HW verified (user + local agent, 2026-10-26/27: T0–T13, fault/EMCY, 0x80 edge, NMT 05 kept on fault = 5)
- 2026-10-26 GitHub + CI/CD: private repo `MCanKandamar/stm32g4-foc-motor-drive` (MIT), `build.yml` (Debug+Release matrix, size, .elf/.map artifacts) via PR #1, `release.yml` (`v*` tag → GitHub Release with .elf/.hex/.bin) via PR #2, tag `v0.1.0` on main (dd47c8a); main merged into this branch (a5674eb) — runs reported green by the user (not seen by the agent: private repo, no `gh`)
- 2026-10-25 agent-lite installed (AGENTS.md, CLAUDE.md, docs/WORK.md, docs/PROJECT.md) — built ✓ (Debug: flash 175.4/512 KB, RAM 66.3/128 KB), verified: `cmake --build --preset Debug` ✓ from bash with the bundle PATH

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->

## Open questions
<!-- question · impact · default used meanwhile -->
- How should the `tests/` GoogleTest suite be built on the PC (host CMake project or preset)? · motor_control changes cannot be unit-tested by agents · meanwhile: report `not run (no host test build)`
