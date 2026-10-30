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
Task: host unit tests in CI, missing tests, open-source blockers · branch: main (PROJECT.md decision 2026-10-27) · started: 2026-10-30 · the agent writes the code (user's choice for this session)
- [x] 1. `AnglePLL::reset()` also sets `pll_speed_filtered` — verified: `motor_control_tests` 51/51 ✓ (scratchpad host build, MinGW), built ✓ (Debug, no new warnings; flash 177.4/512 KB, RAM 66.5/128 KB), needs HW test (normal sensorless start)
- [x] 2. Host test preset `HostTests` for `tests/`; gtest download pinned with SHA-256 — verified: `ctest --preset HostTests` 51/51 ✓, built ✓ (Debug, no new warnings)
- [x] 3. CI: `host-tests` job in `build.yml`; `release.yml` needs it — verified: both files parse (PyYAML) ✓; unverified (not pushed, nothing has run on Linux yet)
- [x] 4. Test names in English — verified: `ctest --preset HostTests` 51/51 ✓ (names only, 51 lines changed)
- [ ] (in progress) 5. New tests: `state_machine`; `PidController` + `speed_control`; `startup` + `math_utils`; `hall_sensor`; `observer` + `AnglePLL::update_emf`. Baseline: 31 % of `motor_control/src` lines (107/344, gcov, MinGW)
- [ ] 6. Move the CiA 402 logic from `canopen_bridge.cpp` to hardware-free code and test it
- [ ] 7. Scrutiny CLI in `release.yml`: `_tagged.elf`, `.sfd`, hex/bin from the tagged ELF (PyPI 0.16.0 needs Python < 3.14)
- [ ] 8. Licences before going public: ThreadX/USBX (see Open questions), `scrutiny-embedded` LICENSE file

## Next
<!-- Order set by the user 2026-10-30 -->
1. (active, see Now) Host build for the `tests/` GoogleTest suite, then run it in CI (see Open questions)
2. CANopen phase 7: robustness — heartbeat consumer / 0x6007, bus-off recovery, NMT return to operational after comm errors, error register 0x1001 (seen 2026-10-30: SLCAN channel closed → no ACK → EMCY 0x8120/0x8110 TX overflow → node goes pre-op and stays there; an NMT start while the error is still active is undone; under-voltage fault latched at power-up when the MCU starts before the 24 V supply, needs a 0x80 fault reset)
3. CANopen phase 8: EDS export and 0x1010 store parameters (needs flash storage; it was planned to come from item 4, which now comes after it)
4. Persistent calibration (user 2026-10-26, last by the user's order 2026-10-30): Hall edge table in a reserved flash page, loaded at boot; needs approval for the linker-script region
Parked by the user: USB command refactor steps 4–6 (command queue, USB parser, telemetry); DC-bus sag and regen/over-voltage control.
Later (user 2026-10-30): top-level README, commit e-mail in history, Docker image for CI (look at it after Now step 7).

## Done
<!-- Newest first, one line each, with evidence label and commit if any. -->
- 2026-10-30 Speed PI tuned on the board: Kp = 0.0025, Ki = 0.02 (was 0.001 / 0.001: stable but ~250 rpm behind a 1000 rpm/s ramp). Plant from a step: friction ≈ 0.18 A, K ≈ 20 000 rpm/(s·A); model with 1 kHz PI + 30 Hz PLL matched (rise 58 vs 58 ms), PM 52°, GM 12 dB; new tool `tools/hil/speed_step.py` (5dace58) — built ✓ (Debug flash 177.4/512 KB, RAM 66.5/128 KB; Release flash 103.9/512 KB, RAM 66.5/128 KB), HW verified (agent on-target 2026-10-30, flashed Release 8695dc0b by the user: Hall 1000 → 1300 step rise 22 ms (was 58), overshoot 23 %; sensorless rise 20 ms, overshoot 17 %; Hall ramp 1000 → 1500 tracking 31 rpm (was 49), settle 166 ms (was 410); earlier with RAM gains: 200 → 400 rise 18 ms, pv 0 → 1000 ramp −15…+7 rpm then 1000.9 ± 1.2 rpm)
- 2026-10-30 Hall speed-loop oscillation (±400 rpm, 6 Hz below ~1000 rpm): the 24-edge average lagged T/2; the speed PI now uses the Hall PLL speed `meas.speed_rpm_hall_pll` (`isr.cpp`) (9be77c7) — built ✓, HW verified (user 2026-10-30: oscillation gone; agent: 0 → 500 and 0 → 1000 rpm Hall ramps, no oscillation)
- 2026-10-30 Standstill speed/torque in the OD: 0x606C/0x2001 use the SMO speed only while the SMO gives the angle, else Hall; 0x6077 = 0 while PWM is off (`canopen_bridge.cpp`) (1f8f63b) — built ✓, HW verified (user 2026-10-30: 0 at standstill with sensorless; agent: Hall pv target 0 → sw 0x1637, IDLE after a run 0x6077 = 0, 0x606C = 0)
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
- How to replace the ThreadX/USBX 6.2.0 copy (X-CUBE-AZRTOS-G4 2.0.0, Microsoft licence, no public source redistribution)? · blocks making the repo public; the 15 STM32 USBX controller files are not in the Eclipse repo; CubeMX regeneration would restore the old copy · meanwhile: repo stays private
