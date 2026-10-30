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
- [x] 1. `AnglePLL::reset()` also sets `pll_speed_filtered` — verified: `motor_control_tests` 51/51 ✓ (scratchpad host build, MinGW), built ✓ (Debug, no new warnings; flash 177.4/512 KB, RAM 66.5/128 KB), HW verified (agent on-target, tests approved by the user, 2026-10-30, firmware 8a4501a6: ALIGN → OL → CLOSED_LOOP in T5, T6, pdo A1; sensorless step 1000 → 1300 rise 22 ms, overshoot 17.7 %, was 20 ms, 17.4 %)
- [x] 2. Host test preset `HostTests` for `tests/`; gtest download pinned with SHA-256 — verified: `ctest --preset HostTests` 51/51 ✓, built ✓ (Debug, no new warnings)
- [x] 3. CI: `host-tests` job in `build.yml`; `release.yml` needs it — verified: both files parse (PyYAML) ✓; unverified (not pushed, nothing has run on Linux yet)
- [x] 4. Test names in English — verified: `ctest --preset HostTests` 51/51 ✓ (names only, 51 lines changed)
- [x] 5. New tests: `state_machine`, `PidController`, `speed_control`, `startup`, `math_utils`, `hall_sensor`, `observer` + `AnglePLL::update_emf` — verified: `ctest --preset HostTests` 131/131 ✓; `motor_control/src` line coverage 31 % → 341/345 (gcov, MinGW); 5 deliberate code breaks in a scratch copy were all caught
- [x] 6. CiA 402 logic moved to `motor_control/cia402`, `canopen_bridge.cpp` only copies OD ↔ `cmd` — verified: `ctest --preset HostTests` 157/157 ✓ (26 new CiA 402 tests), built ✓ (Debug flash 177.6/512 KB, RAM 66.5/128 KB; Release flash 103.4/512 KB, RAM 66.5/128 KB, no new warnings), HW verified (agent on-target, tests approved by the user, 2026-10-30, firmware 8a4501a6: T1 via NMT reset, T2, T3, T4, T6, T7, T8a–c, pdo D, A, B pass); T5 and T9 fail on stale script expectations, not on this commit; open: pdo C1 at 50 ‰ (see Open questions)
- [x] 7. Scrutiny CLI 0.16.0 in `release.yml`: publishes `_tagged.elf`, `.sfd`, and hex/bin made from the tagged ELF — verified: file parses (PyYAML) ✓; unverified (needs a `v*` tag on GitHub, PySide6 install on the runner not tried)
- [x] 8a. `scrutiny-embedded/LICENSE` added (MIT text from the Scrutiny 0.16.0 installer)
- [ ] 8b. ThreadX/USBX licence (see Open questions): user chose FreeRTOS + TinyUSB 2026-10-30; migration runs on `feature/freertos-tinyusb` in its own session, this step closes when it lands

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
- How to replace the ThreadX/USBX 6.2.0 copy (X-CUBE-AZRTOS-G4 2.0.0, Microsoft licence, no public source redistribution)? · blocks making the repo public; the 15 STM32 USBX controller files are not in the Eclipse repo; CubeMX regeneration would restore the old copy; old commits keep these files, so a public repo also needs a clean history · decided 2026-10-30 (user): FreeRTOS (MIT, in STM32CubeG4) + TinyUSB (MIT, `stm32_fsdev` on G4, FreeRTOS OSAL, no ThreadX OSAL), `motor_control/`, `isr.cpp` and `canopen/` unchanged; still open: clean history or a fresh repo · meanwhile: repo stays private
- Why does `tools/hil/test_pdo.py` C1 not turn the motor at 50 ‰ (0.30 A)? · 0x6077 tracks (mean 46.9–49.6 ‰) but speed stays 0 in both runs; 100 ‰ (0.60 A) reaches 2663 rpm in ~0.3 s. Running friction is ≈ 0.18 A, so breakaway friction is the likely cause, not proven; a5b5afd kept the torque path; no earlier log has a C result · next: `--tq 60..90` from two rotor positions with the user at the board · meanwhile: `main` at 50 ‰ and 100 ‰ is the baseline for the FreeRTOS branch
- Stale expectations in `tools/hil/test_cia402_v2.py` · T5 `wait_sw` wants exactly 0x0237 but bit 12 gives 0x1237 at speed 0 (reached ~2 ms after the write, the script reports 32–35 ms), and its 6.0 s sample (392–395 rpm) falls inside the 0x6084 ramp 500 → 300 rpm; bit 12 and the ramp came in c3d5ed5, after T5's last pass. T9 expects 0x06 to stay ignored, but it is applied after INIT ends (0x0231 at 12.5 ms, never during INIT; same on 2026-10-27); `ControlwordIsIgnoredDuringInit` covers the INIT rule · meanwhile: read T5 and T9 failures as known
- One `test_pdo.py` start got no SDO answer in preflight (pdo_20261030_122547, right after the D run); the retry passed · cause not found, may belong to Next 2 · meanwhile: rerun once and report it
