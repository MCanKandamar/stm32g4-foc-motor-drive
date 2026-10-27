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
Task: CANopen phase 4 — CiA 402 state machine (0x6040/0x6041/0x603F + EMCY) · branch: feature/canopen-cia402 · plan agreed 2026-10-25
Context: phases 0–3 done and HW verified (56258fd): node-ID 13, heartbeat 0x70D, SDO 0x60D/0x58D, objects 0x2000–0x2011 bridged in `App/Src/canopen_bridge.cpp`. CANviz writing 0x6040 currently gets an SDO abort (object missing).
The user is learning CANopen: explain each step from basics with byte/frame examples first, then guide; the user writes the code (and edits the OD) unless they say "kodu yaz" / "sen yaz".
- [x] 1. Concept: CiA 402 states (Switch On Disabled, Ready to Switch On, Switched On, Operation Enabled, Quick Stop Active, Fault), controlword commands (0x06 Shutdown, 0x07 Switch On, 0x0F Enable Operation, 0x02 Quick Stop, 0x00 Disable Voltage, 0x80 Fault Reset on rising edge), statusword bits — with CANviz byte examples — explained 2026-10-26
- [x] 2. Mapping design — decisions K1–K8 approved by the user 2026-10-26, recorded in PROJECT.md › Decisions (CiA 402 state in the bridge, MotorState wins, unexpected stop → Switch On Disabled, Switched On = IDLE + offset measured, 0x2011 keeps stop/calibrate only). Startup flow unchanged; ADC offset moves to Switch On (step 4b)
  Statusword values: SOD 0x0250, RTSO 0x0231, SO 0x0233, OE 0x0237, QSA 0x0217, Fault 0x0218 (bit 4 off when Vdc is out of limits)
- [x] 3. OD: 0x603F error code u16 ro TPDO, 0x6040 controlword u16 rw RPDO, 0x6041 statusword u16 ro TPDO — added by the agent at the user's request 2026-10-26, hand-edited in `OD.c/h`, `.eds`, `.xdd` in CANopenEditor's format (no CANopenEditor in the cloud session) — verified: host `gcc -fsyntax-only OD.c` ✓, host `OD_find` check (objects found, list sorted, 42 entries) ✓, xdd parses as XML ✓; firmware build not run (no Arm toolchain in session, CI pending); user to open the xdd in CANopenEditor once to confirm it loads
- [x] 4. Bridge: decode controlword, run the CiA 402 transitions, build the statusword in `CanopenBridgeUpdate()`; 0x2011 keeps 2 = stop, 4 = calibrate — verified: host `g++ -fsyntax-only` ✓, host simulation of the bridge + real `StateMachine` (19 sequences: 06/07/0F, disable operation, fault + 0x80 edge, quick stop, 0x0F from RTSO, unexpected stop, low Vdc) ✓; firmware build via CI pending; needs HW test
- [x] 4b. Supervisor: measure the ADC offset on Switch On (0x07) while idle (`cmd.offset_cal`, `adc_cal.offset_valid`); START skips the measurement when the offset is valid, otherwise measures as today (Scrutiny start); the offset is used once — verified: host `g++ -fsyntax-only` ✓ (no warnings in App/); firmware build via CI pending; needs HW test
- [x] 5. Fault: map `FaultCode` → CiA 402 error code in 0x603F (e.g. over-current 0x2310, over-voltage 0x3210, under-voltage 0x3220) and send EMCY (COB-ID 0x08D) — `fault_update()` in the bridge: EMCY error bit 0x40 + FaultCode, error reset EMCY when the fault clears; `CO_CONFIG_ERR_CONDITION_GENERIC` in `canopen/CMakeLists.txt` so 0x1001 bit 0 follows motor faults — verified: host `g++ -fsyntax-only` bridge ✓, host `gcc -fsyntax-only CO_Emergency.c` with the new define ✓, host simulation (over-current → 0x603F 0x2310 + EMCY report bit 0x42; 0x80 → 0x603F 0 + EMCY reset) ✓; needs HW test
- [x] 6. Debug build with no new warnings + memory use; CI green on GitHub — built ✓ in CI run 13 (c68262f, Debug + Release green, no compiler warnings; only the known "scrutiny bulunamadı" CMake warning): Debug flash 176.7/512 KB, RAM 66.3/128 KB (+1.2 KB flash vs 318c03f)
- [ ] 7. HW test (user, CANviz): SDO write 0x6040 = 06 → 07 → 0F, read 0x6041 and see the motor run; provoke a fault → statusword fault bit, 0x603F code, EMCY; 0x80 clears it
  2026-10-26 run (user + local agent, python-can SLCAN COM15, sensorless, 55bc8bd): T0–T10 passed — HW verified (user, 2026-10-26): statusword values, SDO aborts 0x06010002 / 0x06070013, disable operation, shutdown, disable voltage, quick stop, 0x0F from RTSO, no restart after unexpected stop, 0x2011 = 1/3 ignored. T11/T12 (fault, EMCY, 0x80 edge) not run (needs Scrutiny). T13 failed: transitions that measure the ADC offset take ~1.1 s.
  Findings: (1) `adcOffsetCalibration()` busy-polls ADC3 regular conversions in the Supervisor thread (prio 3) for ~1.1 s (100 rounds × ~11 ms → one 10 ms poll timeout per round, so one channel's offset is probably wrong); CANopen (prio 5) and USBX (prio 10) starve, SDO/heartbeat stall, Scrutiny USB disconnects. (2) OE → 07 → 0F re-measures at START (offset is single-use), same stall. (3) Sensorless: 0x2001 reads 0 right after STOP (SMO speed is not meaningful with PWM off), so coast-down is not visible on CAN.
- [x] 9. Fix from HW test (user approved M1–M5 2026-10-27, agent wrote the code at the user's request): new `MotorState::INIT` (device init once at power-up + current offset, then IDLE), `ADC_OFFSET_CAL` removed; offset measured in the ADC3 ISR from injected samples with the power stage off (128 samples = 6.4 ms, no busy-wait); refreshed on CiA 402 Switch On; START uses the latest offset and waits only while a refresh runs; START_HALL → CLOSED_LOOP, START_SENSORLESS → ALIGN; bridge maps INIT ↔ Not Ready to Switch On (statusword 0x0000) — verified: host `g++ -fsyntax-only` on all changed files ✓ (no warnings in App/ or motor_control/), host simulation bridge + StateMachine 21 sequences ✓, host StateMachine transition test 16 cases ✓; built ✓ in CI run 16 (c2563ab, Debug + Release green, no compiler warnings): Debug flash 175.5/512 KB, RAM 66.3/128 KB (−1.2 KB flash vs c68262f); needs HW test
  - [x] 9e. HW re-test — HW verified (user + local agent, python-can SLCAN + Scrutiny 0.16 SDK, sensorless, 27f1d0a, 2026-10-27): power-up 0x0000 at 2.7 ms → 0x0250 at 7.5 ms, 0x2002 = 1; heartbeat 1000 ms steady (max 1000.3 ms); 06/07/0F/00 transitions 3.9–5.0 ms, heartbeat not interrupted on 07; offsets 2046/2047/2047, offset_valid = 1, Scrutiny stayed connected; start → 0x2002 2→3→4, ~300 rpm in closed loop; restart 0F 3.9 ms; T11/T12: fault = 5 → 0x0218, 0x603F 0x3220, 0x1001 0x81, EMCY 20 32 81 45; 0x80 edge reset → EMCY 00 00 00 45, held 0x80 does not reset, 00 → 80 resets. T9 (06 during INIT) expectation was wrong: INIT ends ~7.5 ms after boot, before a command can arrive, and the level-triggered controlword then takes SOD → RTSO as CiA 402 specifies.
  Findings: (a) NMT drops to pre-operational and never returns: `NMT_CONTROL` in CO_app_STM32.c uses CO_NMT_ERR_ON_ERR_REG with GENERIC | COMMUNICATION and no CO_NMT_ERR_FREE_TO_OPERATIONAL; SLCAN reconnect (EMCY 0x8120, 0x8110) triggers it, and since c68262f every motor fault does too (generic bit) — would stop TPDOs on a fault in phase 6. Fixed 2026-10-27 (user approved): own `NMT_CONTROL` in canopen/CMakeLists.txt = STARTUP_TO_OPERATIONAL | ERR_ON_ERR_REG | ERR_REG_COMMUNICATION, so motor faults no longer change NMT; ERR_FREE_TO_OPERATIONAL left out because it would undo a master's explicit Enter Pre-Operational (CO_NMT_Heartbeat.c:237) — recovery after comm errors stays with the master (NMT start), revisit in phase 7 — needs CI build + HW test (motor fault keeps NMT operational). (b) Open loop runs at OMEGA_OL = 500 rpm regardless of 0x2010 (by design, app.hpp:38). (c) Restarting sensorless while the rotor still coasts: ALIGN fights the rotation, SMO reads −163 rpm (flying-start topic).
- [x] 8. Commit, update WORK.md / PROJECT.md — commits 318c03f, 0521646, 2663079, c68262f pushed; phase 4 is closed once step 7 passes on the board

## Next
<!-- CANopen roadmap (fixed list, agreed with the user 2026-10-22) -->
1. Persistent calibration (added by the user 2026-10-26): Hall edge table saved to a reserved flash page after CALIBRATING and loaded at boot, like a real drive's commissioning (today it lives in RAM, reset → ideal table). Needs approval for the linker-script flash region; becomes the storage backend for 0x1010 in phase 8
2. CANopen phase 5: modes 0x6060/0x6061 — Profile Velocity (accel/decel ramp 0x6083/0x6084) and Profile Torque (kt ≈ 0.036 Nm/A); sensor mode / calibrate as 0x2xxx objects
3. CANopen phase 6: PDO mapping (RPDO1 controlword + target, TPDO1 statusword + actual) and SYNC; DBC file for CANviz decoding
4. CANopen phase 7: robustness — heartbeat consumer / 0x6007, bus-off recovery, error register 0x1001
5. CANopen phase 8: EDS export and 0x1010 store parameters (uses the flash storage from item 1)
6. Host build for the `tests/` GoogleTest suite, then run it in CI (see Open questions)
Parked by the user: USB command refactor steps 4–6 (command queue, USB parser, telemetry); DC-bus sag and regen/over-voltage control.

## Done
<!-- Newest first, one line each, with evidence label and commit if any. -->
- 2026-10-26 GitHub + CI/CD: private repo `MCanKandamar/stm32g4-foc-motor-drive` (MIT), `build.yml` (Debug+Release matrix, size, .elf/.map artifacts) via PR #1, `release.yml` (`v*` tag → GitHub Release with .elf/.hex/.bin) via PR #2, tag `v0.1.0` on main (dd47c8a); main merged into this branch (a5674eb) — runs reported green by the user (not seen by the agent: private repo, no `gh`)
- 2026-10-25 agent-lite installed (AGENTS.md, CLAUDE.md, docs/WORK.md, docs/PROJECT.md) — built ✓ (Debug: flash 175.4/512 KB, RAM 66.3/128 KB), verified: `cmake --build --preset Debug` ✓ from bash with the bundle PATH

## Dead ends
<!-- date · what was tried · why it failed · when it might be worth retrying -->

## Open questions
<!-- question · impact · default used meanwhile -->
- How should the `tests/` GoogleTest suite be built on the PC (host CMake project or preset)? · motor_control changes cannot be unit-tested by agents · meanwhile: report `not run (no host test build)`
