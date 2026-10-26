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
- [ ] 4. Bridge: decode controlword, run the CiA 402 transitions, build the statusword in `CanopenBridgeUpdate()`
- [ ] 4b. Supervisor: measure the ADC offset on Switch On (0x07) while idle (`cmd.offset_cal`, `adc_cal.offset_valid`); START skips the measurement when the offset is valid, otherwise measures as today (Scrutiny start)
- [ ] 5. Fault: map `FaultCode` → CiA 402 error code in 0x603F (e.g. over-current 0x2310, over-voltage 0x3210, under-voltage 0x3220) and send EMCY (COB-ID 0x08D)
- [ ] 6. Debug build with no new warnings + memory use; CI green on GitHub
- [ ] 7. HW test (user, CANviz): SDO write 0x6040 = 06 → 07 → 0F, read 0x6041 and see the motor run; provoke a fault → statusword fault bit, 0x603F code, EMCY; 0x80 clears it
- [ ] 8. Commit, update WORK.md / memory of the roadmap

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
