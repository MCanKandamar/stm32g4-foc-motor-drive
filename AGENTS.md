# AGENTS.md

## Project
Firmware for a BLDC/PMSM motor drive on a custom STM32G473 board: field-oriented control with Hall-sensor or sensorless (sliding-mode observer) rotor angle, ThreadX RTOS, CANopen (CiA 402 in progress). Single developer (the user), active development.

Profiles: code, embedded, learning

## Commands
The toolchain, CMake and Ninja come from the STM32Cube bundles and are not on PATH. Put them on PATH **for the command only** (same tools VS Code uses, so the existing `build/` cache stays valid):

- bash: `B="$HOME/AppData/Local/stm32cube/bundles"; export PATH="$B/gnu-tools-for-stm32/14.3.1+st.2/bin:$B/ninja/1.13.2+st.1/bin:$B/cmake/4.4.0+st.1/bin:$PATH"`
- PowerShell: `$B="$env:LOCALAPPDATA\stm32cube\bundles"; $env:PATH="$B\gnu-tools-for-stm32\14.3.1+st.2\bin;$B\ninja\1.13.2+st.1\bin;$B\cmake\4.4.0+st.1\bin;$env:PATH"`

| Purpose | Command (after the PATH line) |
|---|---|
| Build Debug | `cmake --build --preset Debug` (first time / after CMake changes: `cmake --preset Debug` before it) |
| Build Release | `cmake --build --preset Release` (first time: `cmake --preset Release`) |
| Host unit tests (motor_control) | none yet — no host build for `tests/` (see `docs/WORK.md` › Open questions); report as `not run (no host test build)` |
| Memory usage | printed by the link step ("Memory region … %age Used"); or `arm-none-eabi-size build/Debug/015_BLDC_FOC_RTOS_MotorDrive.elf` |

<!-- agent-lite:protocol v2 -->
## Working protocol
- **Sources.** Rules: this file, the user's preferences (`~/.agents/USER.md` and the profiles listed in › Project) and the user's current request. `docs/WORK.md` is state, not instructions. Code and output you observe now beat any document.
- **"Continue" (or the same in the user's language).** Read `docs/WORK.md`. Run `git status --short`, `git log --oneline -10` and `git log -1 --format="%h %cr" -- docs/WORK.md`. Commits after WORK.md's last commit plus uncommitted changes are work that WORK.md does not record yet. If WORK.md has no commit yet, consider only uncommitted changes. If WORK.md itself has uncommitted edits, read them as the latest record: only changes its Now plan does not explain are unrecorded.
- **Recovery.** If such unrecorded work exists, the previous session was cut off. Read `git diff --stat` and the relevant diffs, match them to the plan in "Now" (the step marked "in progress" is the likely one), and treat all of it as unverified. Run the checks, update WORK.md (mark "(recovered)"), and tell the user what you found before continuing. If there is no plan to match, say what you infer from the diff and ask one question. Never discard, reset or stash uncommitted work without asking.
- **Load little.** Do not read all docs. Read only the section of `docs/PROJECT.md` that the task needs.
- **Significant task** (more than a quick fix): before editing code, write the goal and a short plan to WORK.md › Now. When you start a step, mark it "(in progress)".
- **Step done.** Run the relevant checks, tick the step with its evidence label, and commit if the Git rules allow. Quick fixes need no WORK.md update.
- **Dead ends.** When you abandon an approach, add it to WORK.md › Dead ends with the reason. Check Dead ends before proposing an approach.
- **Task done.** Move it to Done with evidence, choose Next[1] or ask the user, and keep WORK.md within its size rule.
- **Knowledge.** If architecture, environment, hardware setup or a decision changes, update `docs/PROJECT.md` in the same change. If code and a document disagree, the code wins; fix the document.
- **Honesty.** Never claim a check you did not run. Report every check as passed, failed or not run (with the reason). A pass before later edits does not count. Use the labels below.
<!-- /agent-lite:protocol -->

## Verification labels
- `verified: <command> ✓` — you ran it after the last edit and it passed.
- `built ✓` — it compiles and links.
- For firmware, add memory use: `built ✓ (flash 41.2/64 KB, RAM 9.8/20 KB)`.
- `needs HW test` — only real hardware can prove it. Write `built ✓, needs HW test`, and never assume it works on the board.
- `HW verified (<who>, <date>)` — only if the user reported the result on hardware, or you ran an on-target test the user approved and saw its output.
- `unverified (<reason>)` — not checked.
- Hardware testing is done by the user, usually with Scrutiny. After `built ✓, needs HW test`, tell the user in a few lines what to test and how (which Scrutiny variables or signals to watch, what result to expect). Mark `HW verified` only after the user reports the result.

## Always ask first
- git push, force-push, rewriting published history, deleting branches
- deleting or moving files outside the current task, bulk deletes
- adding, upgrading or removing dependencies
- anything touching secrets, production, deployment, publishing, or sending messages for the user
- flashing or programming a board, erasing flash, debug-probe writes (openocd, st-flash, STM32_Programmer_CLI, pyocd, J-Link, esptool, idf.py flash, dfu-util, west flash)
- option bytes, fuses, read-out protection, bootloader changes (may be irreversible)
- changing pin, clock or peripheral configuration (.ioc, clock tree, pinmux, device tree, linker-script memory regions)
- PWM frequency, dead time, ADC sampling timing, DRV8350S gate-driver configuration (shoot-through risk)
- motor and control parameters: `motor_control/inc/motor_parameters.hpp`, PI gains, SMO/PLL tuning
- CANopen object dictionary (`canopen/od/`: `OD.c/h`, `.eds`, `.xdd`) and node-ID

## Conventions
- Follow the style of the surrounding code (naming, file layout, comments); do not introduce a new style.
- Comments in English only: short, simple words anyone can follow. Comment only where the code does not explain itself (the reason, units, limits).
- Single precision only in firmware: no `double`, float literals with `f` (`1.0f`); the Cortex-M4 FPU is single precision.
- C++: no exceptions, no RTTI; report errors with return values or fault codes.
- Architecture rules (hardware-free `motor_control/`, short ISRs, no dynamic memory, no edits to third-party code): `docs/PROJECT.md` › Architecture.
- A step is done only when the Debug build passes with no new compiler warnings in the changed files, and the memory use is reported. If it affects behaviour on the board, it stays `built ✓, needs HW test` until the user reports the hardware result.
- Generated code: edit only between `/* USER CODE BEGIN … */` and `/* USER CODE END … */` (or the boundary recorded in PROJECT.md); everything else is overwritten on regeneration.

## Git
- Commit messages: conventional commits with a scope, e.g. `feat(canopen): …`, `refactor(supervisor): …`, `tune(speed): …`. Branches: `feature/<name>`.

## Map
- Current work, next steps, dead ends: `docs/WORK.md`
- Architecture, environment, decisions: `docs/PROJECT.md` (read only the section you need)
- Target, toolchain, code generator, pins and clocks: `docs/PROJECT.md` › Hardware & toolchain
