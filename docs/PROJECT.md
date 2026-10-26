# Project

## Overview
Motor-drive firmware for a custom STM32G473 board driving a BLDC/PMSM motor through a DRV8350S gate driver. Field-oriented control with the rotor angle from Hall sensors or from a sensorless sliding-mode back-EMF observer; ThreadX RTOS; CANopen node (CANopenNode on FDCAN1) with CiA 402 drive profile in progress; Scrutiny embedded debugger for live variables and datalogging.

## Architecture
Three layers (see `doc/motor_control/README.md`, local only):
- `App/` — owns the hardware and timing: ISR (`isr.cpp`), ADC measurements, `Supervisor` state machine, `StartupSeq` (Hall calibration, open-loop I/f start), `Monitor` (faults), CANopen application and the bridge to the motor (`canopen_app.c`, `canopen_bridge.cpp`: 0x2000–0x2011 objects, CiA 402 state machine on 0x6040/0x6041, fault → 0x603F + EMCY), Scrutiny glue.
- `motor_control/` — hardware-independent C++20 FOC library (transforms, current/speed PI, SVPWM, Hall sensor, SMO observer, PLL, startup, state machine). Never touches registers, HAL or interrupts; works on physical units; the same sources build for the target and for host unit tests (`tests/`).
- `canopen/` — CANopenNode stack, STM32 FDCAN driver (`CANopenNode_STM32/`) and the project's own object dictionary (`od/`: `OD.c/h`, `bldc_drive.eds/.xdd`).
- Third party: `scrutiny-embedded/`, CubeMX-generated `Core/`, `Drivers/`, `Middlewares/`, `AZURE_RTOS/`, `USBX/`.
Rules:
- `motor_control/` stays hardware-free: no HAL, registers, ThreadX or interrupt code; only math on physical units, so it keeps building and testing on the PC.
- Control-loop ISRs stay short: no blocking, no waiting, no dynamic memory and no blocking ThreadX calls inside an ISR.
- No dynamic memory in the firmware (no `malloc`/`new`, no heap-backed containers such as `std::vector`); use static allocation.
- Do not modify third-party code (`canopen/CANopenNode/`, `canopen/CANopenNode_STM32/`, `scrutiny-embedded/`, ThreadX/USBX under `Middlewares/`); adapt in our own layer. An exception needs the user's approval.

## Environment
- Windows; VS Code with the STM32Cube extension (build, ST-Link GDB debug, live watch). STM32CubeIDE 2.2.0 also installed.
- Toolchain: GNU Arm `gnu-tools-for-stm32` 14.3.1 from the STM32Cube bundles (`%LOCALAPPDATA%/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin`), not on the system PATH. CMake + Ninja, presets `Debug` / `Release` (`CMakePresets.json`, toolchain file `cmake/gcc-arm-none-eabi.cmake`).
- Scrutiny CLI (`C:/Program Files (x86)/Scrutiny Debugger`): if found, a post-build step writes `<name>_tagged.elf` and `.sfd`; the board should run the tagged ELF. Without the CLI the build still works.
- Shell builds: put the bundle's `gnu-tools-for-stm32`, `ninja` and `cmake` bin folders on PATH for the command (exact lines in AGENTS.md › Commands). These are the same tools the VS Code extension uses (`cube-cmake`), so the existing `build/` cache stays valid. Verified 2026-10-25: `cmake --build --preset Debug` builds from bash.
- Host tests: GoogleTest sources in `tests/` (gtest fetched by CMake), but there is no host-side CMake project or preset that builds them yet; a host compiler (WinLibs MinGW g++) is installed. See WORK.md › Open questions.
- Git hosting: private GitHub repo `MCanKandamar/stm32g4-foc-motor-drive` (remote `origin`, MIT licence). Changes reach `main` through pull requests (merge commits). `gh` CLI is not installed locally. The agent-lite files (`AGENTS.md`, `docs/`) exist only on `feature/canopen-cia402` until it is merged into `main`.
- CI/CD (GitHub Actions, `.github/workflows/`): `build.yml` builds Debug and Release on every push (ubuntu-latest, Arm GNU 14.3.Rel1 via `carlosperate/arm-none-eabi-gcc-action@v1`, Ninja from apt), prints size and uploads `.elf`/`.map` artifacts (14 days); `release.yml` runs on `v*` tags and publishes a GitHub Release with Release `.elf`/`.hex`/`.bin`. Scrutiny is absent in CI, so CI ELFs are not tagged for Scrutiny.
- CAN bench: the user's own STM32F072 SLCAN adapter and the CANviz PC tool (raw frames, SDO, CiA 402 commands); bus 500 kbit/s; drive node-ID 13.

## Hardware & toolchain
- Target: STM32G473VET6 (LQFP100, Cortex-M4F, `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`) on a custom board; gate driver DRV8350S (SPI2), current-sense amplifiers AD8418.
- Toolchain & build: arm-none-eabi-gcc 14.3.1, CMake ≥ 3.22 + Ninja; C11, C++20. Build and size commands: see AGENTS.md › Commands.
- Code generator: STM32CubeMX (`015_BLDC_FOC_RTOS_MotorDrive.ioc`, toolchain CMake, `KeepUserCode=true`). Generated: `Core/`, `Drivers/`, `Middlewares/`, `AZURE_RTOS/`, `USBX/`, `cmake/stm32cubemx/`, `startup_stm32g473xx.s`, `STM32G473xx_FLASH.ld`. Edit generated files only inside `USER CODE` blocks. The root `CMakeLists.txt` is generated once and is free to edit.
- Pins, clocks, peripherals: HSE 24 MHz → PLL → SYSCLK 170 MHz. TIM8 centre-aligned PWM, period 4250 (≈ 20 kHz): high side INH_A/B/C = PC6/PC7/PC8, low side INL_A/B/C = PC10/PC11/PC12. Hall HAL_A/B/C = PA6/PA7/PB0 (TIM3). Phase currents CURR_A/B/C = PD13/PD11/PE8; phase voltages SENS_A/B/C = PD14/PD12/PD10; DC bus SENS_VDC = PA8 (ADC3/4/5). DRV8350S: SPI2, DRV_CS = PB12, DRV_ENABLE = PD15, FAULT_N = PE0. FDCAN1 RX/TX = PD0/PD1. LEDs: CAN_LED = PD2, PROCCESS_LED = PE1. Also CORDIC, DMA, USB.
  The pin, clock and peripheral setup in the `.ioc` is fixed; no other reserved pins or limits recorded (changes need the user's approval, see AGENTS.md › Always ask first).
- RTOS & memory budget: Azure RTOS ThreadX; threads Init (2 KB stack), Supervisor (2 KB), Monitor (1 KB), CANopen (2 KB) in `Core/Src/app_threadx.c`. Flash 512 KB, RAM 128 KB (linker script). Last measured 2026-10-23: Debug flash 175.4/512 KB, RAM 66.3/128 KB; Release flash 101.9/512 KB, RAM 66.2/128 KB.
  No explicit memory budget: report flash/RAM use with every build and flag large jumps. Thread stacks are statically allocated arrays in `app_threadx.c`.
- Debug & flashing setup: ST-Link via VS Code (`.vscode/launch.json`, `stlinkgdbtarget`), loads `build/Release/015_bldc_foc_rtos_motordrive_tagged.elf`. For information only; flashing needs the user's approval every time.

## Decisions & constraints
<!-- Newest first. "YYYY-MM-DD · Decision: … · Why: …" or "YYYY-MM-DD · Constraint: … · Learned from: …".
Mark replaced entries "(superseded by YYYY-MM-DD)" instead of deleting them. -->
- 2026-10-26 · Decision: motor faults reach CANopen as 0x603F (CiA 402 error code: driver nFAULT 0x5000, motor over-current 0x2310, driver over-current 0x2300, over-voltage 0x3210, under-voltage 0x3220, motor over-temp 0x4310, driver over-temp 0x4210, sensor 0x7300, sensorless lost 0xFF01) and one EMCY per change, using CANopenNode error bit 0x40 + `FaultCode` (manufacturer critical range); `CO_CONFIG_ERR_CONDITION_GENERIC` is overridden in `canopen/CMakeLists.txt` so 0x1001 bit 0 is set too · Why: CiA 301 requires the generic bit for any active error; the stack's default only looks at its own bits.
- 2026-10-26 · Decision: CiA 402 state lives in its own variable in `canopen_bridge.cpp`; `MotorState` and `motor_control/` stay unchanged. `MotorState` wins: FAULT → Fault; an Operation Enabled / Quick Stop Active drive that drops to IDLE (stop from Scrutiny or 0x2011, quick stop done) → Switch On Disabled · Why: our `MotorState` has no Ready/Switched On split; falling back to Switch On Disabled stops a controlword left at 0x0F from restarting the motor on its own.
- 2026-10-26 · Decision: CiA 402 mapping — Switched On = IDLE with the ADC offset measured (DRV and PWM still off until 0x0F; deviation from the standard's "power stage on"); Operation Enabled reported as soon as START is accepted (also during ALIGN/OL); 0x0F starts only from IDLE; Quick Stop = coast (`cmd.stop`, no braking ramp); Fault Reaction Active skipped (power is cut at once); statusword bit 4 = Vdc within limits, bit 9 = 1, bits 7/10 = 0 for now · Why: `powerOn()` enables DRV and MOE together; master timeouts on a ~1 s startup; regen braking is parked.
- 2026-10-26 · Decision: ADC current offset is measured on the CiA 402 Switch On transition (0x07) and reused by START; START still measures when no valid offset exists. Startup flow unchanged: Hall ADC → CL, sensorless ADC → ALIGN → OL → CL · Why: offset drifts with temperature so it is measured at every switch-on, like real drives; align stays in sensorless because the SMO sees no back-EMF at standstill (e = kₑ·ω = 0).
- 2026-10-26 · Decision: 0x2011 command keeps only 2 = stop and 4 = calibrate; 1 = start and 3 = reset fault are removed · Why: they would bypass the CiA 402 Shutdown → Switch On → Enable Operation sequence; stopping is always safe.
- 2026-10-26 · Constraint: no communication-loss reaction yet — if the CAN master disappears the motor keeps running until phase 7 (heartbeat consumer / 0x6007) · Learned from: CiA 402 mapping review.
- 2026-10-26 · Decision: CI on GitHub Actions with Linux runners and the same Arm GNU version as the local STM32Cube bundle (14.3) · Why: cheapest runners; matching compiler avoids "builds in CI but not locally" differences.
- 2026-10-23 · Decision: CANopenNode runs in its own ThreadX thread (priority 5) that calls `canopen_app_process()` and `canopen_app_interrupt()` every 1 ms; no TIM7 timer interrupt · Why: user's choice, keeps the stack out of ISRs.

## References
Local only (git-ignored, not available in other clones or cloud agents):
- `doc/motor_control/README.md`, `doc/motor_control/docs/foc.md`, `doc/motor_control/docs/smo.md` — library design, FOC and SMO derivations
- `doc/canopen/docs/` — CANopen guides (PDF)
- `doc/smo_kitap/` — sliding-mode observer booklet
- `.claude/notes/INDEX.md` — datasheet notes: DRV8350S, AD8418, STM32G473 ADC
- `canopen/VERSIONS.md` — CANopenNode versions (tracked)
