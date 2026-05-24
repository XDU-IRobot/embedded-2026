# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

XDU-IRobot RoboMaster 2026 season embedded firmware, targeting STM32F407IGHx (Cortex-M4F, 168 MHz). Build system is CMake with arm-none-eabi-gcc toolchain. C11/C++20, hard FPU.

## Build Commands

```bash
# Configure (first time or after CMakeLists changes)
cmake --preset Debug

# Build a specific target
cmake --build build/Debug --target <target_name> -j

# Build all targets
cmake --build build/Debug -j

# Release build
cmake --preset Release
cmake --build build/Release --target <target_name> -j
```

**Available targets:** `steer_infantry_cs`, `steer_infantry_gb`, `omni_target`, `drone_gb`, `drone_gb_new`, `old_sentry`, `test_gb`, `dart_rack`, `algorithm_enroll_test`

## Flash / Debug

```bash
# Flash via ST-Link
openocd -f openocd/stlink.cfg -c "program build/Debug/<target_name>.elf verify reset exit"

# Or via CMSIS-DAP
openocd -f openocd/cmsisdap.cfg -c "program build/Debug/<target_name>.elf verify reset exit"
```

## Lint & Format

```bash
# clang-format (Google-based, 120 cols, C++17, Right pointer alignment)
clang-format -i app/**/*.cc app/**/*.hpp app/**/*.h
```

CI auto-runs clang-format on push and auto-commits fixes. clang-tidy is configured but currently disabled in CI.

## Architecture

**Non-invasive CubeMX pattern**: STM32CubeMX generates `Core/` (HAL init) and `USB_DEVICE/` — these are minimally modified. The only change to generated code is calling `AppMain()` from `main.c`. All user code lives in `app/`.

```
main.c (CubeMX) → AppMain() → target-specific startup
```

**`app/` directory structure:**
- `app/common/` — shared modules used across all targets (controllers, kinematics solvers, LED/buzzer drivers, CAN communication helpers, timer abstractions)
- `app/targets/<name>/` — per-robot code. Each is a separate CMake executable. A `-D<NAME>` define is set per target for conditional compilation.

**`libs/librm/`** (git submodule) — internal cross-platform robotics library providing:
- `device/` — DJI motor drivers (M2006, M3508, M6020), IMU drivers (BMI088, IST8310), referee system protocol, RC receiver (DR16/SBUS)
- `modules/` — PID controller, AHRS (Mahony/EKF), DSP filters, trajectory limiting, chassis kinematics, power modeling
- `hal/` — STM32/Linux hardware abstraction (CAN, SPI, I2C, UART, GPIO)
- `core/` — base types, threading, time, error handling

**Key controllers** (in `app/common/controllers/`): quad steering chassis, quad omni chassis, 2-DOF gimbal, double-yaw gimbal, 2/3-friction-wheel shooter.

## Branch Convention

- `main` — protected, always buildable
- `target/<name>` — one branch per robot target
- Workflow: `git merge main` → resolve conflicts → develop → PR back to `main`

## Adding New Code

- New shared module: create directory under `app/common/`, add sources to `app/CMakeLists.txt`
- New target: create directory under `app/targets/<name>/`, add `add_exe_target()` call in `app/CMakeLists.txt`

## CubeMX Regeneration

When regenerating from `boardc.ioc` (CubeMX 6.15.0, STM32Cube FW_F4 V1.28.3), only `Core/` and `USB_DEVICE/` are overwritten. Re-apply the `AppMain()` call in `Core/Src/main.c` after regeneration.
