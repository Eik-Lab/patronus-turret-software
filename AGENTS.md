# Patronus Turret Software — Agent Guide

This file governs how AI coding assistants operate on this project.

---

## Project Overview

Real-time anti-drone turret control system running on **NVIDIA Jetson AGX Thor** (aarch64) and x86_64 dGPU. Two Basler cameras feed parallel DeepStream 8.0 / YOLOv8 pipelines; detections are consumed by a tracking loop that drives CAN bus motors via the CANdle-SDK.

| Aspect | Detail |
|--------|--------|
| Language | **C++17** (`CMAKE_CXX_STANDARD 17`) |
| Build | **CMake 3.15+** — `cmake -B build && cmake --build build` |
| GPU | Blackwell sm_11.0, CUDA 13.0, DeepStream 8.0 |
| Coding Standard | **C++ Core Guidelines** ([isocpp.github.io/CppCoreGuidelines](https://isocpp.github.io/CppCoreGuidelines)) |
| Formatter | `.clang-format` (K&R layout, 2-space indent, 100 col limit) |
| Linter | `.clang-tidy` — enforced via `CMAKE_CXX_CLANG_TIDY` (set `-DCMAKE_CXX_CLANG_TIDY=OFF` to bypass) |

---

## Build & Development

```bash
# Configure (Debug)
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CUDA_ARCHITECTURES=native

# Configure (Release)
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=native

# Build
cmake --build build -j$(nproc)

# Lint (if not running via CMake)
clang-tidy src/**/*.cpp --extra-arg=-std=c++17 -Iinclude

# Format (in-place)
clang-format -i src/**/*.cpp include/**/*.hpp
```

**Thor-specific flags:**
- `-DCMAKE_CUDA_ARCHITECTURES=110` (sm_11.0 Blackwell; use `native` for auto-detection)
- Set `CUDACXX=/usr/local/cuda-13.0/bin/nvcc` if not in `PATH`
- DeepStream at `/opt/nvidia/deepstream/deepstream-8.0/`
- CPU: 14-core Arm Neoverse-V3AE (aarch64) — CMake detects this automatically

---

## Code Style — C++ Core Guidelines

Follow the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines). Key rules:

### Naming (NL.8–NL.10)

| Entity | Convention | Example |
|--------|-----------|---------|
| Namespace | `lower_case` | `patronus::core` |
| Class / Struct / Enum | `CamelCase` | `DetectionState`, `AimAngles` |
| Function | `lower_case` | `compute_center`, `serial_open` |
| Variable (local, global, parameter) | `lower_case` | `frame_number`, `detections_` |
| Class member | `lower_case` with `_` suffix | `mutex_`, `detections_` |
| Constant / constexpr | `lower_case` | `k_weapon_dx` or `weapon_dx` |
| Macro | `UPPER_CASE` only | `PLATFORM_TEGRA` |
| Template parameter | `CamelCase` | `typename T`, `typename ValueType` |

- **NL.5**: Do not encode type information in names (no Hungarian notation).
- **NL.7**: Name length proportional to scope — short names (`i`, `T`) OK in small scopes.
- **NL.9**: `ALL_CAPS` is reserved for macros. Constants use `lower_case`.

### Layout (NL.17)

- **K&R braces**: opening brace on the same line for functions, control flow, classes.
- **Indentation**: 2 spaces, no tabs.
- **Column limit**: 100 characters.
- Always run `clang-format` before committing.

### Class Member Order (NL.16)

```
class MyClass {
 public:
  // constructors, assignments, destructor
  // functions
  // data members (public)
 protected:
  // ...
 private:
  // member variables (trailing underscore_)
};
```

Access specifiers appear once each, in `public` → `protected` → `private` order.

### Includes

Order (separated by blank lines):
1. Associated header (e.g., `"patronus/core/state.hpp"` for `state.cpp`)
2. Project headers (`"patronus/..."`)
3. Library headers (`<gst/gst.h>`, `<nvds_meta.h>`, `<cuda_runtime.h>`)
4. System/STL headers (`<vector>`, `<mutex>`, `<thread>`)

### Namespaces

Wrap `.hpp` and `.cpp` content in the project namespace after includes:

```cpp
namespace patronus::module_name {

// ... code ...

}  // namespace patronus::module_name
```

- No `using namespace` directives.
- Use `using` declarations sparingly and only in `.cpp` files.

### C++17 Features — Use Generously

Prefer C++17 idioms per C++ Core Guidelines:
- `std::optional<T>` over sentinel values
- `if constexpr` for compile-time branching
- Structured bindings
- `std::string_view` for read-only string parameters
- `[[nodiscard]]` on functions whose return value must not be ignored
- `std::variant<T>` over unions

**Explicit types over `auto`**: Prefer explicit type names (`static_cast<T>`, named types) over `auto` for clarity and self-documentation. Use `auto` only when the type is obvious from context (e.g., iterators, lambda captures).

---

## Code Documentation — Doxygen

Use Doxygen triple-slash style (`///`) for all public/interface declarations,
per C++ Core Guidelines
[NL.1–NL.3](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rl-comment):

| Rule | Meaning |
|------|---------|
| NL.1 | Don't comment obvious code |
| NL.2 | State intent in comments |
| NL.3 | Keep comments crisp |

### Public / Interface Headers (`include/patronus/*.hpp`)

All declarations in public headers MUST have Doxygen blocks:

```cpp
/// @brief Transforms sensor-frame coordinates to weapon-frame angles.
/// @param center_x Normalized x coordinate in sensor frame [0, 1].
/// @param center_y Normalized y coordinate in sensor frame [0, 1].
/// @param distance_mm Target distance in millimetres.
/// @return AimAngles with compensated pan and tilt.
AimAngles compute_aim(float center_x, float center_y, float distance_mm);
```

Required tags: `@brief`, `@param` (unless obvious), `@return` (unless void),
`@tparam` (templates), `@note` / `@warning` (non-obvious behaviour).

### Internal / Implementations (`.cpp` files, private members)

Use plain `//` intent comments (NL.2). No Doxygen tags required.

```cpp
// Compute weapon offset using pre-calibrated lever-arm parameters
float dx = aim.pan * k_weapon_dx;
```

### AI Agents

Any public API created or modified by an AI agent must include the corresponding
Doxygen block before committing.

---

## Architecture Reference

```
Camera (mono) ──► DeepStream Pipeline ──► Detection Queue ──┐
                                                             ├──► Tracking Loop ──► CAN/Motors
Camera (RGB)  ──► DeepStream Pipeline ──► Detection Queue ──┘
                                                             │
                                                             └──► H.264 UDP Stream
```

**Threads** (2 + N): RGB pipeline, mono pipeline, and N tracking loops (one per gimbal).
**IPC**: `LatestValue<T>` (mutex + condition variable) for thread-safe detection transfer.

Key source layout:

| Path | Role |
|------|------|
| `include/patronus/core/` | Geometry types, thread-safe state |
| `include/patronus/comm/` | CAN bus motor driver (CANdle-SDK) |
| `include/patronus/tracking/` | Aim control (P-controller, coordinate transforms) |
| `include/patronus/pipeline/` | DeepStream pipeline declarations |
| `src/pipeline/` | DeepStream GStreamer pipeline implementations |
| `src/tracking/` | Aim control math |
| `src/comm/` | CAN bus motor driver implementation |
| `src/main.cpp` | Entry point, thread orchestration |
| `deps/CANdle-SDK/` | CAN bus motor SDK |
| `config/deepstream/` | YOLOv8 inference configs |

---

## Operational Rules

1. **No git branches, PRs, or Issues** — never create, propose, or modify these. Work only on the current branch.
2. **No commits without user confirmation** — stage and present changes; do not commit until explicitly told.
3. **No AI-generated GitHub Issues** — do not open, close, or reference them.
4. **No file deletions** unless explicitly requested.
5. **Never modify submodule files**.
6. **Never commit secrets, keys, or credentials.**

### Workflow

1. Read the relevant files to understand context.
2. Follow the C++ Core Guidelines and `.clang-format` / `.clang-tidy`.
3. Run `cmake --build build` to verify compilation after changes.
4. Run `clang-tidy` on changed files if possible.

---

## Performance Budget

This is a **real-time control system**. Violations of these rules degrade responsiveness and can cause hardware damage.

### Hot Path Rules

The **tracking loop** (one per gimbal) and **motor commands** are the hot path:

1. **No heap allocations in the tracking loop** — no `new`, `delete`, `std::vector` resize, `std::string` construction, or `std::map` insertion.
2. **No blocking I/O in the tracking loop** — motor commands must return within 1 ms.
3. **Lock duration < 100 μs** — use `LatestValue<T>::try_pop()` with short timeouts, never `wait()`.
4. **No exceptions in hot path** — use `std::optional` or error codes instead.
5. **Avoid `std::mutex` contention** — prefer `LatestValue<T>` (single-producer, single-consumer) over shared mutexes.

### Motor Safety

1. **Motor command timeout** — if tracking loop hasn't sent a command in `home_return_delay_ms`, motors return to home automatically.
2. **Velocity clamping** — never exceed `max_velocity_rad_s` from config. The P-controller must clamp output.
3. **Graceful shutdown** — on SIGTERM/SIGINT, send `set_velocity(0, 0)` to all motors before exit.
4. **Init before use** — `CandleMotor::init()` must succeed before any motor command.

### Thermal Limits

- Monitor GPU/CPU temperature via `tegrastats`.
- If temperature exceeds 85°C, reduce inference frame rate or batch size.
- The Thor throttles at 90°C — stay below this to avoid performance cliffs.

---

## Error Handling

### Convention

- **Recoverable errors**: `std::optional<T>` or `bool` return. Log and continue.
- **Unrecoverable errors**: `std::abort()` after logging. Never silently fail.
- **No raw exceptions in public API** — use `std::expected<T, E>` (C++23) or `std::optional<T>`.
- **Resource cleanup**: RAII everywhere. No manual `new`/`delete`.

### Specific Rules

| Scenario | Rule |
|----------|------|
| CAN bus disconnect | Retry 3× with 100 ms delay, then disable motors and log error |
| Camera disconnect | Log warning, continue with other camera if available |
| Config file missing | Print usage and exit with code 1 |
| GPU out of memory | Reduce batch size, retry, then abort |
| Signal (SIGTERM/SIGINT) | Set `running = false`, wait for threads to exit, disable motors |

---

## Testing

### Unit Tests

- Framework: Google Test (if added) or Catch2
- Location: `tests/` directory
- Run: `./build/tests` after build

### Integration Tests

- Motor tests: `./build/calibrate_home --test-motors`
- Pipeline tests: `./build/inference --rgb-only` or `--mono-only`

### What to Test

1. **Tracking math** — `compute_control_weapon()` output for known inputs
2. **Config loading** — INI file parsing with missing/invalid values
3. **Motor commands** — velocity clamping, home return, timeout behavior
4. **Edge cases** — empty detection queues, zero gimbals, division by zero in transforms

### Test Conventions

- Test files: `*_test.cpp` or `test_*.cpp`
- One test per behavior
- Name tests descriptively: `TEST(Tracking, ZeroErrorProducesZeroVelocity)`
- No `ASSERT_EQ` in loops — use `EXPECT_EQ` for non-fatal checks

---

## Logging

### Levels

| Level | Use |
|-------|-----|
| `g_debug()` | Verbose diagnostic info (disabled in release) |
| `g_print()` | Normal operation messages |
| `g_warning()` | Recoverable errors (camera disconnect, config fallback) |
| `g_critical()` | Unrecoverable errors (motor init failed, CAN bus lost) |

### Rules

1. **No logging in hot path** — the tracking loop runs at ~100 Hz; logging every frame kills performance.
2. **Log state transitions** — when tracking state changes (KTracking → KReturningHome → KAtHome), log once.
3. **Log errors immediately** — don't batch error messages.
4. **Include context** — always include gimbal index, motor ID, or component name in messages.

---

## Signal Handling & Graceful Shutdown

The system must handle SIGTERM and SIGINT (Ctrl+C) gracefully:

1. Set `running = false` (atomic bool) to signal all threads.
2. Pipeline threads exit their GStreamer main loops.
3. Tracking threads exit their loops and send `set_velocity(0, 0)` to motors.
4. Motor driver calls `CandleMotor::disable()` to cut PWM and detach CANdle.
5. Join all threads before exit.

```cpp
// In main.cpp
std::atomic<bool> running{true};

void signal_handler(int) {
  running = false;
}

std::signal(SIGTERM, signal_handler);
std::signal(SIGINT, signal_handler);
```

---

## Git Conventions

- **Commit messages**: [Conventional Commits](https://www.conventionalcommits.org/)
  - `feat:` new feature
  - `fix:` bug fix
  - `refactor:` code restructuring
  - `perf:` performance improvement
  - `chore:` maintenance, build, dependencies
  - `docs:` documentation
  - `style:` formatting only
- Scope optional: `feat(tracking): add lead compensation`

---

## Platform: NVIDIA Jetson AGX Thor

| Spec | Value |
|------|-------|
| GPU | Blackwell, 2560 cores, 96 fifth-gen Tensor Cores |
| Compute Capability | **sm_11.0** → `CMAKE_CUDA_ARCHITECTURES=110` |
| CUDA | 13.0 (`/usr/local/cuda-13.0/`) |
| DeepStream | 8.0 (`/opt/nvidia/deepstream/deepstream-8.0/`) |
| CPU | 14× Arm Neoverse-V3AE @ ~2.5 GHz |
| Memory | 128 GB LPDDR5X, 273 GB/s |
| Power | 40–130 W (set via `nvpmodel`) |
| NVENC | 2× encoders → hardware H.264/H.265 |
| MIG | Multi-Instance GPU with 10 TPCs available |
| Networking | 1× 5GbE RJ45, 1× QSFP28 (4× 25GbE) |

**Optimization notes:**
- Use `nvpmodel -m 0` for MAXN performance (130 W).
- Use `tegrastats` for runtime power/thermal monitoring.
- Prefer TensorRT FP16/INT8 for YOLOv8 inference.
- Hardware video encoding via NVENC is preferred over software x264enc.
- The codebase already detects `aarch64` vs `x86_64` in `src/CMakeLists.txt`.

---

## Runtime & Deployment

### Starting the System

```bash
# Full inference + tracking (requires cameras + CAN bus)
./build/inference -c config/system.ini

# RGB pipeline only (1 camera)
./build/inference -c config/system.ini --rgb-only

# Mono pipeline only (1 camera)
./build/inference -c config/system.ini --mono-only

# Motor test (no cameras needed)
./build/inference -c config/system.ini --test-motors
```

### Prerequisites

- NVIDIA DeepStream 8.0 installed at `/opt/nvidia/deepstream/deepstream-8.0/`
- CANdle USB device connected (check `ls /dev/can*`)
- Basler cameras connected (check `ls /dev/video*`)
- CAN bus termination resistor installed (120Ω)

### Configuration

INI files in `config/`:

| File | Purpose |
|------|---------|
| `config/system.ini` | Main system config (mode, gimbals, CAN bus) |
| `config/deepstream/*.txt` | YOLOv8 inference engine configs |

### Monitoring

```bash
# GPU/CPU temperature and power
tegrastats

# CAN bus traffic (if can-utils installed)
candump can0

# Motor status (via calibrate_home tool)
./build/calibrate_home
```

---

## Reading Order for New Contributors

1. `include/patronus/core/types.hpp` — core geometry
2. `include/patronus/core/state.hpp` — thread-safe state
3. `src/main.cpp` — application structure and threads
4. `include/patronus/pipeline/inference_mono.hpp` — pipeline API
5. `include/patronus/pipeline/inference_rgb.hpp` — RGB pipeline API
6. `include/patronus/tracking/aim_control.hpp` — control math
7. `include/patronus/comm/candle_motor.hpp` — CAN bus motor driver
8. `src/tools/calibrate_home.cpp` — home calibration tool
