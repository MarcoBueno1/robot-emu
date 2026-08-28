# Robot Emulator Framework

**A high-performance industrial robot controller emulator written in modern C++, with no heavy runtime dependencies.**

> Status: design · License: MIT · Language: C++23 (GCC 15 / Ubuntu 25.10)

---

## 1. Project Vision

Most open-source "robot simulators" solve only the visual part: a 3D mesh moving in response to commands. That's useful for demos, but it doesn't help develop and test **real control software** — which has to deal with state machines, binary protocols, cycle jitter, sensor faults, and safety limits.

The **Robot Emulator Framework** takes the inverse approach: emulate the *behavior of a real robot controller* — its network interface, its state machine, its sensors, its faults — and treat 3D visualization as an optional consumer, not the core of the system.

**Summary:**

> High-performance C++ framework that emulates the firmware/controller of an industrial robot — deterministic control loop, custom binary protocol, virtual hardware, fault injection, and automated tests — enabling control software to be developed and validated without physical hardware.

### 1.1 What this project **is**
- A **controller behavior** emulator, not a physics engine.
- A library/service that speaks the same "language" (protocol, states, faults) as a real robot.
- A **C++ systems engineering** project: concurrency, soft real-time, binary protocols, low resource footprint.

### 1.2 What this project **is not** (non-goals, at least in the early phases)
- Not a physics simulation engine (no collision, no full rigid-body dynamics).
- Does not depend on ROS, Gazebo, or Webots in the core — those are, at most, optional integrations at the edge.
- Not a GUI. Visualization is a protocol client, not part of the core.

Making this explicit in the README prevents the project from being mistaken for "yet another Gazebo simulator" and keeps the project's scope precise.

---

## 2. Architecture

```
┌───────────────────────────────────────────────┐
│                  Applications                  │
│         Robot SDK / Test Software / CI         │
└───────────────────────┬────────────────────────┘
                         │  TCP / UDP / Unix Socket
┌────────────────────────▼────────────────────────┐
│                 Robot Emulator Core              │
│                                                   │
│  ┌─────────────────────────────────────────┐    │
│  │  Communication Layer (binary protocol)   │    │
│  └─────────────────────┬─────────────────────┘   │
│  ┌─────────────────────▼─────────────────────┐   │
│  │  Controller State Machine                  │   │
│  └─────────────────────┬─────────────────────┘   │
│  ┌─────────────────────▼─────────────────────┐   │
│  │  Motion Controller / Trajectory Planner    │   │
│  └─────────────────────┬─────────────────────┘   │
│  ┌─────────────────────▼─────────────────────┐   │
│  │  Virtual Hardware (motors, encoders, IO)   │   │
│  └─────────────────────┬─────────────────────┘   │
│  ┌─────────────────────▼─────────────────────┐   │
│  │  Robot Model (joints, limits, tool)        │   │
│  └─────────────────────────────────────────┘     │
└───────────────────────────────────────────────────┘
```

Gazebo/Webots/ROS2 **never** enter this central column. They are external consumers of the protocol, at the same level as `robotctl` or a third-party SDK:

```
                        Robot Emulator Core
                                │
              ┌─────────────────┼─────────────────┐
              │                 │                 │
             CLI            TCP/Protocol         ROS2 bridge
         (robotctl)        (external SDK)        (optional)
                                                       │
                                                 ┌─────▼─────┐
                                                 │  Gazebo /  │
                                                 │  Webots    │
                                                 └────────────┘
```

This separation is what guarantees the **very low resource footprint** requirement: the `robot-emulator` binary starts on its own, with no X11, no graphics framework, no third-party runtime.

This decision also mirrors how real industrial robot controllers are actually built: none of them run ROS2 internally — the firmware speaks a proprietary protocol, and ROS2 comes in from the outside as an integration layer (the same role `ros2_control`/*hardware interface* plays for real robots), translating the native protocol into topics/actions. Keeping the core decoupled from DDS avoids inheriting the compatibility cycle between ROS2 distros and keeps the control loop's communication model (synchronous, fixed-cycle) free from the pub/sub paradigm, which is well suited for high-level orchestration, not for the hot path of joint control.

---

## 3. Technical Requirements

### 3.1 Language and standard

**C++23**, aligned with the default toolchain of Ubuntu 25.10 (Questing Quokka), which ships **GCC 15.2** out of the box with full C++23 support (`-std=c++23` / `gnu++23`). This is the most recent stable standard with mature compiler support already provided by the system — no PPA needed, no toolchain to build by hand.

What C++23 offers concretely for this project, beyond what C++17 already provided:

| C++23 feature | Where it helps in Robot Emulator |
|---|---|
| `std::expected<T, E>` | Replaces exceptions/badly-fitted `std::optional` for reporting command validation failure, limit exceeded, sensor fault — error path with no exception cost in the control loop |
| `std::mdspan` | Multidimensional view over joint state buffers (e.g. a `joint × sample` matrix in the telemetry ring buffer) with no extra allocation |
| `std::print` / `std::println` | Formatted output in `robotctl` without classic `iostream` overhead |
| `if consteval` | Distinguishing compile-time vs runtime paths in robot configuration validation |
| `std::ranges` improvements (`views::zip`, `views::enumerate`) | Clean iteration over joint/sensor collections in parallel, without manual index-based loops |
| `[[assume]]` | Annotating control-loop invariants (e.g. `dt > 0`) to help the optimizer at no runtime cost |

**About C++26:** GCC 15 already implements a subset of C++26 in experimental mode (`-std=c++26`), but the specification hasn't been finalized by the ISO committee yet and compiler support is partial. Recommendation: keep the project on **stable C++23** as the build/CI baseline, and isolate any C++26 experimentation behind an optional CMake flag (`ROBOT_EMULATOR_EXPERIMENTAL_CXX26`), never as a requirement of the main build — a public project whose build breaks with every draft change of the standard signals technical insecurity, not the opposite.

**Compiler:** GCC 15 is the system default; **Clang 20** is also available in Ubuntu 25.10's repositories and supports C++23 equivalently — it's worth keeping both in the CI matrix (GCC + Clang) to catch early any code that depends on non-standard behavior of a specific compiler.

### 3.2 Core dependencies
**Core runs with:**
- STL
- POSIX sockets (`epoll` on Linux)
- `std::thread` / `pthread`
- `std::chrono`, `std::atomic`

**Outside the core (forbidden to link into the `robot-emulator` binary):**
Qt, Boost, ROS, Gazebo, Webots, Python, Node.js.

External libraries are added only when they solve a real problem (e.g. a small JSON parsing library for robot configuration files — but that can stay isolated inside a `config/` module, without contaminating `core/`).

### 3.3 Hardware model

```
Robot
 ├── Controller
 ├── Joint × N
 │    ├── Motor
 │    ├── Encoder
 │    ├── Brake
 │    ├── Temperature sensor
 │    ├── Current sensor
 │    └── Limits (position, velocity, acceleration)
 ├── Digital Inputs / Outputs
 ├── Analog Inputs / Outputs
 ├── Emergency Stop
 ├── Safety System
 └── Tool (end effector)
```

Public interface of a joint (the *what*, not the *how*):

```cpp
class Joint {
public:
    double position() const noexcept;
    double velocity() const noexcept;
    double acceleration() const noexcept;
    double torque() const noexcept;

    bool enabled() const noexcept;
    bool homed() const noexcept;
    bool faulted() const noexcept;

    void enable();
    void disable();
    void home();

    void setTargetPosition(double position);
    void setTargetVelocity(double velocity);

    void update(std::chrono::nanoseconds dt);
};
```

Internal representation optimizations (fixed-point, SIMD, SoA vs AoS layout, memory pools) are documented as **future, measurable work** (Phase 11), not as an upfront promise — this avoids committing to a number that hasn't been measured yet. With C++23, iteration over joint collections can use `std::ranges::views::enumerate`/`zip` instead of manual index-based loops, keeping the code expressive without sacrificing performance (the compiler optimizes views the same way it optimizes raw loops).

### 3.4 Controller state machine

```
POWER_OFF → BOOTING → INITIALIZING → IDLE → SERVO_ON → READY
                                                  │
                                    ┌─────────────┼─────────────┐
                                    ▼             ▼             ▼
                                 MOVING        PAUSED      EMERGENCY_STOP
                                    │             │             │
                                    └──────► READY ◄────────────┘
                                                  │
                                              FAULT ──► RECOVERY ──► READY
```

Formal states: `POWER_OFF, BOOTING, INITIALIZING, IDLE, SERVO_OFF, SERVO_ON, READY, MOVING, PAUSED, STOPPING, EMERGENCY_STOP, FAULT, RECOVERY, SHUTDOWN`.

Implemented as an explicit state machine (transition table + guards), not a tangle of `if/else` — this keeps it independently testable and easy to reason about.

### 3.5 Motor/joint model (per-cycle control loop)

```
target position → trajectory planner → velocity controller
                                              │
                                              ▼
                                     acceleration → motor model → joint position
```

```cpp
struct JointState {
    double position;
    double velocity;
    double acceleration;
    double torque;
};

void Joint::update(std::chrono::nanoseconds dt) {
    controller_.update(dt);
    motor_.update(dt);
    encoder_.update(dt);
}
```

### 3.6 Deterministic real-time control loop (soft real-time)

Configurable frequency: `500 Hz | 1 kHz | 2 kHz | 5 kHz | 10 kHz`.

```
┌────────────────────────────┐
│        Control Loop         │
│  Read Sensors                │
│       ↓                      │
│  Update Controller            │
│       ↓                      │
│  Compute Motion               │
│       ↓                      │
│  Update Motors                │
│       ↓                      │
│  Update Encoders              │
│       ↓                      │
│  Safety Check                 │
└─────────────┬────────────────┘
              │
           repeat
```

Metrics collected continuously and exposed via `GET_STATUS`/benchmark: average cycle time, jitter, *deadline misses*, CPU usage, command latency, throughput. This becomes the quantitative material referenced in the Benchmark section (3.13).

> Technical precision note: this is *soft real-time* (the goal is low latency and low jitter, measured and reported), not *hard real-time* with formal guarantees — unless run on a PREEMPT_RT kernel with dedicated CPU affinity. Worth making this distinction explicit in the documentation to avoid promising a guarantee the project doesn't provide.

### 3.7 Communication protocol

Custom binary protocol, versioned from the start:

```
┌────────┬─────────┬────────┬─────────┬─────────┬──────────┐
│ MAGIC  │ VERSION │ FLAGS  │ TYPE    │ LENGTH  │ PAYLOAD   │
└────────┴─────────┴────────┴─────────┴─────────┴──────────┘
```

Commands: `CONNECT, GET_STATUS, ENABLE, DISABLE, HOME, MOVE_JOINT, MOVE_LINEAR, STOP, EMERGENCY_STOP, GET_POSITION, GET_IO, SET_IO, RESET_FAULT, INJECT_FAULT`.

Command flow:

```
Client → MOVE_JOINT → Robot Emulator
                          ├── validate command
                          ├── check safety
                          ├── check limits
                          ├── generate trajectory
                          └── execute
```

The `VERSION` field in the header already enables backward compatibility — important for a protocol intended to be stable enough for external SDKs.

> **Extensibility note (future security, not implemented now):** the `FLAGS` field (bitmask, e.g. 1 byte reserved from the start) and `VERSION` exist precisely to allow, in the future, wiring in an authentication/integrity layer — for example a `FLAG_AUTHENTICATED` bit indicating the `PAYLOAD` is followed by a MAC (HMAC-SHA256, or DUKPT/3DES for scenarios requiring parity with secure banking communication standards) — **without breaking existing clients**: an old client simply never sets the bit and keeps working as it does today. MAC validation, if/when implemented, would enter as an additional step in the flow above (`verify MAC` before `validate command`), isolated in a `protocol/security/` module that **does not exist yet** and is not a dependency of anything — but the frame format won't need to change to accommodate it.

### 3.8 Simulated sensors
- **Encoder**: absolute/incremental, configurable resolution, noise, offset, failure mode.
- **Temperature**: motor, controller, ambient.
- **Current**: motor current, overcurrent.
- **Proximity**: digital sensor.
- **Force/Torque** (later phase): `Fx, Fy, Fz, Tx, Ty, Tz`.

### 3.9 Fault injection

The project's real differentiator over a plain simulator — it's what lets control software be tested against failure scenarios without having to break real hardware.

```
robot-emulator --fault motor-overcurrent --joint 3
```

or via the protocol:

```
INJECT_FAULT
    type = ENCODER_FAILURE
    joint = 2
```

Fault catalog: `ENCODER_FAILURE, MOTOR_FAILURE, OVER_CURRENT, OVER_TEMPERATURE, COMMUNICATION_TIMEOUT, POSITION_ERROR, VELOCITY_ERROR, BRAKE_FAILURE, LIMIT_SWITCH, EMERGENCY_STOP, POWER_FAILURE`.

### 3.10 Declarative robot configuration

No recompilation needed to switch robots:

```
robots/
    industrial_6axis.json
    scara.json
    delta.json
```

```json
{
    "name": "RE-6AXIS",
    "joints": 6,
    "joint_1": {
        "min": -180,
        "max": 180,
        "max_velocity": 2.0,
        "max_acceleration": 5.0
    }
}
```

### 3.11 CLI (`robotctl`)

```
$ robotctl connect 127.0.0.1:9000
$ robotctl status

Robot: RE-6AXIS
State: READY

Joint   Position   Velocity   Temperature
------------------------------------------
J1      12.31°      0.00       31.2°C
J2      -8.42°      0.00       29.8°C
J3      43.11°      0.00       32.4°C
J4       0.00°      0.00       28.9°C
J5      15.20°      0.00       29.1°C
J6      90.00°      0.00       30.0°C

$ robotctl move-joint 1 90
```

### 3.12 Automated testing

```
tests/
├── protocol/
├── controller/
├── motion/
├── joints/
├── sensors/
├── safety/
├── faults/
└── integration/
```

```cpp
TEST(Joint, DoesNotExceedMaximumPosition) {
    Joint joint(config);
    joint.setTargetPosition(500.0);
    joint.update(dt);
    EXPECT_LE(joint.position(), 180.0);
}
```

Recommendation: **GoogleTest** from the start. Writing a custom test framework to "reduce dependencies" costs more engineering time than it saves — GoogleTest is header-friendly, widely accepted, and doesn't compromise the goal of a lightweight *runtime* (it's a build/test-only dependency, never linked into the final binary).

### 3.13 Benchmark

Real, measured numbers — never estimated — published in the README:

```
Robot Emulator Benchmark
-------------------------
CPU: AMD Ryzen 7 | OS: Ubuntu 25.10 | Compiler: GCC 15.2 (-std=c++23, -O3) | Build: Release
Control frequency: 1 kHz

Joints:              6
Sensors:             24

CPU:                 0.8%
Memory:              7.2 MB
Average cycle:       31 µs
P99 cycle:           44 µs
P99.9 cycle:         58 µs
Deadline misses:     0
```

### 3.14 Visualization (optional, after the core)

```
Robot Emulator ──TCP──► Robot Viewer (separate process)
```

Candidates for later: OpenGL, SDL, Vulkan, WebSocket + Web UI, bridge to Gazebo/Webots — all as external protocol clients, never linked into the core.

### 3.15 Functional Safety & MISRA C++

The robot's safety system (E-stop, limits, watchdog — sections 3.4 and 3.9) is described so far in terms of **behavior**. This section defines the **engineering rigor** behind it — what separates "I know how to model an E-stop" from "I know how to model an E-stop in a way that would survive a safety-critical audit." This matters most in domains where a silent failure is not an option, such as industrial and safety-critical firmware.

**Important to state explicitly in the README:** the project **is not certified** and **does not claim formal compliance** with ISO 26262 or with the full text of the MISRA C++ standard (which is licensed and cannot be reproduced or "fully implemented" without the MISRA association). What the project does is **adopt a subset of publicly known, tool-verified practices and rules** as an engineering discipline — an honest choice, common among open-source projects that want to demonstrate rigor without claiming a certification they don't hold.

**Practices adopted:**

| Practice | How it's applied in the project |
|---|---|
| No dynamic allocation on the control path (`Joint::update`, `Robot::update`) | Already a Phase 1 requirement (see the "Definition of Done" in the phase design doc); validated with `valgrind --tool=massif` in CI |
| No exceptions in the control core | Reinforces the decision already made to use `std::expected` instead of `throw` (see Phase 1 design) — a classic MISRA-like rule (no `throw` outside the I/O boundary) |
| No `goto`, no dangerous implicit conversions, no unchecked pointer arithmetic | Verified by automated static analysis (below), not by manual review |
| Every variable explicitly initialized; no UB from reading uninitialized memory | Sanitizers (`-fsanitize=address,undefined`) are already part of the Debug build (Phase 1) — they complement, not replace, static analysis |
| Small functions, single responsibility, bounded cyclomatic complexity | Measured with `lizard` or `pmccabe` in CI, with a configurable limit (e.g. CCN ≤ 15) |
| Explicit `const`-correctness and `noexcept` where applicable | Already an adopted convention since Phase 1 (`Joint`/`Robot`) |
| Safety watchdog independent from the main control loop | Already anticipated in section 3.4 (`FAULT` state); here it gains the additional requirement of running on a separate thread, so a control-loop lockup doesn't prevent fault detection |

**CI verification tools:**

```
cppcheck --enable=all --addon=misra src/ include/
clang-tidy --checks='cppcoreguidelines-*,bugprone-*,misc-*' src/
lizard src/ --CCN 15
```

`cppcheck` with the MISRA add-on identifies a subset of MISRA C++ rules in an automated, free way (without needing the license for the official standard's text) — sufficient for the engineering-discipline purpose of an open-source project, without claiming formal certification.

**Where this fits in the roadmap:** as a cross-cutting phase, not an isolated item at the end — `cppcheck`/`clang-tidy`/`lizard` checks enter CI **starting from Phase 1** (running against the already-existing `robot_core`) and stay valid for all subsequent phases, with the accumulated compliance report versioned in `docs/safety-report.md`.

**Positioning statement for the README:**

> The control core follows a subset of practices aligned with MISRA C++ and ISO 26262 principles (no dynamic allocation or exceptions on the control path, bounded cyclomatic complexity, automated static verification), without claiming formal certification — reflecting mission-critical systems engineering discipline applied to an open-source project.

---

### 3.16 ROS2 Bridge (future, optional phase — not implemented now)

As established in section 2, ROS2 never enters the core. When this phase is implemented, the adopted alternative is the one with the **highest real value**, not the fastest to prototype: a **`ros2_control` plugin**, not a simple topic bridge.

**Why this and not a generic topic bridge:** a bridge that only publishes `sensor_msgs/JointState` and subscribes to a command topic solves visualization (RViz), but doesn't integrate with anything in the real planning ecosystem (MoveIt, standard trajectory controllers). A `ros2_control` plugin makes the emulator treated exactly the way `ros2_control` treats a real UR or Fanuc — any standard controller (`joint_trajectory_controller`, `position_controllers`) works against the emulator with no additional code. It's the same role official vendor drivers play today, which makes the project a plausible testing tool for real robotics stacks, not just a standalone demo.

```cpp
// bridges/ros2_bridge/include/robot_emulator_ros2/hardware_interface.hpp
class RobotEmulatorHardwareInterface : public hardware_interface::SystemInterface {
public:
    // Connects to robot-emulator via the binary protocol from section 3.7 —
    // exactly like robotctl does. The core has no idea ROS2 exists on the other end.
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    // GET_STATUS over the binary protocol → fills state_interfaces_
    hardware_interface::return_type read(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

    // command_interfaces_ → MOVE_JOINT/SET_IO over the binary protocol
    hardware_interface::return_type write(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

private:
    RobotEmulatorClient client_;   // thin protocol wrapper, reused from robotctl
};
```

```
┌───────────────┐   binary protocol   ┌──────────────────────────┐  ros2_control  ┌───────────────┐
│ robot-emulator  │◄──────TCP─────────►│ RobotEmulatorHardware     │◄───interfaces──►│ joint_trajectory│
│  (core, zero    │                    │ Interface (SystemInterface)│                │ controller etc. │
│   ROS2           │                    │ only place that knows ROS2│                │ (ROS2 standard) │
│   dependency)    │                    └──────────────────────────┘                └───────────────┘
```

**Build isolation — `rclcpp` never contaminates the core:**

```cmake
option(ROBOT_EMULATOR_BUILD_ROS2_BRIDGE
       "Build optional ROS2 bridge (requires ROS2 installed; OFF by default)" OFF)

if(ROBOT_EMULATOR_BUILD_ROS2_BRIDGE)
    find_package(rclcpp REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(pluginlib REQUIRED)
    add_subdirectory(bridges/ros2_bridge)
endif()
```

Anyone without ROS2 installed builds the whole project normally — the flag defaults to `OFF` and the absence of ROS2 on the system doesn't break `robot_core`, `robot-emulator`, or `robotctl`.

**How this builds on what's already designed:**
- The bridge's `RobotEmulatorClient` reuses the same protocol wrapper `robotctl` already uses (section 3.11) — no new frame-parsing code.
- The reserved `FLAGS` field in the header (section 3.7) allows, if needed, negotiating a continuous state-streaming mode during `CONNECT` (instead of `read()` polling `GET_STATUS` on every `ros2_control` cycle), without breaking the frame format.
- `bridges/ros2_bridge/` directory, alongside `apps/` — visually explicit that this is edge integration, not part of the core.

**Positioning statement for the README:**

> Optional ROS2 integration via a `ros2_control` plugin (`hardware_interface::SystemInterface`), allowing the emulator to be driven by the same standard controllers (`joint_trajectory_controller`, MoveIt) used with real industrial robots — with zero ROS2 or DDS dependency in the emulator core.

---

## 4. Repository Structure

```
robot-emulator/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── CONTRIBUTING.md
├── docs/
│   ├── architecture.md
│   ├── protocol.md
│   ├── robot-model.md
│   ├── performance.md
│   └── safety-report.md      # accumulated cppcheck/clang-tidy/lizard report
├── include/robot/
├── src/
│   ├── core/
│   ├── controller/
│   ├── motion/
│   ├── hardware/
│   ├── sensors/
│   ├── protocol/
│   ├── network/
│   └── safety/
├── apps/
│   ├── robot-emulator/
│   └── robotctl/
├── bridges/
│   └── ros2_bridge/           # optional; build controlled by ROBOT_EMULATOR_BUILD_ROS2_BRIDGE
├── tests/
├── benchmarks/
├── examples/
├── configs/
└── scripts/
    ├── static-analysis.sh    # cppcheck --addon=misra, clang-tidy, lizard
    └── check-headers.sh      # validates presence of the authorship header in every file
```

---

### 4.1 Toolchain in `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.25)
project(robot-emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   # pure -std=c++23, not gnu++23

option(ROBOT_EMULATOR_EXPERIMENTAL_CXX26 "Enable experimental C++26 features (GCC 15)" OFF)
```

Recommended CI: matrix with `gcc-15` and `clang-20` (both available in Ubuntu 25.10's repositories), `Release` and `Debug` builds, sanitizers (`-fsanitize=address,undefined`) in the Debug build.

## 5. Incremental Roadmap

Each phase delivers something that compiles, runs, and is testable — no "big bang" at the end.

| Phase | Deliverable |
|---|---|
| 1 | Core: `Robot` with N `Joint`s (position, velocity, limits) — **static analysis CI (cppcheck/MISRA, clang-tidy, lizard) already active from here on, cross-cutting all subsequent phases** |
| 2 | Controller state machine |
| 3 | Deterministic real-time control loop (1 kHz) |
| 4 | Motion: trajectory, velocity, acceleration |
| 5 | Virtual hardware: motor, encoder, brake, IO |
| 6 | Binary protocol over TCP |
| 7 | `robotctl` CLI |
| 8 | Safety: limits, E-stop, watchdog |
| 9 | Sensors: encoder, temperature, current, force/torque |
| 10 | Fault injection |
| 11 | Performance optimization (measured, with before/after benchmarks) |
| 12 | Optional visualization |
| 13 | *(optional, future)* ROS2 Bridge — `ros2_control` plugin |
| 14 | *(optional, future)* Secure Communication Layer — frame authentication/integrity (see extensibility note in section 3.7) |

---

## 6. License and File Header Policy

### 6.1 License

**MIT License.** This was the deliberate choice among permissive licenses because it best serves the project's goal: **anyone can use the code for any purpose — including commercial use, including inside closed-source proprietary software —** with the sole obligation of keeping the copyright notice and license text in copies of the software.

Quick comparison against the most common alternatives, so the choice is justified rather than arbitrary:

| License | Why it wasn't chosen here |
|---|---|
| **Apache 2.0** | Also permissive and preserves attribution, but adds an explicit patent grant and a change notice (`NOTICE`) — useful for projects with patent litigation risk, unnecessary overhead for this project |
| **BSD-2/3-Clause** | Equivalent in spirit to MIT, but less universally recognized; MIT is the de facto standard in the C++/GitHub ecosystem |
| **GPL/LGPL** | Copyleft — would force anyone redistributing derivative code to also open-source it. This goes **against** the stated goal ("anyone can use it for any purpose") |
| **0BSD / Unlicense** | Permissive to the extreme, but they even drop the requirement to keep the authorship notice — doesn't meet the "keep author credentials" requirement |

The `LICENSE` file (repository root) contains the official MIT text with:

```
Copyright (c) 2026 Marco Bueno
```

and a closing section with the author's contact information.

### 6.2 Mandatory standard header in every code file

Every `.hpp`, `.cpp`, `.cmake`, `CMakeLists.txt`, and script (`.sh`, `.py`) file in the project must start with the following block, right after the file-path identification line:

```cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
```

(in shell/Python scripts, the same block using `#` instead of `//`)

This header has already been applied to every file specified in the Phase 1 technical design (`joint_error.hpp`, `joint_limits.hpp`, `joint_state.hpp`, `joint.hpp`, `robot_config.hpp`, `robot.hpp`, `clock.hpp`, the test files, and `CMakeLists.txt`), serving as the template for all code written in subsequent phases.

**Enforcement recommendation:** add a simple CI check (`scripts/check-headers.sh`, a `grep`/regex validating the block's presence in every `.cpp`/`.hpp` file tracked in `git diff`) to guarantee no new file enters the repository without the header — cheaper to keep automated than to enforce through manual code review.

---

## 7. Recommended Next Step

Before writing any code: finalize the technical design of **Phase 1** — class definitions, public interfaces, thread model, clock/timing strategy, and the initial `CMakeLists.txt` structure. This becomes the repository's first real commit and the foundation the following phases build on.
