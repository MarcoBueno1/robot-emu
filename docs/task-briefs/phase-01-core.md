# Phase 01 — Core (`Robot` + `Joint`)

**Status:** Done
**Prerequisites:** None (this is the first phase)

---

## 1. Context

This project emulates the behavior of a real industrial robot controller in C++23, targeting GCC 15 / Ubuntu 25.10, with zero heavy runtime dependencies in the core (see `docs/architecture.md` sections 1–3 for the full rationale). This phase builds the foundational in-memory data model: a `Robot` composed of N `Joint`s, each with position/velocity/acceleration state and configurable limits. Nothing here talks to a network, a thread, or a clock — see Non-Goals below.

## 2. Goal

A `robot_core` static library exposing `Robot` and `Joint` types that can be constructed from configuration, advanced deterministically via an explicit `dt`, and that reject invalid commands/configuration through `std::expected` rather than exceptions — fully covered by unit tests, with a working CMake build.

## 3. Non-Goals / Out of Scope

- Real acceleration-profile trajectories (S-curve/trapezoidal) — deferred to Phase 4 (Motion). This phase uses a simple saturated proportional controller inside `Joint::update()`, sufficient to validate the data model.
- Controller state machine (`READY`, `MOVING`, `FAULT`, etc.) — deferred to Phase 2. This phase only has simple boolean flags (`enabled_`, `homed_`, `faulted_`).
- Motor/encoder as separate simulated components — deferred to Phase 5 (Virtual Hardware). The "motor" is implicit inside `Joint::update()` here.
- Any threading, or a dedicated control-loop with jitter measurement — deferred to Phase 3. This phase is called synchronously, from tests only.
- Any I/O (networking, JSON config file loading) — deferred to Phase 6 (protocol) and an intermediate config-loading step. `RobotConfig` is built directly in code/tests here.

## 4. Inputs

- `docs/architecture.md`, sections 3.1 (language/standard), 3.2 (core dependencies), 3.3 (hardware model), 3.15 (functional safety practices — no dynamic allocation, no exceptions on the control path).
- `CONTRIBUTING.md` — coding standards and mandatory file header.

No other file in this repository is required to complete this phase.

## 5. Deliverables

```
include/robot/core/joint_error.hpp
include/robot/core/joint_limits.hpp
include/robot/core/joint_state.hpp
include/robot/core/joint.hpp
include/robot/core/robot_config.hpp
include/robot/core/robot.hpp
include/robot/core/clock.hpp

src/core/joint.cpp
src/core/joint_limits.cpp
src/core/robot.cpp
src/core/robot_config.cpp

tests/core/joint_test.cpp
tests/core/robot_test.cpp

CMakeLists.txt   (repository root)
```

## 6. Interfaces / Contracts

These signatures are load-bearing — Phase 2 (state machine) and Phase 3 (control loop) are designed assuming they hold exactly as written. See section "Detailed Design" below for the full definitions with field-level comments; the essential public surface is:

```cpp
namespace robot::core {

enum class JointError {
    PositionOutOfRange, VelocityOutOfRange, AccelerationOutOfRange,
    JointDisabled, InvalidConfiguration,
};

struct JointLimits {
    double min_position, max_position, max_velocity, max_acceleration;
    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

struct JointState {
    double position = 0.0, velocity = 0.0, acceleration = 0.0, torque = 0.0;
};

class Joint {
public:
    explicit Joint(JointLimits limits) noexcept;
    [[nodiscard]] double position() const noexcept;
    [[nodiscard]] double velocity() const noexcept;
    [[nodiscard]] double acceleration() const noexcept;
    [[nodiscard]] double torque() const noexcept;
    [[nodiscard]] bool enabled() const noexcept;
    [[nodiscard]] bool homed() const noexcept;
    [[nodiscard]] bool faulted() const noexcept;
    void enable() noexcept;
    void disable() noexcept;
    void home() noexcept;
    [[nodiscard]] std::expected<void, JointError> setTargetPosition(double position) noexcept;
    [[nodiscard]] std::expected<void, JointError> setTargetVelocity(double velocity) noexcept;
    void update(std::chrono::nanoseconds dt) noexcept;
};

struct RobotConfig {
    std::string name;
    std::vector<JointLimits> joints;
    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

class Robot {
public:
    [[nodiscard]] static std::expected<Robot, JointError> create(RobotConfig config);
    [[nodiscard]] std::size_t jointCount() const noexcept;
    [[nodiscard]] Joint& joint(std::size_t index) noexcept;
    [[nodiscard]] const Joint& joint(std::size_t index) const noexcept;
    [[nodiscard]] std::span<Joint> joints() noexcept;
    [[nodiscard]] std::span<const Joint> joints() const noexcept;
    void update(std::chrono::nanoseconds dt) noexcept;
};

class Clock {
public:
    virtual ~Clock() = default;
    [[nodiscard]] virtual std::chrono::steady_clock::time_point now() const noexcept = 0;
};
class SteadyClock final : public Clock { /* wraps std::chrono::steady_clock::now() */ };
class ManualClock final : public Clock { /* settable, for deterministic tests */ };

}  // namespace robot::core
```

**Non-negotiable design rules for this phase** (see `docs/architecture.md` 3.15 for why):
- No dynamic heap allocation inside `Joint::update()` or `Robot::update()`.
- No exceptions thrown from any `noexcept`-marked function above — all fallible paths return `std::expected`.
- `Joint`/`Robot` must not read any clock internally; `dt` is always passed in explicitly. `Clock`/`SteadyClock`/`ManualClock` exist for future phases to use — this phase only needs to define them, not wire them into `Joint`/`Robot`.
- `Robot::create()` must never leave a partially-constructed, invalid `Robot` observable from the outside — validate first, construct only on success.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_core` compiles as a static library with no dependencies beyond the STL.
- [ ] `Robot::create()` succeeds for a valid N-joint configuration and fails with `JointError::InvalidConfiguration` for zero joints or inconsistent limits.
- [ ] `Joint::update(dt)` advances position/velocity while respecting position, velocity, and acceleration limits (clamping verified by test).
- [ ] Setting a target position/velocity beyond limits returns the correct `JointError` and does not change joint state.
- [ ] A disabled joint does not move even if a target was set before disabling — reflected in a test.
- [ ] `Joint::home()` resets position and sets `homed() == true`.
- [ ] No heap allocation inside `Joint::update()` — verified with `valgrind --tool=massif` (or documented equivalent) showing zero allocations during a simulated run.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`.
- [ ] `CMakeLists.txt` builds successfully in both `Release` and `Debug`, with `-fsanitize=address,undefined` enabled in Debug, on both GCC 15 and Clang 20.
- [ ] GoogleTest suite covers: position clamping, velocity clamping, acceleration clamping, disabled-joint-does-not-move, invalid-configuration-rejected, `Robot::update` advancing all joints in stable order.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.

## 8. Notes for the Implementer

- You do not need to design anything from scratch — section 9 below ("Detailed Design") contains complete class definitions, the full `CMakeLists.txt`, and example test bodies. Implement `.cpp` files (the `.hpp` declarations and `CMakeLists.txt` are given verbatim) and fill in any logic left as a stub.
- `Joint::update()`'s internal control law does not need to be sophisticated — a simple saturated proportional controller toward `target_position_`/`target_velocity_`, clamped to `limits_`, satisfies this phase's acceptance criteria. Do not build a real trajectory planner here; that is explicitly Phase 4's job.
- The thread model, clock strategy, and directory layout below are **binding design decisions already made** for the whole project, not just this phase — do not deviate from them even though this phase itself is single-threaded and clock-free.
- You do not need `docs/task-briefs/` entries for any other phase to complete this one.

---

## 9. Detailed Design (reference implementation)

### 9.1 Class model

```
                     ┌───────────────────┐
                     │    RobotConfig     │
                     │  (config data)      │
                     └─────────┬─────────┘
                               │ used to build
                               ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│    JointLimits      │◄──│      Joint         │──►│     JointState     │
│ (min/max pos/vel/acc)│   │ (behavior)          │   │  (position/vel/acc)│
└───────────────────┘   └─────────┬─────────┘   └───────────────────┘
                               │ 1..N
                               ▼
                     ┌───────────────────┐
                     │       Robot         │
                     │  owner of Joints    │
                     └───────────────────┘
```

### 9.2 `include/robot/core/joint_error.hpp`

```cpp
// include/robot/core/joint_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::core {

enum class JointError {
    PositionOutOfRange,
    VelocityOutOfRange,
    AccelerationOutOfRange,
    JointDisabled,
    InvalidConfiguration,
};

}
```

### 9.3 `include/robot/core/joint_limits.hpp`

```cpp
// include/robot/core/joint_limits.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include "robot/core/joint_error.hpp"

namespace robot::core {

struct JointLimits {
    double min_position;      // rad
    double max_position;      // rad
    double max_velocity;      // rad/s
    double max_acceleration;  // rad/s^2

    // Validates internal consistency of the limits themselves (min < max, positive values, etc.)
    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

}
```

### 9.4 `include/robot/core/joint_state.hpp`

```cpp
// include/robot/core/joint_state.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::core {

struct JointState {
    double position     = 0.0;  // rad
    double velocity     = 0.0;  // rad/s
    double acceleration = 0.0;  // rad/s^2
    double torque       = 0.0;  // N·m (placeholder until Phase 5 — Virtual Hardware)
};

}
```

### 9.5 `include/robot/core/joint.hpp`

```cpp
// include/robot/core/joint.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>
#include <expected>
#include "robot/core/joint_error.hpp"
#include "robot/core/joint_limits.hpp"
#include "robot/core/joint_state.hpp"

namespace robot::core {

class Joint {
public:
    explicit Joint(JointLimits limits) noexcept;

    // --- State reads (no side effects) ---
    [[nodiscard]] double position() const noexcept     { return state_.position; }
    [[nodiscard]] double velocity() const noexcept     { return state_.velocity; }
    [[nodiscard]] double acceleration() const noexcept { return state_.acceleration; }
    [[nodiscard]] double torque() const noexcept       { return state_.torque; }

    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    [[nodiscard]] bool homed() const noexcept   { return homed_; }
    [[nodiscard]] bool faulted() const noexcept { return faulted_; }

    // --- Commands ---
    void enable() noexcept;
    void disable() noexcept;

    // Home is synchronous and deterministic in Phase 1 (no real homing trajectory yet —
    // that's expanded in Phase 4/5). Here, it simply zeroes position and sets homed_ = true.
    void home() noexcept;

    [[nodiscard]] std::expected<void, JointError> setTargetPosition(double position) noexcept;
    [[nodiscard]] std::expected<void, JointError> setTargetVelocity(double velocity) noexcept;

    // Advances the joint's physical state by dt. No heap allocation. No clock reads.
    void update(std::chrono::nanoseconds dt) noexcept;

private:
    JointLimits limits_;
    JointState  state_;

    double target_position_ = 0.0;
    double target_velocity_ = 0.0;

    bool enabled_ = false;
    bool homed_   = false;
    bool faulted_ = false;

    // Internal clamping used by update() — not exposed publicly.
    void clampToLimits() noexcept;
};

}
```

Note: since error paths use `std::expected` instead of exceptions, virtually the entire `Joint` API can (and should) be `noexcept`.

### 9.6 `include/robot/core/robot_config.hpp`

```cpp
// include/robot/core/robot_config.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <string>
#include <vector>
#include "robot/core/joint_limits.hpp"

namespace robot::core {

struct RobotConfig {
    std::string name;
    std::vector<JointLimits> joints;   // one entry per joint, in order J1..JN

    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

}
```

### 9.7 `include/robot/core/robot.hpp`

```cpp
// include/robot/core/robot.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>
#include <expected>
#include <span>
#include <vector>
#include "robot/core/joint.hpp"
#include "robot/core/robot_config.hpp"

namespace robot::core {

class Robot {
public:
    // Fails if the config is invalid (inconsistent limits, zero joints, etc.)
    [[nodiscard]] static std::expected<Robot, JointError> create(RobotConfig config);

    [[nodiscard]] std::size_t jointCount() const noexcept { return joints_.size(); }

    [[nodiscard]] Joint&       joint(std::size_t index) noexcept       { return joints_[index]; }
    [[nodiscard]] const Joint& joint(std::size_t index) const noexcept { return joints_[index]; }

    [[nodiscard]] std::span<Joint>       joints() noexcept       { return joints_; }
    [[nodiscard]] std::span<const Joint> joints() const noexcept { return joints_; }

    // Advances all joints by dt. Iteration order is stable (J1..JN).
    void update(std::chrono::nanoseconds dt) noexcept;

private:
    explicit Robot(std::string name, std::vector<Joint> joints) noexcept;

    std::string name_;
    std::vector<Joint> joints_;
};

}
```

`Robot::create()` is a named constructor returning `std::expected` — the private constructor is never called with invalid data, so there's no partially inconsistent state observable from the outside.

### 9.8 `include/robot/core/clock.hpp`

```cpp
// include/robot/core/clock.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>

namespace robot::core {

// Minimal time-source abstraction — allows swapping in a manual clock for
// deterministic tests (Phase 3+) without touching Robot/Joint.
class Clock {
public:
    virtual ~Clock() = default;
    [[nodiscard]] virtual std::chrono::steady_clock::time_point now() const noexcept = 0;
};

class SteadyClock final : public Clock {
public:
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override {
        return std::chrono::steady_clock::now();
    }
};

// Used in control-loop tests (Phase 3): advances time manually, with no
// reliance on real sleeps — timing tests become instantaneous and deterministic.
class ManualClock final : public Clock {
public:
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override {
        return now_;
    }
    void advance(std::chrono::nanoseconds delta) noexcept { now_ += delta; }

private:
    std::chrono::steady_clock::time_point now_{};
};

}
```

`Clock` stays isolated in `core/`, but `Joint`/`Robot` do not depend on it — only the future control loop (Phase 3) will instantiate a `SteadyClock` in production and a `ManualClock` in tests, computing `dt` and passing it into `Robot::update(dt)`.

### 9.9 Thread model (binding decision for the whole project, not just this phase)

Phase 1 is single-threaded by design — neither `Robot` nor `Joint` do any internal synchronization (no mutex, no atomic). This is intentional:

1. In this phase, the only "client" of `Robot`/`Joint` is the test process itself, running synchronously.
2. In Phase 3 (deterministic control loop), a single dedicated thread will own the `Robot` instance and call `update()` at a fixed cadence. That thread is the sole writer.
3. In Phase 6 (protocol/networking), the network I/O thread will not touch `Robot` directly. Commands arrive through a lock-free queue and are applied by the control thread at the start of each cycle. State reads for `GET_STATUS` are published by the control thread into a snapshot read by the network thread, avoiding a lock on the hot path.

This decision avoids, from this phase onward, the common trap of "let's just put a `std::mutex` inside `Robot` to keep things simple" — which becomes a contention bottleneck once the control loop is trying to run at 1 kHz and competes for the lock with the network thread on every cycle. Do not add any synchronization primitive to `Robot`/`Joint` in this phase.

### 9.10 CMakeLists.txt

```cmake
# -----------------------------------------------------------------------------
# Robot Emulator Framework
# -----------------------------------------------------------------------------
# Author:   Marco Bueno
# Email:    bueno.marco@gmail.com
# LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
# License:  MIT (see LICENSE file at project root)
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.25)
project(robot-emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOT_EMULATOR_BUILD_TESTS "Build unit tests" ON)
option(ROBOT_EMULATOR_ENABLE_SANITIZERS "Enable ASan/UBSan in Debug builds" ON)
option(ROBOT_EMULATOR_EXPERIMENTAL_CXX26 "Enable experimental C++26 features (GCC 15)" OFF)

if(ROBOT_EMULATOR_EXPERIMENTAL_CXX26)
    set(CMAKE_CXX_STANDARD 26)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND ROBOT_EMULATOR_ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# --- robot_core: domain library, no external dependencies ---
add_library(robot_core STATIC
    src/core/joint.cpp
    src/core/joint_limits.cpp
    src/core/robot.cpp
    src/core/robot_config.cpp
)

target_include_directories(robot_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(robot_core PUBLIC cxx_std_23)

# --- Tests ---
if(ROBOT_EMULATOR_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    )
    # Prevents GoogleTest's own config from overriding the project's flags
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(robot_core_tests
        tests/core/joint_test.cpp
        tests/core/robot_test.cpp
    )

    target_link_libraries(robot_core_tests PRIVATE
        robot_core
        GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(robot_core_tests)
endif()
```

Notes on the CMake choices:
- `CMAKE_CXX_EXTENSIONS OFF` forces pure `-std=c++23` (not `-std=gnu++23`), guaranteeing portability if the project later needs a toolchain other than Ubuntu's GCC 15.
- `-Werror` from Phase 1 onward: cheaper to fix warnings now, with a small codebase, than to accumulate silent technical debt.
- Sanitizers enabled by default in Debug builds — catch undefined behavior and out-of-bounds errors early.
- `FetchContent` for GoogleTest: builds the same way on any machine/CI, without depending on what's installed on the system.

### 9.11 Test examples (GoogleTest)

```cpp
// tests/core/joint_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/core/joint.hpp"

using namespace robot::core;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0,
        .max_position     =  180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

}  // namespace

TEST(Joint, StartsDisabledAndNotHomed) {
    Joint joint(defaultLimits());
    EXPECT_FALSE(joint.enabled());
    EXPECT_FALSE(joint.homed());
}

TEST(Joint, RejectsTargetPositionBeyondLimits) {
    Joint joint(defaultLimits());
    joint.enable();

    auto result = joint.setTargetPosition(500.0);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::PositionOutOfRange);
}

TEST(Joint, DoesNotMoveWhileDisabled) {
    Joint joint(defaultLimits());
    // joint stays disabled on purpose

    auto set = joint.setTargetPosition(90.0);
    ASSERT_FALSE(set.has_value());
    EXPECT_EQ(set.error(), JointError::JointDisabled);

    joint.update(std::chrono::milliseconds(10));
    EXPECT_DOUBLE_EQ(joint.position(), 0.0);
}

TEST(Joint, ConvergesTowardTargetPositionRespectingVelocityLimit) {
    Joint joint(defaultLimits());
    joint.enable();
    ASSERT_TRUE(joint.setTargetPosition(10.0).has_value());

    // 1 simulated second in 1 ms steps — deterministic, no real sleep.
    for (int i = 0; i < 1000; ++i) {
        joint.update(std::chrono::milliseconds(1));
    }

    EXPECT_LE(joint.velocity(), defaultLimits().max_velocity);
    EXPECT_NEAR(joint.position(), 10.0, 0.5);
}

TEST(Joint, HomeResetsPositionAndMarksHomed) {
    Joint joint(defaultLimits());
    joint.enable();
    ASSERT_TRUE(joint.setTargetPosition(45.0).has_value());
    for (int i = 0; i < 500; ++i) joint.update(std::chrono::milliseconds(1));

    joint.home();

    EXPECT_TRUE(joint.homed());
    EXPECT_DOUBLE_EQ(joint.position(), 0.0);
}
```

```cpp
// tests/core/robot_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/core/robot.hpp"

using namespace robot::core;

TEST(Robot, CreateFailsWithZeroJoints) {
    RobotConfig config{.name = "empty", .joints = {}};

    auto result = Robot::create(config);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::InvalidConfiguration);
}

TEST(Robot, CreateSucceedsWithValidSixAxisConfig) {
    RobotConfig config{
        .name = "RE-6AXIS",
        .joints = std::vector<JointLimits>(6, JointLimits{
            .min_position = -180.0, .max_position = 180.0,
            .max_velocity = 2.0,    .max_acceleration = 5.0,
        }),
    };

    auto result = Robot::create(config);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->jointCount(), 6u);
}

TEST(Robot, UpdateAdvancesAllJointsInOrder) {
    RobotConfig config{
        .name = "RE-6AXIS",
        .joints = std::vector<JointLimits>(6, JointLimits{
            .min_position = -180.0, .max_position = 180.0,
            .max_velocity = 2.0,    .max_acceleration = 5.0,
        }),
    };
    auto robot = Robot::create(config).value();

    for (auto& j : robot.joints()) {
        j.enable();
        ASSERT_TRUE(j.setTargetPosition(30.0).has_value());
    }

    for (int i = 0; i < 500; ++i) {
        robot.update(std::chrono::milliseconds(1));
    }

    for (const auto& j : robot.joints()) {
        EXPECT_NEAR(j.position(), 30.0, 1.0);
    }
}
```
