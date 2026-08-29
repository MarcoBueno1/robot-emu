# Phase 08 — Safety: Limits, E-Stop, Watchdog

**Status:** Done
**Prerequisites:** Phase 1 (Core), Phase 2 (Controller State Machine), Phase 5 (Virtual Hardware) — all `Done`. This phase combines `robot_core`, `robot_controller`, and `robot_hardware` for the first time; it does not depend on `robot_runtime`, `robot_motion`, `robot_protocol`, or `robot_cli`.

---

## 1. Context

The safety behaviors this phase implements were **anticipated but deferred** by earlier phases, on purpose:

- Phase 2's `ControllerStateMachine` already has an `EmergencyStop` state and `EStopTriggered`/`EStopReset` events — but nothing in the codebase calls `handleEvent(EStopTriggered)` from anywhere real. This phase is that "somewhere real."
- Phase 5's `VirtualBrake` already models a fail-safe brake with an unconditional `engage()` — documented then as existing specifically so "Phase 8's safety system must be able to engage every brake robot-wide regardless of any other component's state." This phase is that safety system.
- `docs/architecture.md` section 3.15 names the third piece directly: "Safety watchdog independent from the main control loop... running on a separate thread, so a control-loop lockup doesn't prevent fault detection."

**A genuine interface gap, found while designing this phase:** neither `robot::core::Joint` nor `Robot` expose their configured `JointLimits` — Phase 1 never needed to read a limit back, only enforce it internally. A limit *monitor* (this phase) genuinely needs to read it. The fix is a single, minimal, purely additive change to already-shipped Phase 1 code: a `Joint::limits() const noexcept` getter, mirroring `position()`/`velocity()`'s existing shape exactly. This is not a behavior change — Phase 1's existing tests are required to keep passing completely unmodified (see Acceptance Criteria) — it's completing an interface that has a real caller now. This is the *only* Phase 1 file this phase touches.

## 2. Goal

A `robot_safety` static library exposing three independent, composable pieces:

- `LimitMonitor` — proactively flags joints within a configurable margin of their position limits (a *soft*, observational check, independent of `Joint`'s own hard clamp — the same defense-in-depth idea as a real robot's software position limits being separate from its physical hard stops).
- `EmergencyStopController` — the first real caller of `ControllerStateMachine::handleEvent(EStopTriggered/EStopReset)`, and the first real caller of `VirtualBrake::engage()` at the "robot-wide" level Phase 5 anticipated.
- `Watchdog` — an independently-threaded liveness monitor: fed a heartbeat counter, it trips if that counter stops advancing within a configured timeout.

## 3. Non-Goals / Out of Scope

- **Automatically wiring `LimitMonitor`/`Watchdog` into `ControllerStateMachine`.** Both are purely observational — `LimitMonitor::check()` returns violations, `Watchdog::tripped()` returns a bool. Neither calls `handleEvent()` itself. This is a deliberate asymmetry with `EmergencyStopController`, which *does* act directly: an E-stop's whole purpose is an immediate, unconditional response, whereas different callers might reasonably want different responses to a limit warning or a watchdog trip (log and continue, escalate to E-stop, alert an operator) — this phase doesn't presume which.
- **A unified "physical robot" aggregate** bundling a `Robot` with its `VirtualBrake`s (and eventually motors/encoders) into one object. `EmergencyStopController` takes a `Robot&` and a separate `std::span<robot::hardware::VirtualBrake>` because nothing in this codebase bundles them yet (Phase 5's Non-Goals deliberately left `Joint`/`VirtualMotor`/`VirtualEncoder`/`VirtualBrake` unwired to each other) — building that aggregate is future integration work, the same category of work `apps/robot-emulator` still needs (see Phase 7's Non-Goals).
- **Real sensor-based validation** (temperature/current thresholds) — Phase 9.
- **Fault injection** (`INJECT_FAULT`, the fault catalog in section 3.9) — not yet reached in this roadmap's numbered phases.
- **Config-file-driven margins/timeouts.** `LimitMonitor`'s margin and `Watchdog`'s timeout/poll interval are constructor parameters, not loaded from anywhere — section 3.10's declarative configuration is separate, unstarted work.
- **Auto-releasing brakes on `EmergencyStopController::reset()`.** Reset only attempts the state-machine transition back to `Ready`; releasing a brake stays a deliberate, separate action a caller takes explicitly — see section 8.

## 4. Inputs

- `docs/architecture.md` sections 3.4 (the `EmergencyStop` state this phase finally drives), 3.9 (fault catalog — context only, not implemented here), and 3.15 (the watchdog requirement, quoted above).
- `include/robot/core/joint.hpp`/`robot.hpp` (Phase 1) — read, and `joint.hpp` gets the one-line `limits()` addition described above.
- `include/robot/controller/controller_state_machine.hpp` (Phase 2) — `EmergencyStopController` calls `handleEvent()` on one directly.
- `include/robot/hardware/virtual_brake.hpp` (Phase 5) — `EmergencyStopController` calls `engage()` on a span of these directly.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/core/joint.hpp        (MODIFIED — adds one accessor, see section 6.0)

include/robot/safety/limit_violation.hpp
include/robot/safety/limit_monitor.hpp
include/robot/safety/emergency_stop_controller.hpp
include/robot/safety/watchdog_error.hpp
include/robot/safety/watchdog.hpp

src/safety/limit_monitor.cpp
src/safety/emergency_stop_controller.cpp
src/safety/watchdog.cpp

tests/safety/limit_monitor_test.cpp
tests/safety/emergency_stop_controller_test.cpp
tests/safety/watchdog_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

### 6.0 `Joint::limits()` — the one Phase 1 change

```cpp
// Added alongside position()/velocity()/acceleration()/torque() — same shape exactly.
[[nodiscard]] const JointLimits& limits() const noexcept { return limits_; }
```

### 6.1 `LimitMonitor`

```cpp
namespace robot::safety {

struct LimitViolation {
    std::size_t jointIndex;
    double position;
    double distanceToNearestLimit;  // always >= 0; how far inside the margin it is.
};

class LimitMonitor {
public:
    /// @param marginRadians How close to min_position/max_position counts
    ///        as a violation. Must be >= 0 (0 means "flag only if already
    ///        exactly at a limit" — structurally rare given Joint's own
    ///        hard clamp, but not impossible at the exact boundary).
    explicit LimitMonitor(double marginRadians) noexcept;

    /// @brief Scans every joint in robot; returns one LimitViolation per
    ///        joint currently within marginRadians of either bound.
    ///        Empty if none are. Order matches robot.joints() (J1..JN).
    [[nodiscard]] std::vector<LimitViolation> check(const robot::core::Robot& robot) const;

private:
    double marginRadians_;
};

}
```

### 6.2 `EmergencyStopController`

```cpp
namespace robot::safety {

class EmergencyStopController {
public:
    /// controller and brakes are borrowed (not owned) — both must outlive
    /// this EmergencyStopController. brakes need not correspond 1:1 to any
    /// particular Robot's joints (see Non-Goals) — this phase does not
    /// assume or enforce that correspondence.
    EmergencyStopController(robot::controller::ControllerStateMachine& controller,
                             std::span<robot::hardware::VirtualBrake> brakes) noexcept;

    /// @brief Engages every brake in brakes, unconditionally, THEN attempts
    ///        controller.handleEvent(EStopTriggered).
    ///
    /// Brakes are engaged regardless of whether the state transition
    /// itself succeeds (e.g. if the controller was already PowerOff) —
    /// physical safety must never be gated on a state machine's
    /// bookkeeping succeeding first.
    void trigger() noexcept;

    /// @brief Attempts controller.handleEvent(EStopReset). Does not touch
    ///        any brake — see this phase's Non-Goals on why release is
    ///        never automatic.
    [[nodiscard]] std::expected<void, robot::controller::ControllerError> reset() noexcept;

private:
    robot::controller::ControllerStateMachine& controller_;
    std::span<robot::hardware::VirtualBrake> brakes_;
};

}
```

### 6.3 `Watchdog`

```cpp
namespace robot::safety {

enum class WatchdogError { AlreadyRunning, NotRunning };

class Watchdog {
public:
    using HeartbeatCounterFn = std::function<std::uint64_t()>;

    /// @param heartbeatCounter Returns a counter expected to keep
    ///        increasing (e.g. a ControlLoop's cyclesExecuted metric —
    ///        Watchdog itself has no dependency on robot_runtime; any
    ///        counter-returning callable works). Must be safe to call
    ///        from the watchdog's own background thread.
    /// @param timeout If heartbeatCounter() hasn't advanced for this long, tripped() becomes true.
    /// @param pollInterval How often the background thread checks.
    Watchdog(HeartbeatCounterFn heartbeatCounter, std::chrono::nanoseconds timeout,
             std::chrono::nanoseconds pollInterval) noexcept;

    ~Watchdog();  // Stops the background thread if still running.
    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;

    [[nodiscard]] std::expected<void, WatchdogError> start();
    [[nodiscard]] std::expected<void, WatchdogError> stop();
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief True once the heartbeat counter has been observed to not
    ///        advance for at least timeout. Sticky — stays true until reset().
    [[nodiscard]] bool tripped() const noexcept;

    /// @brief Clears tripped() and re-baselines the internal "last seen
    ///        counter/time" so the watchdog doesn't immediately re-trip on
    ///        its next poll.
    void reset() noexcept;

private:
    void runLoop(std::stop_token stopToken);
    // ...
};

}
```

`start()`/`stop()`/`isRunning()`'s shape deliberately mirrors `robot::runtime::ControlLoop` (Phase 3) exactly — same vocabulary, same lifecycle pattern, for a reader already familiar with that class.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_safety` compiles as a static library depending on `robot_core`, `robot_controller`, `robot_hardware`, and `Threads::Threads` — no dependency on `robot_runtime`/`robot_motion`/`robot_protocol`/`robot_cli`.
- [ ] `Joint::limits()` is the *only* change to any Phase 1–7 file — every existing test suite (`robot_core_tests` through `robot_cli_tests`) passes completely unmodified.
- [ ] `LimitMonitor::check()` returns no violations for joints well within their limits, and exactly the expected violations (right `jointIndex`, right sign/magnitude of `distanceToNearestLimit`) for joints placed near either bound — verified by test using a `Robot` built the same way Phase 1's own tests build one.
- [ ] `EmergencyStopController::trigger()` is verified to engage every brake in a multi-brake span and to move the controller to `EmergencyStop` when triggered from a state that allows it (e.g. `Ready`); and to still engage every brake even when triggered from a state where the transition itself is illegal (e.g. `PowerOff`) — the brake side-effect must be unconditional, verified by test.
- [ ] `EmergencyStopController::reset()` is verified to transition the controller back to `Ready` when called from `EmergencyStop`, and to leave every brake's state completely unchanged (still engaged) — verified by test.
- [ ] `Watchdog::tripped()` is verified `false` while a heartbeat counter is actively incrementing faster than `timeout`, and `true` once the counter stops advancing for longer than `timeout` — a real-timing test (short, tens-of-milliseconds `timeout`/`pollInterval`, same tolerant-assertion approach Phase 3 used for its own threaded tests, not exact-count assertions).
- [ ] `Watchdog::reset()` is verified to clear `tripped()` and not immediately re-trip on the very next poll.
- [ ] `start()` returns `WatchdogError::AlreadyRunning` if already running; `stop()` returns `WatchdogError::NotRunning` if not running — both verified by test.
- [ ] The `Watchdog` destructor stops a still-running background thread without an explicit `stop()` call — verified under Debug's sanitizers, same category of check Phase 3 used for `ControlLoop`'s destructor.
- [ ] No dynamic heap allocation inside `LimitMonitor::check()`'s per-joint comparison logic itself (the returned `std::vector<LimitViolation>` allocating for its results is expected and acceptable, not a per-cycle hot-path allocation in the sense section 3.15 cares about) or inside `Watchdog`'s per-poll check.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers, including the real-threaded `Watchdog` tests) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_safety` and its tests successfully alongside every existing target.
- [ ] GoogleTest suite (`robot_safety_tests`) covers all criteria above.
- [ ] Every new/modified deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- `EmergencyStopController::trigger()`'s ordering (brakes first, state transition second, and proceeding with the brakes regardless of the transition's outcome) is the one piece of business logic in this phase worth getting exactly right — write the brake-engagement loop first, unconditionally, before even looking at `handleEvent()`'s result.
- For `Watchdog`'s background thread, reuse the same `std::jthread` + `stop_token` idiom `robot::runtime::ControlLoop` established in Phase 3 — including its documented bounded-stop-latency caveat (a poll-interval-sized delay between `request_stop()` and the thread actually noticing, since a `sleep_until`/`sleep_for`-based wait isn't itself interruptible by a stop token). Keep `pollInterval` short in tests for the same reason Phase 3 kept its test periods short.
- `Watchdog`'s "last seen counter value" and "last time it changed" need to live behind atomics (or a mutex) readable from both the background thread and `tripped()`/`reset()` being called from another thread — same category of cross-thread state Phase 3's `ControlLoopMetrics` already handled with independent atomics; a single atomic transaction isn't required here either.
- Don't make `LimitViolation::distanceToNearestLimit` signed/directional (e.g. negative for "past the limit") — Phase 1's `Joint::update()` already makes it structurally very hard to end up outside `[min_position, max_position]` in the first place (see Context), so this phase's monitor is about proximity, not violation-with-direction; keep it a plain non-negative magnitude and let `position` (also returned) tell the caller which side it's near, if that ever matters.
