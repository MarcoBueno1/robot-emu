# Phase 10 — Fault Injection

**Status:** Done
**Prerequisites:** None beyond Phase 1 being `Done` (repository/toolchain conventions only). This phase depends on nothing but the standard library — no `robot_core`/`robot_controller`/`robot_hardware`/`robot_sensors`/anything else.

---

## 1. Context

`docs/architecture.md` section 3.9 calls fault injection "the project's real differentiator over a plain simulator" and gives an eleven-entry catalog spanning nearly every module built so far: `ENCODER_FAILURE` (Phase 9's `EncoderSensor` already has a `Stuck`/`Disconnected` mechanism for exactly this), `MOTOR_FAILURE`, `OVER_CURRENT`/`OVER_TEMPERATURE` (Phase 9's `CurrentSensor`/`TemperatureSensor` already *detect* these conditions, but nothing *forces* one), `COMMUNICATION_TIMEOUT` (Phase 6), `POSITION_ERROR`/`VELOCITY_ERROR` (Phase 1), `BRAKE_FAILURE` (Phase 5's `VirtualBrake`), `LIMIT_SWITCH` (Phase 1/8), `EMERGENCY_STOP` (Phase 2/8, already real), `POWER_FAILURE` (not modeled anywhere).

Actually wiring all eleven into their respective real components' behavior would be substantial, module-spanning integration work — the same category of not-yet-done work as `apps/robot-emulator` itself (see Phase 7's Non-Goals). This phase does not do that. Instead, it builds the **one missing, genuinely reusable primitive** every one of those eventual wirings would need: a decoupled fault registry and dispatcher, `FaultInjector`, that records which faults are active and calls a registered handler when one is injected — without knowing or caring what a "motor" or "encoder" *is*. This mirrors `Watchdog`'s decoupling from `ControlLoop` in Phase 8 exactly: a generic mechanism, wired to something concrete by a caller this phase doesn't need to know about.

The CLI example in section 3.9 (`INJECT_FAULT type=ENCODER_FAILURE joint=2`) maps directly onto this phase's `FaultInjector::inject(FaultType::EncoderFailure, jointIndex)` — a future extension of Phase 7's `robotctl` (which doesn't support `InjectFault` yet — documented there as a Non-Goal) would call exactly this.

## 2. Goal

A `robot_fault` static library exposing `FaultType` (the eleven-entry catalog), `ActiveFault`, and `FaultInjector` — a registry supporting one handler per fault type, fault activation/clearing, and querying what's currently active.

## 3. Non-Goals / Out of Scope

- **Simulating the physical effect of any fault type.** `inject(MotorFailure, ...)` does not itself zero out any `VirtualMotor`'s torque, `inject(EncoderFailure, ...)` does not itself call any `EncoderSensor::setFailureMode()`. `FaultInjector` calls whatever handler was registered for that type — if none was registered, nothing beyond the fault being recorded as active happens. No handlers are pre-wired to any real component in this phase.
- **Multiple simultaneous handlers per `FaultType`** (a full pub/sub with many subscribers) — one handler per type, registering again replaces the previous one. Nothing in this project yet needs more than one subscriber to a given fault type.
- **Persisting fault history or timestamps.** `activeFaults()` reflects only current state; this is not an event log.
- **`robotctl`/protocol wiring.** Phase 7's `CommandKind` doesn't include `InjectFault`; adding it is a natural, minor future extension built on top of this phase's `FaultInjector`, not part of this phase.
- **Thread-safety / internal synchronization.** Unlike `Watchdog`/`ControlLoop` (which own a background thread and therefore need atomics), `FaultInjector` spawns no thread and does no I/O — it follows this codebase's majority pattern (`ControllerStateMachine`, `TrapezoidalTrajectory`, etc.): a single owner thread is expected to call its methods.

## 4. Inputs

- `docs/architecture.md` section 3.9 (fault catalog and the `INJECT_FAULT` example this phase's API shape is modeled on).
- `include/robot/safety/watchdog.hpp` (Phase 8) — read as a precedent for how this codebase decouples a generic mechanism from the concrete things it eventually gets wired to; not depended on at the code level.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/fault/fault_type.hpp
include/robot/fault/active_fault.hpp
include/robot/fault/fault_injector.hpp

src/fault/fault_type.cpp
src/fault/fault_injector.cpp

tests/fault/fault_injector_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

```cpp
namespace robot::fault {

/// @brief The eleven-entry fault catalog from docs/architecture.md section 3.9.
enum class FaultType {
    EncoderFailure,
    MotorFailure,
    OverCurrent,
    OverTemperature,
    CommunicationTimeout,
    PositionError,
    VelocityError,
    BrakeFailure,
    LimitSwitch,
    EmergencyStop,
    PowerFailure,
};

[[nodiscard]] std::string_view toString(FaultType type) noexcept;

/// @brief One currently-active fault.
struct ActiveFault {
    FaultType type;
    /// nullopt for a robot-wide fault (e.g. PowerFailure); set for a
    /// fault scoped to one joint (e.g. EncoderFailure on joint 2, matching
    /// section 3.9's `INJECT_FAULT type=ENCODER_FAILURE joint=2` example).
    std::optional<std::size_t> jointIndex;
};

class FaultInjector {
public:
    using Handler = std::function<void(std::optional<std::size_t> jointIndex)>;

    FaultInjector() noexcept = default;

    /// @brief Registers handler to be called by every subsequent inject(type, ...).
    ///        Replaces any previously registered handler for type — see Non-Goals.
    void onFault(FaultType type, Handler handler);

    /// @brief Records (type, jointIndex) as active and, if a handler is
    ///        registered for type, invokes it with jointIndex.
    ///
    /// Idempotent with respect to activeFaults()/isActive(): injecting an
    /// already-active (type, jointIndex) pair again does not duplicate the
    /// entry — but the handler, if any, is still invoked every call, since
    /// a caller may deliberately want to re-trigger its effect.
    void inject(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt);

    /// @brief Clears one active fault. No-op, not an error, if it wasn't active.
    void clear(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt);

    /// @brief Clears every active fault. Does not affect registered handlers.
    void clearAll() noexcept;

    [[nodiscard]] bool isActive(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt) const;

    /// @return Every currently active fault, in injection order.
    [[nodiscard]] std::vector<ActiveFault> activeFaults() const;

private:
    std::vector<ActiveFault> active_;
    std::unordered_map<FaultType, Handler> handlers_;
};

}  // namespace robot::fault
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_fault` compiles as a static library with no dependency on any other `robot_*` target.
- [ ] `inject()` is verified to call a registered handler with the exact `jointIndex` passed (including `std::nullopt` for a robot-wide fault), and to do nothing but record the fault (no crash, no exception) when no handler is registered for that type.
- [ ] `onFault()` called twice for the same `FaultType` is verified to replace the handler — only the second one fires on a subsequent `inject()`.
- [ ] `isActive()`/`activeFaults()` correctly distinguish faults by the `(type, jointIndex)` pair — e.g. `EncoderFailure` on joint 0 and `EncoderFailure` on joint 1 are independently active/clearable, and a robot-wide fault (`jointIndex == nullopt`) is distinct from any joint-scoped one of the same type.
- [ ] Injecting the same `(type, jointIndex)` twice is verified not to duplicate the entry in `activeFaults()`, while still invoking the handler both times.
- [ ] `clear()` on a fault that isn't active is verified to be a harmless no-op (no crash, `activeFaults()` unaffected).
- [ ] `clearAll()` is verified to empty `activeFaults()` while leaving previously registered handlers intact (a subsequent `inject()` still calls them).
- [ ] `toString()` is covered for all eleven `FaultType` values (no default/fallthrough case silently returning an empty or wrong string, matching the `switch`-with-no-`default` pattern established in Phase 2/6).
- [ ] No dynamic heap allocation inside `isActive()` beyond what `std::vector`/`std::unordered_map`'s own amortized growth already implies for `inject()`/`onFault()` — `isActive()`/`toString()` themselves must not allocate.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_fault` and its tests successfully alongside every existing target — Phases 1–9's tests must keep passing unmodified.
- [ ] GoogleTest suite (`robot_fault_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- `std::unordered_map<FaultType, Handler>` needs `FaultType` to be hashable — either provide a `std::hash<robot::fault::FaultType>` specialization, or (simpler, and consistent with this codebase generally preferring the least-machinery option that works) key the map by the enum's underlying integer value instead of the enum itself, if that proves less friction. Either is acceptable; pick whichever keeps `fault_injector.cpp` easiest to read.
- `activeFaults()`'s "in injection order" guarantee is naturally satisfied by storing `active_` as a `std::vector` appended to on each new `(type, jointIndex)` pair (checking for an existing entry first, to satisfy the idempotency acceptance criterion) rather than reaching for a `std::set`/`std::map`, which would reorder by key instead.
- This phase deliberately does not import any enum from `robot::hardware`/`robot::controller` (e.g. reusing `BrakeState` or `ControllerState` for anything) — `FaultType` is its own, independent catalog matching section 3.9's naming exactly, since a "brake failure" fault and a `VirtualBrake`'s `BrakeState` are different concepts (one is "this component is malfunctioning," the other is "this component's current engaged/released position").
