# Phase 02 — Controller State Machine

**Status:** Done
**Prerequisites:** Phase 1 (Core) — `Done`. This phase does not use `robot::core` types directly, but follows the same conventions (`std::expected`, `noexcept`, no dynamic allocation) established there.

---

## 1. Context

This phase builds the controller's formal state machine: `POWER_OFF → BOOTING → INITIALIZING → IDLE → SERVO_ON → READY`, branching into `MOVING`/`PAUSED`/`EMERGENCY_STOP`, with a separate `FAULT → RECOVERY` path — all converging back to `READY`. See `docs/architecture.md` section 3.4 for the source diagram and formal state list.

The state machine is implemented as a **standalone, explicit transition table** (state, event) → state, not a tangle of `if/else`, and not yet wired to anything else. It owns no `Robot`, no clock, no thread, no I/O. It exists so that later phases (3: control loop, 6: protocol, 8: safety) have a single, independently-tested source of truth for "what state is the controller in, and what transitions are legal from here" — each of those phases will drive this state machine with events derived from their own concerns (cycle completion, network commands, watchdog signals), but none of that wiring happens in this phase.

## 2. Goal

A `robot_controller` static library exposing `ControllerStateMachine`, built from a `ControllerState`/`ControllerEvent` pair via an explicit, table-driven `handleEvent()`, rejecting illegal transitions through `std::expected` rather than exceptions — fully covered by unit tests, with a working CMake build alongside `robot_core`.

## 3. Non-Goals / Out of Scope

- Any wiring to `robot::core::Robot`/`Joint`, a clock, or a control loop — deferred to Phase 3. `ControllerStateMachine` is driven purely by explicit `handleEvent()` calls from tests in this phase.
- A dedicated watchdog thread independent from the main control loop — deferred to Phase 3/8 (`docs/architecture.md` section 3.15 notes this explicitly).
- Guard conditions / conditional transitions (e.g. "only allow `CommandMove` if all joints are homed") — this phase's transitions are unconditional given a (state, event) pair. Guards can be layered on top of `handleEvent()` in a later phase without changing this phase's public interface.
- A concrete `FaultReason`/error-code payload on `FaultDetected` — deferred to Phase 10 (Fault Injection). This phase's `FaultDetected` event carries no data.
- Anything that triggers these events from outside (network commands driving `CommandMove`, a watchdog driving `FaultDetected`, etc.) — deferred to Phases 6 (protocol) and 8 (safety). This phase only defines the events and the table.
- Persisting or logging transition history — not needed yet; `state()` always reflects only the current state.

## 4. Inputs

- `docs/architecture.md`, sections 3.4 (controller state machine — source diagram and formal state list) and 3.15 (functional safety practices — no dynamic allocation, no exceptions on the control path; this applies here exactly as it did in Phase 1).
- `CONTRIBUTING.md` — coding standards and the mandatory file header.
- `include/robot/core/joint_error.hpp` (Phase 1) as a style reference only, for how this codebase shapes an `enum class` + `std::expected<void, ErrorEnum>` pair. `robot_controller` does not depend on `robot_core` — no `#include` of any `robot/core/*` header is needed or expected.

No other file in this repository is required to complete this phase.

## 5. Deliverables

```
include/robot/controller/controller_state.hpp
include/robot/controller/controller_event.hpp
include/robot/controller/controller_error.hpp
include/robot/controller/controller_state_machine.hpp

src/controller/controller_state_machine.cpp

tests/controller/controller_state_machine_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

These signatures are load-bearing — Phase 3 (control loop) and Phase 8 (safety) are designed assuming they hold exactly as written.

```cpp
namespace robot::controller {

enum class ControllerState {
    PowerOff, Booting, Initializing, Idle, ServoOff, ServoOn, Ready,
    Moving, Paused, Stopping, EmergencyStop, Fault, Recovery, Shutdown,
};

enum class ControllerEvent {
    PowerOn, BootComplete, InitComplete,
    ServoEnable, ServoDisable, ControllerReady,
    CommandMove, MotionComplete,
    CommandPause, CommandResume,
    CommandStop, StopComplete,
    EStopTriggered, EStopReset,
    FaultDetected, RecoveryStart, RecoveryComplete,
    PowerOff,
};

enum class ControllerError {
    InvalidTransition,  // No table entry for (current state, event).
};

[[nodiscard]] std::string_view toString(ControllerState state) noexcept;
[[nodiscard]] std::string_view toString(ControllerEvent event) noexcept;

class ControllerStateMachine {
public:
    // Starts in ControllerState::PowerOff.
    ControllerStateMachine() noexcept;

    [[nodiscard]] ControllerState state() const noexcept;

    // True if (state(), event) has a table entry, without applying it.
    [[nodiscard]] bool canHandle(ControllerEvent event) const noexcept;

    // Applies event if legal from the current state; on success, state()
    // reflects the new state afterward. On failure, state() is unchanged.
    [[nodiscard]] std::expected<void, ControllerError> handleEvent(ControllerEvent event) noexcept;

private:
    ControllerState state_ = ControllerState::PowerOff;
};

}  // namespace robot::controller
```

**Non-negotiable design rules for this phase** (same spirit as Phase 1, see `docs/architecture.md` 3.15):
- No dynamic heap allocation inside `handleEvent()` or `canHandle()` — the transition table is `static constexpr`.
- No exceptions thrown from any `noexcept`-marked function above — the only fallible path (`handleEvent`) returns `std::expected`.
- `ControllerStateMachine` reads no clock and does no I/O — it is a pure, synchronous state container, exactly like `Joint`/`Robot` in Phase 1.
- `toString()` returns `std::string_view` into string literals (static storage) — never an owning `std::string`, to keep these `noexcept` and allocation-free.

### 6.1 Full transition table (authoritative — implement exactly this)

`docs/architecture.md` section 3.4 gives the state machine at a diagram level; some formal states it lists (`SERVO_OFF`, `STOPPING`) aren't fully spelled out edge-by-edge in the diagram itself. The table below is the **resolved, unambiguous version** — implement exactly this, do not re-derive it from the diagram.

| From | Event | To |
|---|---|---|
| `PowerOff` | `PowerOn` | `Booting` |
| `Booting` | `BootComplete` | `Initializing` |
| `Initializing` | `InitComplete` | `Idle` |
| `Idle` | `ServoEnable` | `ServoOn` |
| `SERVO_OFF (ServoOff)` | `ServoEnable` | `ServoOn` |
| `ServoOn` | `ServoDisable` | `ServoOff` |
| `ServoOn` | `ControllerReady` | `Ready` |
| `Ready` | `ServoDisable` | `ServoOff` |
| `Ready` | `CommandMove` | `Moving` |
| `Moving` | `MotionComplete` | `Ready` |
| `Moving` | `CommandPause` | `Paused` |
| `Paused` | `CommandResume` | `Moving` |
| `Moving` | `CommandStop` | `Stopping` |
| `Paused` | `CommandStop` | `Stopping` |
| `Ready` | `CommandStop` | `Stopping` |
| `Stopping` | `StopComplete` | `Ready` |
| `ServoOn` | `EStopTriggered` | `EmergencyStop` |
| `Ready` | `EStopTriggered` | `EmergencyStop` |
| `Moving` | `EStopTriggered` | `EmergencyStop` |
| `Paused` | `EStopTriggered` | `EmergencyStop` |
| `Stopping` | `EStopTriggered` | `EmergencyStop` |
| `EmergencyStop` | `EStopReset` | `Ready` |
| `Idle` | `FaultDetected` | `Fault` |
| `ServoOff` | `FaultDetected` | `Fault` |
| `ServoOn` | `FaultDetected` | `Fault` |
| `Ready` | `FaultDetected` | `Fault` |
| `Moving` | `FaultDetected` | `Fault` |
| `Paused` | `FaultDetected` | `Fault` |
| `Stopping` | `FaultDetected` | `Fault` |
| `EmergencyStop` | `FaultDetected` | `Fault` |
| `Fault` | `RecoveryStart` | `Recovery` |
| `Recovery` | `RecoveryComplete` | `Ready` |
| `Idle` | `ControllerEvent::PowerOff` | `Shutdown` |
| `ServoOff` | `ControllerEvent::PowerOff` | `Shutdown` |
| `Ready` | `ControllerEvent::PowerOff` | `Shutdown` |
| `Fault` | `ControllerEvent::PowerOff` | `Shutdown` |

Any `(state, event)` pair not listed above must return `ControllerError::InvalidTransition` and leave `state()` unchanged. This includes, notably: `EmergencyStop` accepts *only* `EStopReset` and `FaultDetected` — no `CommandMove`/`CommandStop`/etc. while stopped; `Shutdown` is terminal (accepts no events in this phase — restarting means constructing a new `ControllerStateMachine`).

Design rationale worth knowing (do not re-litigate this in the implementation): the diagram in `docs/architecture.md` draws `MOVING`, `PAUSED`, and `EMERGENCY_STOP` as three siblings of `READY` that all converge back to `READY`, and separately draws `FAULT → RECOVERY → READY` off of `READY`. `STOPPING` and `SERVO_OFF` are in the formal state list but not spelled out in the diagram; this table places `STOPPING` as the intermediate state entered from `MOVING`/`PAUSED`/`READY` on `CommandStop` (so a stop is itself a distinct, observable state rather than an instant transition), and treats `SERVO_OFF` as the state reached by explicitly disabling servos from `ServoOn`/`Ready` — distinct from `Idle`, which is only the pre-servo state reached once, right after initialization.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_controller` compiles as a static library with no dependencies beyond the STL, and does not depend on `robot_core`.
- [ ] Every row of the transition table in section 6.1 is covered by at least one test asserting the resulting `state()`.
- [ ] At least one test per state confirms that an event *not* in that state's table row set returns `ControllerError::InvalidTransition` and leaves `state()` unchanged (e.g. `CommandMove` while `PowerOff`).
- [ ] `EmergencyStop` is reachable from every state the table lists it from, and only leaves via `EStopReset` (to `Ready`) or `FaultDetected` (to `Fault`) — verified by test.
- [ ] `Shutdown` is confirmed terminal: `canHandle()` returns `false` for every `ControllerEvent` once in `Shutdown`.
- [ ] `toString()` is covered for every `ControllerState` and every `ControllerEvent` value (no default/fallthrough case silently returning an empty or wrong string).
- [ ] No heap allocation inside `handleEvent()`/`canHandle()` — the transition table is `static constexpr`, verified by code review (a `massif` run is optional here since the table's small, fixed size makes heap use structurally unlikely, unlike Phase 1's `Joint::update()`).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror` (already project-wide, via the root `CMakeLists.txt`).
- [ ] `CMakeLists.txt` builds `robot_controller` and its tests successfully in both `Release` and `Debug`, with sanitizers enabled in Debug, alongside the existing `robot_core` target — Phase 1's tests must keep passing unmodified.
- [ ] GoogleTest suite (`robot_controller_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments (`///` + `@brief`/`@param`/`@return`) on every public type and method, consistent with the style already applied to `robot::core` headers.

## 8. Notes for the Implementer

- You do not need to design the transition table — section 6.1 is authoritative and final for this phase; if it looks incomplete or wrong, that's a defect in *this brief*, not something to silently patch by inventing your own rows.
- Implement `handleEvent()`/`canHandle()` as a linear scan over a `static constexpr std::array` of `{ControllerState, ControllerEvent, ControllerState}` triples. At this table's size (~30 rows), a linear scan is simpler, just as fast in practice, and easier to audit row-by-row than a `std::map`/hash-based lookup — don't over-engineer this.
- `toString()` should be a `switch` with no `default:` case (let `-Wswitch` catch a state/event added later without an updated string), returning a string literal per enumerator; add a final `return "Unknown";` *after* the switch only if the compiler warns about a non-void "not all control paths return a value" — check first, since exhaustive `switch` over an `enum class` with an unreachable trailing return may itself warn under `-Wpedantic`. If so, `assert(false)`/`__builtin_unreachable()` before the trailing return is acceptable.
- This phase deliberately does not touch `robot_core` or anything in `include/robot/core/`. Do not add an `#include "robot/core/..."` anywhere in `robot_controller` — the two libraries are independent until Phase 3 wires them together.
- You do not need `docs/task-briefs/` entries for any other phase to complete this one.
