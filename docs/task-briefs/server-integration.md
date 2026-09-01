# Server Integration — `apps/robot-emulator`

**Status:** Done
**Prerequisites:** Phase 1 (Core), Phase 2 (Controller State Machine), Phase 3 (Control Loop), Phase 5 (Virtual Hardware), Phase 6 (Protocol), Phase 7 (CLI), Phase 8 (Safety) — all `Done`. This is the integration phase every one of those briefs' Non-Goals pointed to as "future work" — the first time `robot_core`, `robot_controller`, `robot_runtime`, `robot_hardware`, `robot_protocol`, `robot_cli`, and `robot_safety` are all wired together in one running process.

**Numbering note:** this is not one of the roadmap's numbered phases (`docs/architecture.md` section 5 lists 14; the next unstarted one is Phase 12, "Optional visualization"). It fills the gap every README revision since Phase 7 has flagged explicitly under "Known gap": `robotctl` had nothing real to connect to. This brief follows the same format as the numbered ones for consistency, without claiming a roadmap number that isn't its own.

---

## 1. Context

Phase 7's `robotctl` defined a response wire convention and a `Client` that speaks it, but explicitly built nothing to answer on the other end (`docs/task-briefs/phase-07-cli.md` Non-Goals: *"The real `apps/robot-emulator` server... That's substantial, separate integration work... not something this CLI-focused phase does incidentally."*). Phase 5's `VirtualBrake`, Phase 8's `EmergencyStopController`, and Phase 8's `Watchdog` were all built and tested standalone, each explicitly deferring "wiring into a real running system" to later. This phase is that later.

**The one hard problem this phase actually has to solve, that no prior phase faced:** `robot::core::Robot`, `robot::controller::ControllerStateMachine`, and everything built on them are explicitly *not* thread-safe by design — every one of their briefs says so directly ("Thread-safety: none... a single owner thread is expected"). But a real server has two things that must legitimately happen concurrently: `robot::runtime::ControlLoop` advancing the robot's physical state every millisecond, and incoming TCP commands (`MoveJoint`, `Enable`, `EmergencyStop`, ...) mutating that same `Robot`/`ControllerStateMachine` from a different thread. This phase resolves that with one `std::mutex`, owned by the server (not by any Phase 1–8 type, which stay completely unmodified), held around every access to the shared `Robot&`/`ControllerStateMachine&` — see section 6.2.

## 2. Goal

A `robot_server` static library exposing `CommandDispatcher` (pure command-handling logic: given a request `Frame`, produce a response `Frame`, no networking, no threading) and a real `apps/robot-emulator` executable that wires a `Robot`, `ControllerStateMachine`, `VirtualBrake`s, `EmergencyStopController`, `Watchdog`, `ControlLoop`, and `TcpListener` together into a running server `robotctl` can actually talk to.

## 3. Non-Goals / Out of Scope

- **Multi-client concurrent serving.** One connection at a time, served sequentially — matching Phase 6's own explicit deferral ("a real server accepting many concurrent clients... is future work, not this phase's"). This is still genuinely useful: every `robotctl` invocation (Phase 7) is a single connect-execute-disconnect cycle by design.
- **`InjectFault`/`GetPosition`/`GetIO`/`SetIO`/`ResetFault`/`MoveLinear`/`Connect`-as-a-wire-command.** `CommandDispatcher` handles exactly the seven `CommandType`s Phase 7's `robotctl` actually sends (`GetStatus`, `Enable`, `Disable`, `Home`, `MoveJoint`, `Stop`, `EmergencyStop`); anything else gets a generic error response, not a crash — extending to more commands is a natural, separate future addition on both ends.
- **`robot::fault::FaultInjector` wiring** — no `InjectFault` command exists to trigger it from the wire (see above); this server does not construct or use a `FaultInjector`.
- **A declarative config file** (section 3.10) for joint count/limits/port — this server takes a `--port` argument (default `9000`, matching the doc's own `robotctl connect 127.0.0.1:9000` example) and hardcodes a 6-joint scenario matching Phase 11's benchmark scenario exactly.
- **Persisting anything across restarts** — every server run starts fresh: boot chain complete, `Idle`, brakes engaged, no state carried from a previous run.

## 4. Inputs

- `docs/task-briefs/phase-07-cli.md` sections 6.2–6.4 — the exact response/payload wire conventions this server must honor for `robotctl` to work against it unmodified (this is the contract that brief promised a future phase would keep).
- `include/robot/runtime/control_loop.hpp` (Phase 3), `include/robot/safety/{emergency_stop_controller,watchdog}.hpp` (Phase 8), `include/robot/protocol/{tcp_listener,tcp_connection,frame_codec}.hpp` (Phase 6), `include/robot/cli/{status_payload,move_joint_payload,response}.hpp` (Phase 7) — read, and reused as-is; none are modified by this phase.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/server/command_dispatcher.hpp

src/server/command_dispatcher.cpp

tests/server/command_dispatcher_test.cpp

apps/robot-emulator/main.cpp

CMakeLists.txt   (repository root — extended, not replaced)
README.md        (MODIFIED — clears the "known gap" note, updates the roadmap area)
```

## 6. Interfaces / Contracts

### 6.1 `CommandDispatcher` — pure logic, no networking, no threading

```cpp
namespace robot::server {

/// @brief Given a request Frame, performs the corresponding action against
///        a Robot/ControllerStateMachine/EmergencyStopController and
///        returns an encoded response Frame, following exactly the
///        convention docs/task-briefs/phase-07-cli.md sections 6.2–6.4 define.
///
/// Does no locking itself and spawns no thread — see section 6.2. A caller
/// (apps/robot-emulator/main.cpp) is responsible for holding the shared
/// mutex for the duration of each dispatch() call.
class CommandDispatcher {
public:
    CommandDispatcher(robot::core::Robot& robot, robot::controller::ControllerStateMachine& controller,
                       robot::safety::EmergencyStopController& estop) noexcept;

    /// @brief Handles one request. Unrecognized/unsupported CommandTypes
    ///        get a well-formed error response (matching the request's
    ///        type, ResponseStatus::Error), never a crash or an unanswered request.
    [[nodiscard]] robot::protocol::Frame dispatch(const robot::protocol::Frame& request);

private:
    // handleGetStatus/handleEnable/handleDisable/handleHome/handleMoveJoint/
    // handleStop/handleEmergencyStop, one per supported CommandType — see
    // section 6.3 for what each one actually does.

    robot::core::Robot& robot_;
    robot::controller::ControllerStateMachine& controller_;
    robot::safety::EmergencyStopController& estop_;
};

}
```

### 6.2 Concurrency design (authoritative — the central decision of this phase)

`apps/robot-emulator/main.cpp` owns exactly one `std::mutex` guarding the shared `Robot`/`ControllerStateMachine`. Two things touch that state, and both hold this mutex for their entire critical section:

1. **The control thread**, a `std::jthread` the server spawns itself — *not* `ControlLoop::start()`/`stop()`. `ControlLoop::step()` was designed in Phase 3 to be "safe and meaningful to call directly... however you want"; this server takes that literally and reimplements the same pacing loop `start()`'s internal `runLoop()` uses, just with the mutex `ControlLoop` itself has no hook to accept:
   ```cpp
   auto next = std::chrono::steady_clock::now();
   while (!stopToken.stop_requested()) {
       { std::lock_guard lock(robotMutex); controlLoop.step(); }
       next += controlLoop.period();
       std::this_thread::sleep_until(next);
   }
   ```
   `ControlLoop::start()`'s *own* internal background thread is never used here — there is no way to inject an external lock into it, since it's entirely private to `ControlLoop`.
2. **The connection-serving loop**, running on the thread that called `TcpListener::accept()` (the main thread, in this single-client-at-a-time server — see Non-Goals): for each received request, it locks the same mutex, calls `CommandDispatcher::dispatch()`, unlocks, *then* sends the response bytes (network I/O deliberately happens outside the lock — no reason to hold it while writing to a socket).

`CommandDispatcher::dispatch()` itself takes no lock and knows nothing about threads — correctness here comes entirely from every *caller* of it (and of `controlLoop.step()`) agreeing to hold the same external mutex, which is why this is documented here rather than left implicit.

### 6.3 Command semantics (this server's own operational policy — not mandated by `docs/architecture.md`, resolved explicitly here)

- **Startup** (before accepting any connection): the controller is driven through `PowerOn → BootComplete → InitComplete` automatically, landing in `Idle` — a real controller's boot sequence isn't something an operator triggers per-command.
- **`Enable`**: `joint.enable()` on every joint, then `controller.handleEvent(ServoEnable)` followed immediately by `controller.handleEvent(ControllerReady)` — landing in `Ready`. This simulation has no separate "servo ready" hardware check to wait on, so `Enable` completes both transitions in one request rather than requiring a second command.
- **`Disable`**: `joint.disable()` on every joint, then `controller.handleEvent(ServoDisable)`.
- **`Home`**: `joint.home()` on every joint. No controller state transition — homing is joint-level only (there is no `Home` `ControllerEvent`).
- **`MoveJoint`**: decodes `robot::cli::MoveJointPayload`, calls `robot.joint(jointIndex).setTargetPosition(targetRadians)`; if the controller is currently `Ready`, also fires `CommandMove` to enter `Moving` (if already `Moving`, no additional transition is attempted). The control thread's already-running `ControlLoop::step()` picks up and executes the motion on its own from there — this is exactly the mechanism Phase 3 built.
- **`Stop`**: `controller.handleEvent(CommandStop)` immediately followed by `controller.handleEvent(StopComplete)` — this server has no separate background monitor for "motion has actually settled," so stopping is treated as synchronous/immediate.
- **`EmergencyStop`**: `estop.trigger()` (Phase 8) — engages every brake unconditionally and attempts the state transition, exactly as that class's own contract says.
- Any other `CommandType` (or a request whose payload fails to decode): `ResponseStatus::Error`, echoing the request's `CommandType`.

### 6.4 Server wiring (`apps/robot-emulator/main.cpp`)

- 6 joints, same `JointLimits` as Phase 11's benchmark scenario.
- 6 `robot::hardware::VirtualBrake`s, one per joint, wired into one `EmergencyStopController`.
- One `robot::safety::Watchdog`, fed `controlLoop.metrics().cyclesExecuted` (no lock needed for this specific read — `ControlLoopMetrics`' fields are already individual atomics, per Phase 3), timeout `100ms`, poll interval `10ms`. If it trips, a small dedicated thread calls `estop.trigger()` once (holding the shared mutex around that call, same as any other `Robot`/`ControllerStateMachine` access) — the first real, automatic consequence any watchdog trip has had anywhere in this codebase.
- `TcpListener::create(port)` (default port `9000`), then loop: `accept()` one client, serve it (receive → decode → dispatch → encode → send, repeating until the client disconnects), then `accept()` the next one.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_server` compiles as a static library depending on `robot_core`, `robot_controller`, `robot_hardware`, `robot_protocol`, `robot_cli`, and `robot_safety`.
- [ ] `CommandDispatcher` is unit-tested with **no real networking and no threading involved** — tests construct a `Robot`/`ControllerStateMachine`/brakes/`EmergencyStopController` directly and call `dispatch()` with hand-built or `robot_cli`-encoded request `Frame`s, exactly like Phase 7's own `Client` tests did against its fake server, but one level further in: this is the logic a real server's dispatch would run.
- [ ] Each of the seven supported commands is verified to produce the documented effect in section 6.3 and a well-formed `ResponseStatus::Ok` response; `GetStatus`'s response is verified to decode (via `robot::cli::decodeStatusPayload`) into a `StatusPayload` whose joint count and controller state name match the live objects' actual state at the time of the call.
- [ ] An unrecognized `CommandType` is verified to produce a `ResponseStatus::Error` response echoing the request's type, not a crash or a missing response.
- [ ] `apps/robot-emulator` builds as a real executable; run manually against a real `robotctl` invocation (`status`, `enable`, `move-joint`), confirmed end-to-end over a real loopback TCP connection — this is the acceptance criterion every prior phase's "known gap" note was waiting on.
- [x] Under Debug's sanitizers, running the server briefly while issuing several commands via `robotctl` produces no ThreadSanitizer/AddressSanitizer/UBSan report — the concurrency design in section 6.2 is exactly what this validates in practice, not just on paper. **Note:** the project's Debug default only enabled ASan/UBSan, which do not detect data races; validating this criterion required building with `-DROBOT_EMULATOR_SANITIZER=thread` (added to `CMakeLists.txt` by this phase specifically to make this checkable — see section 8). Two independent stress runs against the real server (sequential rapid-fire commands while the control thread and watchdog-response thread ran concurrently) plus Phase 3's and Phase 8's own threaded test suites (`robot_runtime_tests`, `robot_safety_tests`) were all re-run under real ThreadSanitizer instrumentation: zero race reports.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_server` and `robot-emulator` successfully alongside every existing target — every prior test suite passes unmodified.
- [ ] GoogleTest suite (`robot_server_tests`) covers `CommandDispatcher`'s acceptance criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.
- [ ] The README's "Known gap" note (present since Phase 7) is removed or updated to reflect that `apps/robot-emulator` now exists and works.

## 8. Notes for the Implementer

- Resist adding a second mutex "to be safe" around, say, just the `Watchdog`'s trip-response thread — one mutex, guarding exactly the `Robot`/`ControllerStateMachine` pair, held by every one of the three threads that ever touch them (control thread, connection thread, watchdog-response thread) for the smallest critical section that does the job. Multiple locks guarding overlapping state is how deadlocks get introduced later; this phase doesn't need more than one.
- `CommandDispatcher::dispatch()` deliberately takes no lock itself (see section 6.2) — resist the temptation to make it "self-contained" by giving it its own mutex; that would make it impossible for the server to correctly serialize it against the control thread's `step()` calls, which is the entire point of this phase's design.
- Reuse `robot::cli::encodeStatusPayload`/`decodeMoveJointPayload` (Phase 7) rather than re-implementing status/move-joint payload parsing a third time — this is exactly the payoff of Phase 7 having deliberately kept those free of any `robot_core`/`robot_controller` dependency.
- The response `Frame`'s `type` must always echo the request's `type` (per Phase 7's `Client::execute()`, which checks exactly this and returns `CliError::UnexpectedResponseType` if it doesn't match) — get this wrong and every `robotctl` command will fail with a confusing error even though the server "did the right thing" internally.
- **This phase adds `-DROBOT_EMULATOR_SANITIZER=thread` as a `CMakeLists.txt` cache variable.** ASan and TSan cannot be linked into the same binary, and the project's prior default (`-fsanitize=address,undefined`) cannot detect data races at all — it was silently validating everything *except* the one property this phase's entire design (section 6.2) exists to guarantee. Any future concurrency-sensitive component in this codebase should be validated the same way: build a second time with `-DCMAKE_BUILD_TYPE=Debug -DROBOT_EMULATOR_SANITIZER=thread`, not just trust the default Debug build's sanitizer output.
- **Deliberately not built here: async command queueing with a separate "queued" ack followed later by a "completed" result.** `dispatch()` is synchronous — one request produces exactly one response, and for `MoveJoint` specifically, that response means "the target was accepted and motion has started," not "motion has finished" (the control thread keeps converging afterward, unobserved by that request). This was a deliberate choice, not an oversight: `robotctl` (Phase 7) is connect-execute-disconnect per invocation with no persistent connection to push a later result to, and the wire format has no correlation ID to match multiple in-flight commands to their eventual results — adding either is a real protocol change, not a small addition. If a future need for real completion-notification or explicit multi-client queueing arises, the recommended approach is an external layer in front of this server (a proxy/gateway process) rather than complicating `CommandDispatcher`/the wire format themselves. A smaller, protocol-compatible option for "did the move finish" in the meantime: a `robotctl move-joint --wait` that polls `GetStatus` internally.
