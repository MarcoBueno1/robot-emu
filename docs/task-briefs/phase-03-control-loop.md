# Phase 03 — Deterministic Real-Time Control Loop

**Status:** Done
**Prerequisites:** Phase 1 (Core) — `Done`, Phase 2 (Controller State Machine) — `Done`. This is the first phase where `robot_core` and `robot_controller` are used together — until now they were fully independent.

---

## 1. Context

This phase builds the runtime that actually drives `robot::core::Robot` and `robot::controller::ControllerStateMachine` over time, at a configurable fixed frequency (`docs/architecture.md` section 3.6: `500 Hz | 1 kHz | 2 kHz | 5 kHz | 10 kHz`), collecting the basic timing metrics that later become the quantitative material for benchmarking (section 3.13) and `GET_STATUS` (section 3.7).

Section 3.6's full pipeline diagram (`Read Sensors → Update Controller → Compute Motion → Update Motors → Update Encoders → Safety Check`) assumes hardware components — sensors, motors, encoders — that don't exist yet (`Virtual Hardware` is Phase 5, `Sensors` is Phase 9). This phase implements the two steps that map to what already exists (`Update Controller` and `Compute Motion`, i.e. `Robot::update()`), and stubs the rest out as explicit non-goals below, so this brief doesn't quietly invent hardware interfaces that Phase 5 is supposed to own.

## 2. Goal

A `robot_runtime` static library exposing `ControlLoop`: a class that ties a `robot::core::Robot&` and a `robot::controller::ControllerStateMachine&` together and advances them at a configurable frequency, either one cycle at a time (synchronous, fully deterministic — for tests and for embedding in Phase 6/7's own loops) or continuously on a background thread (for real use), while tracking cycle count, average cycle time, jitter, and deadline misses.

## 3. Non-Goals / Out of Scope

- Reading real or simulated sensors, driving motors/encoders, or anything from the `Update Motors`/`Update Encoders`/`Read Sensors` boxes in section 3.6's diagram — those components don't exist until Phase 5 (Virtual Hardware) and Phase 9 (Sensors). This phase's cycle body is exactly: check controller state, and if `Moving`, call `Robot::update(dt)`.
- The independent watchdog thread mentioned in section 3.15 ("Safety watchdog independent from the main control loop... running on a separate thread, so a control-loop lockup doesn't prevent fault detection") — that is Phase 8's job, specifically *because* it must be independent of `ControlLoop`, not a feature of it. `ControlLoop`'s own `Safety Check` step in this phase is limited to the deadline-miss bookkeeping already described below; no E-stop/limit logic is added here.
- Anything that *drives* the `ControllerStateMachine` with real-world events (network commands from Phase 6, a CLI from Phase 7, a real watchdog from Phase 8) — this phase's tests drive it directly via `handleEvent()`, exactly as Phase 2's tests did. `ControlLoop` never calls `handleEvent()` itself; it only reads `state()`.
- Trajectory planning beyond what `Joint::update()` (Phase 1) already does — Phase 4's job.
- Exposing metrics or control over the binary protocol — Phase 6's job. This phase's metrics are readable in-process only, via `ControlLoop::metrics()`.
- CPU affinity, `SCHED_FIFO`/`PREEMPT_RT` tuning, or any other hard real-time guarantee — per section 3.6's own precision note, this project targets *soft* real-time (low latency/jitter, measured and reported), not a hard guarantee.

## 4. Inputs

- `docs/architecture.md`, sections 3.6 (this phase's primary spec) and 3.15 (no dynamic allocation/exceptions on the control path — applies to `step()` exactly as it did to `Joint::update()`/`Robot::update()`; the watchdog note explains why it's out of scope here, see above).
- `include/robot/core/robot.hpp`, `include/robot/core/clock.hpp` (Phase 1) — `ControlLoop` calls `Robot::update(std::chrono::nanoseconds)` and takes inspiration from `Clock`'s `now()` contract for its own timing, but does **not** take a `Clock&` dependency itself (see section 6, "Why no injected `Clock`").
- `include/robot/controller/controller_state_machine.hpp` (Phase 2) — `ControlLoop` reads `ControllerStateMachine::state()` only; it never calls `handleEvent()`.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/runtime/control_loop_error.hpp
include/robot/runtime/control_loop_frequency.hpp
include/robot/runtime/control_loop_metrics.hpp
include/robot/runtime/control_loop.hpp

src/runtime/control_loop.cpp

tests/runtime/control_loop_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

```cpp
namespace robot::runtime {

// The exact set of allowed frequencies from docs/architecture.md section 3.6.
// An enum, not a raw double/uint32_t, so an invalid frequency is a compile
// error, not a runtime one — no ControlLoopError case is needed for it.
enum class ControlLoopFrequency {
    Hz500   = 500,
    Hz1000  = 1000,
    Hz2000  = 2000,
    Hz5000  = 5000,
    Hz10000 = 10000,
};

enum class ControlLoopError {
    AlreadyRunning,  // start() called while already running.
    NotRunning,      // stop() called while not running.
};

// A point-in-time snapshot. Individual fields are read from independent
// atomics (see section 6, "On metrics() and atomicity") — not a single
// atomic transaction — documented, acceptable imprecision for this phase.
struct ControlLoopMetrics {
    std::uint64_t cyclesExecuted;
    std::chrono::nanoseconds averageCycleTime;
    std::chrono::nanoseconds maxJitter;      // max |actual cycle period - target period| observed
    std::uint64_t deadlineMisses;            // cycles whose own work took longer than the target period
};

class ControlLoop {
public:
    // robot and controller are borrowed (not owned) and must outlive this
    // ControlLoop and any running background thread — same convention as
    // Robot borrowing nothing and owning its Joints outright in Phase 1,
    // just the other direction: here, ControlLoop owns neither.
    ControlLoop(robot::core::Robot& robot,
                robot::controller::ControllerStateMachine& controller,
                ControlLoopFrequency frequency) noexcept;

    ~ControlLoop();  // Stops the background thread if still running.
    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    // Executes exactly one control cycle synchronously, on the caller's
    // thread: reads controller.state(); calls robot.update(period()) only
    // if that state is ControllerState::Moving; increments cyclesExecuted.
    // Deterministic, no sleep, no thread — the unit for tests, exactly as
    // Joint::update()/Robot::update() were in Phase 1.
    void step() noexcept;

    // Spawns a background thread that calls step() in a loop, paced to
    // this ControlLoop's configured frequency via std::this_thread::sleep_until
    // against std::chrono::steady_clock (see "Why no injected Clock" below).
    [[nodiscard]] std::expected<void, ControlLoopError> start();

    // Signals the background thread to stop and joins it. Safe to call
    // from any thread; blocks until the loop thread has actually exited.
    [[nodiscard]] std::expected<void, ControlLoopError> stop();

    [[nodiscard]] bool isRunning() const noexcept;

    // Target period for one cycle, derived from the configured frequency
    // (e.g. Hz1000 -> 1'000'000ns). Exposed so tests and callers can reason
    // about timing without recomputing it.
    [[nodiscard]] std::chrono::nanoseconds period() const noexcept;

    [[nodiscard]] ControlLoopMetrics metrics() const noexcept;

private:
    // ...
};

}  // namespace robot::runtime
```

**Why no injected `Clock&`:** Phase 1's `Clock` abstraction (`SteadyClock`/`ManualClock`) exists so *logic* that reads "now" can be tested without real time passing. `ControlLoop::step()` follows that same spirit by taking no clock at all — it is pure and synchronous, exactly like `Joint::update()`. But `start()`'s background thread does something `Clock` was never meant to support: it *sleeps* to pace itself (`std::this_thread::sleep_until`), which only makes sense against real wall-clock time. A `ManualClock` plugged into a background thread would either spin forever (nothing ever calls `advance()` from inside that thread) or require the test to call `advance()` concurrently, defeating the determinism `ManualClock` exists to provide. So: `step()` needs no clock at all (call it as many times as you want, however you want), and `start()` always paces against `std::chrono::steady_clock` directly, undecorated — this is a deliberate, documented simplification for this phase, not an oversight.

**On `metrics()` and atomicity:** `cyclesExecuted`, `deadlineMisses`, and the running sum used for `averageCycleTime` are each stored in their own `std::atomic`, updated only from the loop thread and read from any thread via `metrics()`. A caller reading `metrics()` while the loop thread is mid-cycle may observe a mix of "old" and "just-updated" fields (e.g. `cyclesExecuted` incremented but `averageCycleTime`'s sum not yet reflecting the same cycle) — this is a known, acceptable imprecision for a diagnostics/benchmark-facing snapshot, not a control-path correctness issue (`step()` itself never reads `metrics()`). Revisit only if a later phase needs a stronger consistency guarantee.

**Cycle semantics (authoritative):**
- `step()` calls `robot.update(period())` **if and only if** `controller.state() == robot::controller::ControllerState::Moving`. Every other state performs a no-motion cycle: no `Robot::update()` call, but `cyclesExecuted` still increments and timing is still recorded — a controller that's `Ready`, `Paused`, etc. still runs a steady cycle rhythm, it just isn't moving anything, matching section 3.6's framing of a continuously running loop with per-cycle "Safety Check" bookkeeping regardless of motion state.
- A "deadline miss" is counted in the *threaded* path only (`start()`'s loop), when the wall-clock time taken by one iteration's `step()` call plus bookkeeping exceeds `period()`, so the loop has no slack left to `sleep_until` before the next cycle's target time — in that case, skip the sleep and proceed immediately (do not try to "catch up" by running extra cycles back-to-back beyond the one already due). `step()` called directly (not via `start()`) never affects `deadlineMisses` — it has no notion of a deadline on its own, since it isn't paced.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_runtime` compiles as a static library depending on both `robot_core` and `robot_controller`, plus `Threads::Threads`.
- [ ] `step()` is covered by fully deterministic tests (no real sleep, no background thread) verifying: no motion in every non-`Moving` state; motion in `Moving` state (position changes consistent with `Robot::update(period())`, reusing the same `JointLimits` used in Phase 1's tests); `cyclesExecuted` increments exactly once per call regardless of state.
- [ ] `start()` returns `ControlLoopError::AlreadyRunning` (via `std::expected`) if called while already running, and `stop()` returns `ControlLoopError::NotRunning` if called while not running — both verified by test.
- [ ] A background-thread smoke test confirms `start()` actually advances `cyclesExecuted` over a short real interval, and `stop()` halts it (count stays constant after `stop()` returns) — bounded with a **generous, documented tolerance** (this test inherently depends on OS scheduling; assert `cyclesExecuted() > 0`, not an exact count — see Notes for the Implementer).
- [ ] `period()` returns the exact expected value for every `ControlLoopFrequency` enumerator (e.g. `Hz1000` → `1'000'000ns`), verified by test — no floating-point division involved (`std::chrono::nanoseconds(1'000'000'000 / static_cast<int>(frequency))` is exact for all five listed frequencies).
- [ ] The destructor stops a still-running background thread without the caller having to call `stop()` first — verified by test (construct, `start()`, let it run briefly, destroy, confirm no crash/leak under the sanitizers already enabled in Debug).
- [ ] No dynamic heap allocation inside `step()` — verified by code review (mirrors Phase 1's `Joint::update()` requirement from section 3.15).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (with `-fsanitize=address,undefined` — including with `start()`/`stop()` actually exercised, since this is the first phase with real threading and the sanitizers are exactly what would catch a data race or lifetime bug here) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_runtime` and its tests successfully alongside the existing `robot_core`/`robot_controller` targets — Phase 1 and Phase 2 tests must keep passing unmodified.
- [ ] GoogleTest suite (`robot_runtime_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with `robot::core`/`robot::controller`.

## 8. Notes for the Implementer

- `step()` and `start()`'s loop body should share the exact same "if `Moving`, call `robot.update(period())`" logic — don't duplicate it; have the threaded loop simply call `step()` each iteration and layer timing/metrics *around* that call, not inside it. This keeps `step()` trivially testable in isolation and keeps the threaded path's added complexity (sleeping, atomics) entirely separate from the cycle's actual work.
- Use `std::jthread` (C++20/23, available with this project's toolchain) for the background thread — its cooperative `stop_token`/`request_stop()` avoids needing a hand-rolled `std::atomic<bool> stop_flag` and gets you a safe, RAII-joining destructor almost for free, which directly satisfies the "destructor stops a still-running thread" acceptance criterion.
- The background-thread smoke test is the one place in this codebase so far that has to tolerate real timing variance. Keep it short (tens of milliseconds of real run time, not seconds) so the test suite stays fast, and keep the assertion loose (`> 0` cycles, or a wide plausible range) — do not assert an exact cycle count or tight timing bounds; that would make the test flaky under CI/sandbox scheduling noise for no real benefit, since the deterministic `step()` tests already cover the actual cycle logic precisely.
- `ControlLoopMetrics::averageCycleTime` only needs to be updated by the threaded path (it requires a real elapsed-time measurement between cycles); `step()` called on its own leaves it at its default value. Document this rather than trying to synthesize a meaningful "average" out of a single synchronous call with no timing reference.
- Don't add a way to change `frequency`/`period()` after construction — if a later phase needs that, it's a new constructor or a new type, not a mutation of a running `ControlLoop`'s invariants.
