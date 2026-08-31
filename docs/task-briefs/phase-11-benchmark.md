# Phase 11 — Performance Optimization (Measured)

**Status:** Done
**Prerequisites:** Phase 1 (Core), Phase 3 (Control Loop), Phase 9 (Sensors) — all `Done`. This phase combines `robot_core`, `robot_controller`, `robot_runtime`, and `robot_sensors` in one benchmark executable; it adds no new library, only a new `apps/` binary and a README update.

---

## 1. Context

`docs/architecture.md` section 3.13 is explicit and non-negotiable about method: **"Real, measured numbers — never estimated — published in the README."** This phase takes that literally in both directions:

1. Build a real measurement harness (`apps/robot-benchmark`) that exercises the actual, already-implemented hot path — `ControlLoop::step()` (Phase 3) driving a 6-joint `Robot` (Phase 1) plus reading 24 attached sensors (Phase 9: 4 sensor kinds × 6 joints, matching section 3.13's example ratio exactly) — and reports real average/P99/P99.9 cycle time, CPU duty cycle, and process memory, in section 3.13's exact report format.
2. Only *then* decide whether any optimization is warranted, based on what that measurement actually shows — not by guessing at a bottleneck. Every prior phase already avoided the classic hot-path sins (no heap allocation in `Joint::update()`/`ControlLoop::step()`/`TrapezoidalTrajectory::sample()`, exact analytic solutions instead of iterative approximations in `VirtualMotor`/`TemperatureSensor`, a flat array instead of a hash map for `ControllerStateMachine`'s small transition table). If this phase's real measurement confirms that discipline already paid off and finds nothing worth changing, **that is this phase's legitimate, documented conclusion** — inventing an optimization unsupported by data would violate the same "never estimated" principle section 3.13 opens with.

**On the reported hardware:** this phase's numbers were measured in this repository's actual build/sandbox environment — a virtualized, single-core container (see the generated report for exact CPU/OS/compiler strings), not the dedicated multi-core workstation section 3.13's own example shows (`AMD Ryzen 7 | Ubuntu 25.10 | GCC 15.2`). Every number in this phase's benchmark output is real and reproducible by running the binary, but is not representative of a dedicated deployment target's absolute performance — only of this codebase's relative efficiency. This caveat is stated explicitly in the README rather than silently presenting sandbox numbers as if they were workstation numbers.

## 2. Goal

An `apps/robot-benchmark` executable that runs the exact report format from section 3.13 against a real 6-joint, 24-sensor scenario, using genuinely measured timing (not estimates), plus a `## Benchmark` section added to the root `README.md` publishing one real run's output — per section 3.13's explicit requirement that these numbers live in the README, not just in a tool nobody runs.

## 3. Non-Goals / Out of Scope

- **Building `apps/robot-emulator`** (the real TCP server) — still not built (see Phase 7's Non-Goals); this benchmark exercises `Robot`/`ControlLoop`/sensors directly, in-process, with no networking involved, which is sufficient to measure the computational core section 3.13 asks about.
- **Benchmarking the protocol/TCP layer** (`robot_protocol`, Phase 6) — a legitimate, separate future benchmark; this phase's cycle-time measurement is specifically the control-loop/sensor workload, matching section 3.13's example fields (`Joints`, `Sensors`, `Average cycle`, etc., none of which are protocol-related).
- **Continuous/automated benchmark tracking** (CI regression alerts, historical trend graphs) — this phase produces one honest, reproducible snapshot; running it again and comparing by hand is left to whoever needs that later.
- **Force-fitting an optimization.** If measurement shows nothing worth changing, this phase does not manufacture a change anyway — see Context.
- **Actually reaching 1 kHz via a real paced background thread for the *measurement* itself.** The benchmark measures `step()`'s own computational cost by calling it directly, synchronously, in a tight loop (no `sleep_until` pacing) — this isolates the work being measured from scheduling/sleep-granularity noise, which would only obscure the number section 3.13 actually wants (how expensive is one cycle's work, not how precisely the OS can wake a thread — that second question was already Phase 3's `ControlLoopMetrics::maxJitter`/`deadlineMisses`, a different, already-answered concern).

## 4. Inputs

- `docs/architecture.md` section 3.13 (this phase's primary spec and required report format) and section 3.6 (confirms cycle-time/jitter/deadline-miss vocabulary this phase reuses, established in Phase 3).
- `include/robot/runtime/control_loop.hpp` (Phase 3) — `step()` is the benchmark's primary subject.
- `include/robot/core/robot.hpp`, `include/robot/controller/controller_state_machine.hpp` — build the scenario `ControlLoop` drives.
- `include/robot/sensors/*.hpp` (Phase 9) — the 24 sensors read once per benchmarked cycle.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
apps/robot-benchmark/main.cpp

README.md   (MODIFIED — adds a ## Benchmark section with one real run's output)
CMakeLists.txt   (repository root — extended, not replaced)
```

No new static library — this phase is one executable plus a README update, consistent with `robotctl` (Phase 7) being this codebase's other `apps/`-only, no-new-library phase.

## 6. Scenario and Report Format (authoritative)

- **Joints:** 6, `JointLimits` matching the values used throughout Phases 1–9's own tests (`min=-180°, max=180°, max_velocity=2 rad/s, max_acceleration=5 rad/s²` in radians).
- **Sensors:** 24 — one `EncoderSensor`, `TemperatureSensor`, `CurrentSensor`, and `ProximitySensor` per joint (4 × 6). Every benchmarked cycle reads all 24 (an `EncoderSensor::read()` of the joint's current position, a `TemperatureSensor::update()` step, and threshold checks on the other two) — this is what makes "Sensors: 24" a real, measured part of the workload rather than a decorative report field.
- **Controller state:** driven to `Moving` before measurement starts (via the boot chain + `ServoEnable` + `ControllerReady` + `CommandMove`, same event sequence Phase 3's own tests use), and each joint given a small nonzero target so `Robot::update()` inside `step()` is doing real convergence work every cycle, not a no-op.
- **Sample count:** 100,000 cycles at the `Hz1000` period (matching section 3.13's "Control frequency: 1 kHz" line) — enough for a stable P99.9 estimate without an unreasonably long benchmark run.
- **Per cycle:** time only the workload (`controlLoop.step()` + the 24 sensor reads) via `std::chrono::steady_clock`, stored as one `std::vector<std::chrono::nanoseconds>` entry per cycle.
- **After the run:** sort the samples to compute the average, P99 (`samples[floor(0.99 * n)]`), and P99.9 (`samples[floor(0.999 * n)]`); CPU duty cycle = `average_cycle_time / period * 100%`; peak resident memory via `getrusage(RUSAGE_SELF, ...).ru_maxrss` (kilobytes on Linux — POSIX, consistent with this project's existing `<sys/socket.h>`-class dependencies from Phase 6); deadline misses = 0 by construction, since this synchronous measurement loop has no pacing to miss (state this explicitly in the printed report rather than reusing `ControlLoop`'s own `deadlineMisses` metric, which measures something this benchmark deliberately doesn't exercise — see Non-Goals).
- **CPU/OS/Compiler strings:** read at runtime from `/proc/cpuinfo`'s `model name` line and `/etc/os-release`'s `PRETTY_NAME`, and at compile time from `__VERSION__` (or the equivalent predefined macro) plus the `CMAKE_BUILD_TYPE` the binary was built with (passed in via a compile definition) — real, not hardcoded, so the report is honest on whatever machine actually runs it.
- **Output format:** exactly section 3.13's layout (field names, order, units) — copy its structure, populate every field with a real measured/detected value.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `apps/robot-benchmark` builds as a real executable, runs to completion, and prints a report matching section 3.13's field layout exactly, with every field populated from a real measurement or runtime/compile-time detection — no hardcoded/placeholder values anywhere in the output.
- [ ] The benchmark actually drives the controller to `Moving` and gives every joint a nonzero target before measuring — verified by running it and confirming joint positions are non-static (e.g., print one joint's position before/after the run, or add a debug assertion during development that at least one `Robot::update()` call inside the measured loop actually changed a position) — a benchmark of an idle robot would understate real per-cycle cost.
- [ ] All 24 sensors are genuinely read every measured cycle — verified by code review of the measured loop body, not just declared in a comment.
- [ ] P99 and P99.9 are computed from the actual sorted sample distribution (not estimated from the average via some formula) — verified by code review of the percentile computation.
- [ ] The root `README.md` gains a `## Benchmark` section (linked from the Documentation table, matching this project's existing convention of every major doc having an entry there) containing one real run's full output, plus the environment caveat from this brief's Context (virtualized/sandboxed numbers, not a dedicated-workstation reference).
- [ ] Running the benchmark binary twice in the same environment produces average/P99/P99.9 values within a reasonable relative range of each other (not required to be bit-identical — real timing measurements vary run to run — but not wildly different either, which would indicate a measurement bug rather than real variance).
- [ ] This phase's own conclusion — whether an optimization was made, and why, or why none was warranted — is written down in this brief's Notes for the Implementer (see below) and/or the README's Benchmark section, backed by the actual measured numbers, not asserted without them.
- [ ] The build (the benchmark binary itself, plus everything else) passes with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` and `Release` — though the benchmark should specifically be *run* under a `Release` build for the reported numbers to be meaningful (Debug's sanitizers add real overhead that would misrepresent the measurement — state which build type the published README numbers came from).
- [ ] `CMakeLists.txt` builds `robot-benchmark` successfully alongside every existing target — Phases 1–10's tests must keep passing unmodified (no test suite changes in this phase; there is nothing here to unit-test in the traditional sense — see Notes).
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.

## 8. Notes for the Implementer

- This phase has no `tests/` deliverable and no GoogleTest suite — a benchmark's "correctness" is that it measures real things correctly, not a set of pass/fail assertions about performance thresholds (hardcoding "cycle time must be under Xµs" into a unit test would immediately become flaky across different machines, exactly the kind of environment-coupling this brief's own Context section argues against for the *published* numbers). Acceptance here is via code review of the measurement methodology (see Acceptance Criteria) plus visually inspecting one real run's output.
- Use `std::nth_element` (not a full `std::sort` if it's not otherwise needed) to find the P99/P99.9 cutoff efficiently for 100,000 samples — though at this sample count either approach finishes essentially instantly, so don't over-engineer this; a full sort is simpler to read and entirely fine here.
- Reading `/proc/cpuinfo`/`/etc/os-release` is Linux-specific, consistent with this project's Linux-only toolchain (section 3.1) and every POSIX-only piece built since Phase 6 — no new platform assumption is being introduced here.
- **This phase's actual conclusion** (fill in after running the real benchmark, do not guess ahead of the data): document in the README's Benchmark section whether the measured numbers prompted any code change, and if so, what and why, with a before/after comparison; if not, say so plainly — "measured, found already efficient, no change made" is a complete and honest answer to a phase titled "(measured)."
