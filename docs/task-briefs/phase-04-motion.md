# Phase 04 — Motion: Trapezoidal Trajectory Planner

**Status:** Done
**Prerequisites:** Phase 1 (Core) — `Done`. This phase depends on `robot_core` for `robot::core::JointLimits` (velocity/acceleration bounds) — the first phase to *intentionally* build on `robot_core` this way, unlike `robot_controller` (Phase 2), which was kept independent on purpose. `robot_runtime` (Phase 3) is not a dependency of this phase.

---

## 1. Context

`docs/architecture.md` section 3.5 describes the per-cycle motion pipeline as `target position → trajectory planner → velocity controller → acceleration → motor model → joint position`. Phase 1's `Joint::update()` already does a version of "velocity controller → acceleration → position" — a saturated proportional controller that converges toward a target while respecting velocity/acceleration limits — but it has no explicit "trajectory planner" box: there's no upfront answer to "where will this joint be at time `t`", only a per-cycle reaction to the current error.

This phase builds that missing box as a standalone, deterministic component: `TrapezoidalTrajectory`. Given a start position, a target position, and a `JointLimits`, it precomputes a full time-parameterized motion profile — classic trapezoidal (accelerate / cruise / decelerate) when the distance is long enough to reach `max_velocity`, or triangular (accelerate / decelerate, no cruise) when it isn't — and can be sampled at any point in time to get position, velocity, and acceleration.

## 2. Goal

A `robot_motion` static library exposing `TrapezoidalTrajectory`: a pure, `noexcept`-friendly value type that precomputes a motion profile at construction and answers `sample(t)` for any `t`, with no per-call recomputation, no heap allocation, and no dependency on real time or on `robot::core::Joint`/`Robot` themselves.

## 3. Non-Goals / Out of Scope

- **Wiring this into `Joint::update()`, replacing Phase 1's proportional controller.** This phase delivers the planner as a standalone, independently-tested component only — swapping `Joint`'s internal motion strategy is a real behavioral change to already-shipped, already-tested Phase 1 code, and deserves its own deliberate phase (with its own test/acceptance criteria for the swap itself) rather than being folded silently into "add a trajectory planner." `Joint`'s public interface and behavior are unchanged by this phase.
- S-curve (jerk-limited) profiles — section 3.5 doesn't ask for jerk limiting, and trapezoidal is the standard baseline; S-curve would be a natural, separate future enhancement, not a requirement here.
- Multi-joint coordinated motion (e.g. `MOVE_LINEAR`'s Cartesian interpolation across several joints, keeping them synchronized) — that's Phase 6/7 protocol-command territory, layered on top of single-joint trajectories like this one.
- Any connection to `robot_controller`'s `ControllerState::Moving` or to `robot_runtime`'s `ControlLoop` — this phase's `TrapezoidalTrajectory` doesn't know either of those modules exist.

## 4. Inputs

- `docs/architecture.md` section 3.5 (motion pipeline this phase implements the missing piece of) and section 3.6 (confirms the control loop advances in fixed `dt` steps, which is why `sample()` is time-based rather than step-based).
- `include/robot/core/joint_limits.hpp` (Phase 1) — reused directly rather than duplicated: `JointLimits::max_velocity`/`max_acceleration` are exactly this phase's velocity/acceleration bounds, and `JointLimits::validate()` already rejects non-positive values, which this phase's `create()` reuses instead of re-implementing the same check under a different name.
- `include/robot/core/joint_error.hpp` (Phase 1) — this phase reuses `robot::core::JointError` (`InvalidConfiguration`, `PositionOutOfRange`) rather than inventing a parallel `TrajectoryError` enum with the same two cases under different names.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/motion/trajectory_sample.hpp
include/robot/motion/trapezoidal_trajectory.hpp

src/motion/trapezoidal_trajectory.cpp

tests/motion/trapezoidal_trajectory_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

```cpp
namespace robot::motion {

/// One instant of a trajectory: position, velocity, and acceleration,
/// all signed consistently with the trajectory's direction of travel.
struct TrajectorySample {
    double position;
    double velocity;
    double acceleration;
};

class TrapezoidalTrajectory {
public:
    // Fails exactly as JointLimits::validate() would (non-positive
    // max_velocity/max_acceleration -> InvalidConfiguration), or if either
    // position falls outside [limits.min_position, limits.max_position]
    // -> PositionOutOfRange, mirroring Joint::setTargetPosition()'s own check.
    [[nodiscard]] static std::expected<TrapezoidalTrajectory, robot::core::JointError>
    create(double start_position, double target_position,
           const robot::core::JointLimits& limits) noexcept;

    // Total time this trajectory takes to go from start to target. Zero if
    // start_position == target_position.
    [[nodiscard]] std::chrono::nanoseconds duration() const noexcept;

    // The state at time t. t is clamped into [0, duration()] first: t <= 0
    // returns the start state (position = start_position, velocity = 0,
    // acceleration = the initial acceleration this trajectory begins
    // with); t >= duration() returns exactly {target_position, 0.0, 0.0}
    // (no floating-point residue at the endpoint).
    [[nodiscard]] TrajectorySample sample(std::chrono::nanoseconds t) const noexcept;

private:
    // ...
};

}  // namespace robot::motion
```

**The profile itself (authoritative — implement exactly this):** let `D = |target_position - start_position|`, `s = sign(target_position - start_position)` (0 if equal), `v = limits.max_velocity`, `a = limits.max_acceleration`.

- If `D == 0`: `duration()` is zero; `sample()` always returns `{start_position, 0.0, 0.0}`.
- Let `d_acc = v*v / (2*a)` (distance covered while accelerating from 0 to `v`).
  - **Trapezoidal** (`2*d_acc <= D`): accelerate for `t_acc = v/a`, cruise at `v` for `t_cruise = (D - 2*d_acc)/v`, decelerate for `t_acc` again. `duration() = 2*t_acc + t_cruise`.
  - **Triangular** (`2*d_acc > D`, `v` is never reached): peak velocity `v_peak = sqrt(D*a)`, accelerate for `t_acc = v_peak/a`, decelerate for the same `t_acc`, no cruise phase. `duration() = 2*t_acc`.
- Within each phase, position/velocity/acceleration follow the standard constant-acceleration equations (`pos = pos0 + v0*Δt + 0.5*a*Δt²`, `vel = v0 + a*Δt`), scaled by `s` for direction, and offset from `start_position`/the end of the previous phase as appropriate.

This is the same shape of motion `Joint::update()` already approximates reactively in Phase 1 — this phase's contribution is computing it exactly, upfront, as a closed-form function of time.

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_motion` compiles as a static library depending only on `robot_core` (for `JointLimits`/`JointError`) — no dependency on `robot_controller` or `robot_runtime`.
- [ ] `create()` rejects non-positive `max_velocity`/`max_acceleration` with `JointError::InvalidConfiguration`, and a `start_position`/`target_position` outside `[min_position, max_position]` with `JointError::PositionOutOfRange` — both verified by test.
- [ ] At least one test exercises the **trapezoidal** branch (distance long enough for a cruise phase) and confirms: `sample(0)` is the start state; `sample()` at the boundary between accelerate/cruise reaches exactly `max_velocity`; `sample(duration())` is exactly the target state with zero velocity.
- [ ] At least one test exercises the **triangular** branch (distance too short to reach `max_velocity`) and confirms the peak velocity at the midpoint is below `max_velocity`, and position at the midpoint in time is at (approximately) half the total distance, by symmetry.
- [ ] A zero-distance trajectory (`start_position == target_position`) has `duration() == 0` and `sample()` returns the start/target state (they're equal) for any `t`, including `t == 0`.
- [ ] A negative-direction trajectory (`target_position < start_position`) is covered by at least one test, confirming velocity/acceleration signs are negative during motion and `sample(duration())` lands exactly on `target_position`.
- [ ] `sample()` is confirmed monotonic in position between `start_position` and `target_position` across many sampled points for at least one trajectory (no overshoot, no backtracking) — this is what distinguishes a precomputed profile from Phase 1's reactive controller, which is *not* guaranteed monotonic under all gains.
- [ ] `t < 0` and `t > duration()` are both covered by test, confirming clamping behavior exactly as specified above.
- [ ] No dynamic heap allocation inside `create()` or `sample()` — verified by code review (`TrapezoidalTrajectory` stores only plain `double`s/`std::chrono::nanoseconds` computed once at construction).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_motion` and its tests successfully alongside the existing targets — Phases 1–3's tests must keep passing unmodified.
- [ ] GoogleTest suite (`robot_motion_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- Compute everything possible once, in `create()`/the constructor — `t_acc`, `t_cruise` (or its absence), `duration()`, `d_acc`, and whichever peak velocity applies — so `sample()` is O(1) branch-and-arithmetic with no recomputation of profile-wide quantities per call.
- Convert the incoming `std::chrono::nanoseconds t` to a `double` seconds value once at the top of `sample()` (`std::chrono::duration<double>(t).count()`), then do the rest of the math in plain `double`s — consistent with how `Joint::update()` already handles `dt` in Phase 1.
- Don't special-case "is this the trapezoidal or triangular case" as a runtime `enum`/tag stored in the object if a single stored "effective cruise duration" (zero for triangular) lets `sample()`'s formulas naturally collapse to the triangular case without a separate code path — fewer branches to keep in sync is better here, but only if it doesn't make the acceleration-phase-boundary math harder to follow than an explicit two-case `if`. Use your judgment; both are acceptable as long as the acceptance criteria's exact-boundary assertions pass.
- `sample()`'s exact-endpoint guarantee (`t >= duration()` returns precisely `target_position`, not `target_position ± epsilon`) matters for the monotonicity test and for any future caller that checks `position == target` to decide a move is complete — don't let floating-point drift from the piecewise formulas leak through; clamp explicitly.
