# Phase 05 — Virtual Hardware: Motor, Encoder, Brake, IO

**Status:** Done
**Prerequisites:** None beyond Phase 1 being `Done` (for repository/toolchain conventions — this phase does not depend on `robot_core`, `robot_controller`, `robot_runtime`, or `robot_motion` at the code level; see Non-Goals).

---

## 1. Context

`docs/architecture.md` section 3.3's hardware model diagram lists, per joint, a `Motor`, `Encoder`, `Brake`, temperature/current sensors, and `Limits`, plus robot-level `Digital Inputs/Outputs` and `Analog Inputs/Outputs`. Per the roadmap (section 5), this phase covers `motor, encoder, brake, IO` — the temperature/current *sensors* are Phase 9's job, and wiring any of this into `Joint`'s per-cycle update (section 3.5's `motor_.update(dt); encoder_.update(dt);` sketch) is deliberately not this phase's job either (see Non-Goals — the same pattern Phase 4 followed for its trajectory planner).

This phase delivers four small, independent, simulated hardware components, each modeling one real physical characteristic that a genuine industrial actuator/sensor has and an idealized `Joint` (Phase 1) does not: a motor's electrical response lag, an encoder's finite resolution, a brake's fail-safe default state, and bounds-checked I/O channels.

## 2. Goal

A `robot_hardware` static library exposing `VirtualMotor`, `VirtualEncoder`, `VirtualBrake`, `DigitalIO`, and `AnalogIO` — each pure/deterministic, independently testable, and free of any dependency on the rest of this codebase's modules.

## 3. Non-Goals / Out of Scope

- Wiring any of these into `robot::core::Joint` (replacing or augmenting its internal model) — same reasoning as Phase 4's trajectory planner: that's a real behavioral change to already-shipped, already-tested code, and deserves its own phase with its own acceptance criteria for the integration itself.
- Temperature and current sensors — Phase 9.
- Anything E-stop/safety-related for `VirtualBrake` beyond exposing an unconditional `engage()` — the safety system that *decides when* to engage every brake robot-wide is Phase 8's job. This phase only models what one brake does once told to engage or release.
- Randomized/stochastic sensor noise on `VirtualEncoder` — this phase's encoder models quantization (a real, deterministic effect of finite resolution) but not noise, which would break this project's established testing philosophy of fully deterministic unit tests. A later phase could layer optional noise on top without changing this phase's interface.
- A generic "hardware bus" or registry tying named channels to physical meaning (e.g. "digital output 3 = gripper solenoid") — `DigitalIO`/`AnalogIO` here are raw, index-addressed channel arrays; giving channels names/roles is a configuration-layer concern (`docs/architecture.md` section 3.10), not this phase's.

## 4. Inputs

- `docs/architecture.md` section 3.3 (hardware model — this phase's primary spec) and section 3.5 (confirms `Motor`/`Encoder` are conceptually per-joint components, informing `VirtualMotor`/`VirtualEncoder`'s shape, without requiring this phase to actually attach them to `Joint`).
- `CONTRIBUTING.md` — coding standards and the mandatory file header.
- No file from Phases 1–4 is required reading — this phase's four components share no code with them (see Non-Goals on why no dependency exists).

## 5. Deliverables

```
include/robot/hardware/hardware_error.hpp
include/robot/hardware/virtual_motor.hpp
include/robot/hardware/virtual_encoder.hpp
include/robot/hardware/virtual_brake.hpp
include/robot/hardware/digital_io.hpp
include/robot/hardware/analog_io.hpp

src/hardware/virtual_motor.cpp
src/hardware/virtual_encoder.cpp
src/hardware/virtual_brake.cpp
src/hardware/digital_io.cpp
src/hardware/analog_io.cpp

tests/hardware/virtual_motor_test.cpp
tests/hardware/virtual_encoder_test.cpp
tests/hardware/virtual_brake_test.cpp
tests/hardware/io_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

All four components share one error enum — following Phase 4's precedent of reusing a single error type across closely related types rather than minting near-duplicate enums:

```cpp
namespace robot::hardware {

enum class HardwareError {
    InvalidConfiguration,  // Bad constructor/create() parameters.
    OutOfRange,             // A commanded value, or a channel index, is out of bounds.
};

}
```

### 6.1 `VirtualMotor` — first-order torque response

Models a motor's electrical/mechanical lag between a *commanded* torque and the *actual* torque it produces — real motors don't reach a new torque instantly. Uses the exact analytic solution of a first-order lag (`torque_new = commanded + (torque_old - commanded) * exp(-dt/tau)`), not Euler integration — this is exact for any `dt`, including large ones, unlike a naive per-step approximation.

```cpp
class VirtualMotor {
public:
    // max_torque must be > 0; time_constant (tau) must be > 0.
    [[nodiscard]] static std::expected<VirtualMotor, HardwareError> create(
        double max_torque, std::chrono::nanoseconds time_constant) noexcept;

    [[nodiscard]] double torque() const noexcept;             // Current actual torque. Starts at 0.
    [[nodiscard]] double commandedTorque() const noexcept;    // Current setpoint. Starts at 0.

    // Rejects |torque| > max_torque -> HardwareError::OutOfRange (commandedTorque() unchanged on failure).
    [[nodiscard]] std::expected<void, HardwareError> setCommandedTorque(double torque) noexcept;

    // Advances torque() toward commandedTorque() using the exact exponential solution above.
    void update(std::chrono::nanoseconds dt) noexcept;
};
```

### 6.2 `VirtualEncoder` — quantized position sensing

Models finite encoder resolution: a real encoder can only report one of `counts_per_revolution` discrete positions per revolution, not a continuous value. Stateless given a position — no internal state beyond configuration, so no `update()` method.

```cpp
class VirtualEncoder {
public:
    // counts_per_revolution must be > 0.
    [[nodiscard]] static std::expected<VirtualEncoder, HardwareError> create(
        std::uint32_t counts_per_revolution) noexcept;

    [[nodiscard]] double resolutionRadians() const noexcept;  // 2*pi / counts_per_revolution.

    // Raw integer count nearest true_position_radians / resolutionRadians()
    // (round-half-away-from-zero).
    [[nodiscard]] std::int64_t counts(double true_position_radians) const noexcept;

    // counts(true_position_radians) * resolutionRadians() — the quantized
    // position a real encoder reading would report.
    [[nodiscard]] double sample(double true_position_radians) const noexcept;
};
```

### 6.3 `VirtualBrake` — fail-safe holding brake

Models a normally-engaged ("fail-safe") holding brake: unpowered/unconfigured, it holds. `engage()`/`release()` are unconditional — no precondition, no failure mode — because Phase 8's safety system must be able to engage every brake robot-wide regardless of any other component's state.

```cpp
enum class BrakeState { Engaged, Released };

class VirtualBrake {
public:
    VirtualBrake() noexcept;  // Starts Engaged (fail-safe default).

    [[nodiscard]] BrakeState state() const noexcept;
    [[nodiscard]] bool isEngaged() const noexcept;

    void engage() noexcept;   // Idempotent.
    void release() noexcept;  // Idempotent.
};
```

### 6.4 `DigitalIO` / `AnalogIO` — bounds-checked I/O channels

Raw, index-addressed signal arrays — no semantic meaning attached to a channel index (see Non-Goals).

```cpp
class DigitalIO {
public:
    // All channels start false. channel_count == 0 is a valid (if useless)
    // configuration — every read()/write() on it simply returns OutOfRange.
    explicit DigitalIO(std::size_t channel_count) noexcept;

    [[nodiscard]] std::size_t channelCount() const noexcept;
    [[nodiscard]] std::expected<bool, HardwareError> read(std::size_t channel) const noexcept;
    [[nodiscard]] std::expected<void, HardwareError> write(std::size_t channel, bool value) noexcept;
};

class AnalogIO {
public:
    // min_value must be < max_value. All channels start at min_value.
    [[nodiscard]] static std::expected<AnalogIO, HardwareError> create(
        std::size_t channel_count, double min_value, double max_value) noexcept;

    [[nodiscard]] std::size_t channelCount() const noexcept;
    [[nodiscard]] std::expected<double, HardwareError> read(std::size_t channel) const noexcept;

    // Rejects an out-of-range channel OR a value outside [min_value, max_value]
    // -> HardwareError::OutOfRange either way (the channel's stored value is
    // unchanged on failure).
    [[nodiscard]] std::expected<void, HardwareError> write(std::size_t channel, double value) noexcept;
};
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_hardware` compiles as a static library with no dependency on any other `robot_*` target in this repository.
- [ ] `VirtualMotor::create()` rejects `max_torque <= 0` and `time_constant <= 0` (both `InvalidConfiguration`); `setCommandedTorque()` rejects `|torque| > max_torque` (`OutOfRange`, `commandedTorque()` unchanged).
- [ ] `VirtualMotor::update()` is verified against the closed-form exponential solution at at least one non-trivial `dt`/`tau` combination (e.g. after one time constant, `torque()` is within tolerance of `commanded * (1 - 1/e)`), and confirmed to converge closer to `commandedTorque()` after repeated updates.
- [ ] `VirtualEncoder::create()` rejects `counts_per_revolution == 0`. `counts()`/`sample()` are verified against hand-computed exact values for at least one `counts_per_revolution`, including a negative `true_position_radians`.
- [ ] `VirtualBrake` starts `Engaged`; `release()`/`engage()` are verified idempotent (calling either twice in a row leaves the same state) and never fail (no `std::expected` involved).
- [ ] `DigitalIO`/`AnalogIO` both reject an out-of-range channel index on `read()`/`write()`; `AnalogIO::write()` additionally rejects a value outside `[min_value, max_value]`, leaving the channel's prior value unchanged; `AnalogIO::create()` rejects `min_value >= max_value`.
- [ ] No dynamic heap allocation inside any `update()`/`read()`/`write()`/`sample()`/`counts()` call — `DigitalIO`/`AnalogIO` may allocate once, at construction, to size their channel storage (this is a one-time setup cost, not a per-cycle hot-path allocation, and is the same category of allocation `Robot`'s `std::vector<Joint>` already does in Phase 1's constructor).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_hardware` and its tests successfully alongside the existing targets — Phases 1–4's tests must keep passing unmodified.
- [ ] GoogleTest suites (`robot_hardware_tests`, one binary covering all four components' test files) cover all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- `VirtualMotor::update()`'s exponential formula needs `std::exp` (`<cmath>`) — same category of standard-library dependency `TrapezoidalTrajectory::create()` already took on for `std::sqrt` in Phase 4.
- `VirtualEncoder::counts()` should use `std::llround`, not a manual `static_cast<std::int64_t>(x + 0.5)` — the latter rounds incorrectly for negative values (this is exactly why the interface calls out "round-half-away-from-zero" explicitly: `std::llround` already implements that correctly for both signs, a hand-rolled version usually doesn't).
- All four components' tests can live in a single `robot_hardware_tests` binary (like `robot_core_tests` covers both `Joint` and `Robot`), split into one `.cpp` per component under `tests/hardware/` for readability, all linked into one executable.
- Don't add a `create()`-with-`std::expected` constructor to `DigitalIO` "for consistency" with `AnalogIO` — `DigitalIO` genuinely has no invalid configuration (any `channel_count`, including 0, is representable and well-defined), so a plain constructor is the honest signature; manufacturing a fallible path where none exists would just make callers handle an error that can never occur.
