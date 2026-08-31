# Phase 09 — Sensors: Encoder, Temperature, Current, Proximity

**Status:** Done
**Prerequisites:** Phase 5 (Virtual Hardware) — `Done`. This phase depends on `robot_hardware` for `VirtualEncoder` and `HardwareError`, reused rather than duplicated. Does not depend on `robot_core`/`robot_controller`/`robot_runtime`/`robot_motion`/`robot_protocol`/`robot_cli`/`robot_safety`.

---

## 1. Context

`docs/architecture.md` section 3.8 lists five sensor types, but its own text marks the fifth explicitly: **"Force/Torque (later phase)"** — deferred even by the spec itself, despite appearing in the roadmap's one-line phase summary (section 5, row 9) without that caveat. This phase resolves that inconsistency by following section 3.8's more specific text: Force/Torque is out of scope here (see Non-Goals), the other four are this phase's actual deliverable.

Phase 5's `VirtualEncoder` already models quantization, and its own brief said noise/offset/failure-mode support "could layer optional noise on top without changing this phase's interface." This phase is that later layer — but as a **new, separate type** (`EncoderSensor`, composing a `VirtualEncoder` by value) rather than modifying `VirtualEncoder` itself, honoring that promise literally: Phase 5's class and its tests are untouched.

## 2. Goal

A `robot_sensors` static library exposing `EncoderSensor` (quantization + configurable Gaussian noise + offset + failure modes, wrapping `VirtualEncoder`), `TemperatureSensor` (first-order thermal lag, reusing the same exact-exponential-solution technique `VirtualMotor` established in Phase 5), `CurrentSensor`, and `ProximitySensor` (both simple stateless threshold classifiers).

## 3. Non-Goals / Out of Scope

- **Force/Torque sensing** (`Fx, Fy, Fz, Tx, Ty, Tz`) — section 3.8 marks this "(later phase)" explicitly; not this one.
- **Wiring any sensor into `Joint`/`Robot`/`ControlLoop`.** Same reasoning as every hardware-adjacent phase before this one (4, 5, 8's `LimitMonitor`): these are standalone, independently-tested components. A caller decides how/when to feed them real values (e.g. a future phase might call `EncoderSensor::read(joint.position())` once per control cycle) — that wiring doesn't exist yet.
- **A shared "sensor suite" aggregate** bundling one of each sensor type per joint — same category of not-yet-existing aggregate as `EmergencyStopController`'s brake span in Phase 8; a future integration phase's job.
- **Non-Gaussian or configurable noise distributions.** `EncoderSensor`'s noise is zero-mean Gaussian only, via `std::normal_distribution` — sufficient to model realistic sensor noise without adding a distribution-selection API surface nothing yet needs.
- **Persisting or streaming sensor readings** (e.g. into `GET_STATUS`'s response, or logging) — Phase 7's `StatusPayload` doesn't carry temperature/current data yet (documented as a known gap there); connecting the two is future work.

## 4. Inputs

- `docs/architecture.md` section 3.8 (this phase's primary spec, including the Force/Torque deferral quoted above) and section 3.9 (fault catalog — `ENCODER_FAILURE`/`OVER_TEMPERATURE`/`OVER_CURRENT` motivate `EncoderSensor`'s failure modes and the other sensors' threshold checks, though fault injection itself remains unimplemented, per Phase 8's Non-Goals).
- `include/robot/hardware/virtual_encoder.hpp` (Phase 5) — composed by value inside `EncoderSensor`, not modified.
- `include/robot/hardware/hardware_error.hpp` (Phase 5) — reused directly for every `create()` in this phase, following the same "one shared error enum" precedent `HardwareError` itself set for `VirtualMotor`/`VirtualEncoder`/`DigitalIO`/`AnalogIO`.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/sensors/encoder_sensor.hpp
include/robot/sensors/temperature_sensor.hpp
include/robot/sensors/current_sensor.hpp
include/robot/sensors/proximity_sensor.hpp

src/sensors/encoder_sensor.cpp
src/sensors/temperature_sensor.cpp
src/sensors/current_sensor.cpp
src/sensors/proximity_sensor.cpp

tests/sensors/encoder_sensor_test.cpp
tests/sensors/temperature_sensor_test.cpp
tests/sensors/current_sensor_test.cpp
tests/sensors/proximity_sensor_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

### 6.1 `EncoderSensor`

```cpp
namespace robot::sensors {

enum class EncoderFailureMode { None, Stuck, Disconnected };

class EncoderSensor {
public:
    /// @param encoder Copied in (VirtualEncoder is a small value type — see Phase 5).
    /// @param noiseStdDevRadians Standard deviation of zero-mean Gaussian
    ///        noise added after quantization. Must be >= 0; 0 disables noise.
    /// @param offsetRadians Constant systematic bias added to the true
    ///        position before quantization (models a mechanical mounting offset).
    /// @param seed PRNG seed — the same seed always produces the same
    ///        noise sequence, preserving deterministic testability.
    [[nodiscard]] static std::expected<EncoderSensor, robot::hardware::HardwareError> create(
        robot::hardware::VirtualEncoder encoder, double noiseStdDevRadians, double offsetRadians,
        std::uint64_t seed) noexcept;

    /// @brief Reads truePositionRadians through offset -> quantization ->
    ///        noise -> failureMode(), in that order.
    ///
    /// NOT const — advances this sensor's internal PRNG state on every
    /// call where failureMode() == None (a deliberate, documented
    /// departure from VirtualEncoder::sample()'s purity: a stateful PRNG
    /// is unavoidable once noise is involved).
    /// @return The reading, or NaN if failureMode() == Disconnected — a
    ///         real disconnected encoder reports no valid signal; NaN is
    ///         this simulation's explicit stand-in (callers must check
    ///         std::isnan()).
    [[nodiscard]] double read(double truePositionRadians) noexcept;

    [[nodiscard]] EncoderFailureMode failureMode() const noexcept;

    /// @brief Sets the failure mode. Transitioning into Stuck freezes
    ///        read()'s return value at whatever read() last returned
    ///        while failureMode() was None (or 0.0 if read() was never
    ///        called yet) — it does not resample truePositionRadians.
    void setFailureMode(EncoderFailureMode mode) noexcept;

private:
    // ...
};

}
```

### 6.2 `TemperatureSensor`

```cpp
namespace robot::sensors {

class TemperatureSensor {
public:
    /// @param ambientCelsius Starting temperature, and the value celsius()
    ///        approaches if update() is always called with
    ///        targetCelsius == ambientCelsius.
    /// @param timeConstant Same meaning as VirtualMotor's tau — time to
    ///        close ~63.2% of the gap to a new target. Must be > 0.
    /// @param maxSafeCelsius Threshold isOverTemperature() compares against.
    [[nodiscard]] static std::expected<TemperatureSensor, robot::hardware::HardwareError> create(
        double ambientCelsius, std::chrono::nanoseconds timeConstant, double maxSafeCelsius) noexcept;

    [[nodiscard]] double celsius() const noexcept;
    [[nodiscard]] bool isOverTemperature() const noexcept;

    /// @brief Advances celsius() toward targetCelsius using the same exact
    ///        analytic first-order-lag solution VirtualMotor::update()
    ///        established in Phase 5 — exact for any dt, not a per-step approximation.
    ///
    /// Unlike VirtualMotor (which separates setCommandedTorque() from
    /// update()), the target is passed directly to update() each call —
    /// deliberate: a real heat source (ambient conditions, nearby motor
    /// load) fluctuates continuously in a way that doesn't need a
    /// persistent "commanded" concept the way motor torque does.
    void update(std::chrono::nanoseconds dt, double targetCelsius) noexcept;

private:
    // ...
};

}
```

### 6.3 `CurrentSensor` / `ProximitySensor` — stateless threshold classifiers

```cpp
namespace robot::sensors {

class CurrentSensor {
public:
    /// maxSafeAmps must be > 0.
    [[nodiscard]] static std::expected<CurrentSensor, robot::hardware::HardwareError> create(
        double maxSafeAmps) noexcept;

    /// @return |amps| > maxSafeAmps — magnitude-based, since current can
    ///         be negative (e.g. regenerative braking direction) without
    ///         that alone indicating an overcurrent condition.
    [[nodiscard]] bool isOverCurrent(double amps) const noexcept;

private:
    double maxSafeAmps_;
};

class ProximitySensor {
public:
    /// triggerDistance must be > 0.
    [[nodiscard]] static std::expected<ProximitySensor, robot::hardware::HardwareError> create(
        double triggerDistance) noexcept;

    /// @return distance <= triggerDistance. Callers are expected to pass
    ///         a non-negative distance; this phase does not separately
    ///         validate that at the call site (a negative reading would
    ///         simply always be treated as triggered).
    [[nodiscard]] bool isTriggered(double distance) const noexcept;

private:
    double triggerDistance_;
};

}
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_sensors` compiles as a static library depending on `robot_hardware` only.
- [ ] `EncoderSensor` with `noiseStdDevRadians == 0` produces exactly the same reading as calling the wrapped `VirtualEncoder::sample()` directly on `truePositionRadians + offsetRadians` — verified by exact-equality test (no noise means no floating-point ambiguity to tolerate).
- [ ] Two `EncoderSensor`s constructed identically (same encoder config, noise stddev, offset, and **seed**) produce an identical sequence of `read()` outputs across several calls — verified by test, confirming deterministic reproducibility despite the internal PRNG.
- [ ] With a nonzero `noiseStdDevRadians`, the sample standard deviation of `(reading - noiseless quantized value)` across several hundred `read()` calls falls within a generous tolerance (e.g. ±20%) of the configured `noiseStdDevRadians` — a statistical test with deliberately loose bounds, not an exact-value assertion tied to `std::normal_distribution`'s implementation-specific output sequence.
- [ ] `EncoderFailureMode::Stuck` is verified to freeze `read()`'s return value at the last `None`-mode reading, even when subsequently called with a different `truePositionRadians`.
- [ ] `EncoderFailureMode::Disconnected` is verified to make `read()` return a value for which `std::isnan()` is true, across multiple calls.
- [ ] `TemperatureSensor::update()` is verified against the closed-form exponential solution at at least one non-trivial `dt`/`timeConstant` combination (same style of check Phase 5 used for `VirtualMotor`), and `isOverTemperature()` is verified both `false` below and `true` above `maxSafeCelsius`.
- [ ] `CurrentSensor::isOverCurrent()` and `ProximitySensor::isTriggered()` are each verified at, just below, and just above their threshold, including a negative-current case for `CurrentSensor` (confirming the magnitude-based check).
- [ ] Every `create()` rejects its documented invalid configuration (non-positive `timeConstant`/`maxSafeAmps`/`triggerDistance`, negative `noiseStdDevRadians`) with `robot::hardware::HardwareError::InvalidConfiguration` — verified by test.
- [ ] No dynamic heap allocation inside `EncoderSensor::read()`, `TemperatureSensor::update()`, `CurrentSensor::isOverCurrent()`, or `ProximitySensor::isTriggered()`.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_sensors` and its tests successfully alongside every existing target — Phases 1–8's tests, including Phase 5's `robot_hardware_tests`, must keep passing completely unmodified.
- [ ] GoogleTest suite (`robot_sensors_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- `EncoderSensor::create()` takes `VirtualEncoder` by value and stores it by value — `VirtualEncoder` has no heap-owned state (Phase 5), so this is a cheap, ordinary copy, not a concern.
- Use `std::mt19937_64` seeded directly from the `seed` parameter, and a member `std::normal_distribution<double>` constructed once (mean 0, the configured stddev) in `create()`/the constructor — don't reconstruct the distribution object on every `read()` call.
- `Stuck` mode's frozen value is whatever `read()` last returned while in `None` mode — not the encoder's live `sample()` of a stored position. Track this with a single `double lastReading_` member, updated only on the `None`-mode path, exactly as described in the interface's doc comment.
- The statistical noise-stddev test is the one place in this phase's tests that has an inherent (if small, given generous tolerance and several hundred samples) chance of flakiness — same category of tradeoff as any statistical test. Keep the sample count high enough (several hundred, as noted above) that the tolerance can stay generous without the test becoming meaningless.
- Don't add a way to change `noiseStdDevRadians`/`offsetRadians`/`seed` after construction, matching this codebase's established pattern (`robot::runtime::ControlLoop` in Phase 3 made the same call about its frequency) — reconstructing a new `EncoderSensor` is the answer if a caller ever needs different noise characteristics.
