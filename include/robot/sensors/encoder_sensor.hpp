// include/robot/sensors/encoder_sensor.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <expected>
#include <random>
#include "robot/hardware/hardware_error.hpp"
#include "robot/hardware/virtual_encoder.hpp"

namespace robot::sensors {

/// @brief Simulated failure states an EncoderSensor can be put into.
enum class EncoderFailureMode {
    None,          ///< Normal operation.
    Stuck,         ///< read() freezes at its last None-mode value.
    Disconnected,  ///< read() returns NaN.
};

/// @brief Wraps a robot::hardware::VirtualEncoder (Phase 5) with
///        configurable Gaussian noise, a systematic offset, and simulated
///        failure modes.
///
/// A new, separate type composing VirtualEncoder by value — not a
/// modification to VirtualEncoder itself, honoring Phase 5's own note that
/// noise "could layer optional noise on top without changing this phase's
/// interface." VirtualEncoder and its tests are untouched by this phase.
class EncoderSensor {
public:
    /// @param encoder Copied in — VirtualEncoder has no heap-owned state,
    ///        so this is a cheap, ordinary copy.
    /// @param noiseStdDevRadians Standard deviation of zero-mean Gaussian
    ///        noise added after quantization. Must be >= 0; 0 disables noise.
    /// @param offsetRadians Constant systematic bias added to the true
    ///        position before quantization (models a mechanical mounting offset).
    /// @param seed PRNG seed — the same seed always produces the same
    ///        noise sequence, preserving deterministic testability.
    /// @return A valid sensor, or HardwareError::InvalidConfiguration if
    ///         noiseStdDevRadians is negative.
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
    /// @return The reading, or NaN if failureMode() == Disconnected.
    [[nodiscard]] double read(double truePositionRadians) noexcept;

    [[nodiscard]] EncoderFailureMode failureMode() const noexcept;

    /// @brief Sets the failure mode. Transitioning into Stuck freezes
    ///        read()'s return value at whatever read() last returned
    ///        while failureMode() was None (0.0 if read() was never
    ///        called yet) — it does not resample truePositionRadians.
    void setFailureMode(EncoderFailureMode mode) noexcept;

private:
    EncoderSensor(robot::hardware::VirtualEncoder encoder, double noiseStdDevRadians, double offsetRadians,
                  std::uint64_t seed) noexcept;

    robot::hardware::VirtualEncoder encoder_;
    double noiseStdDevRadians_;
    double offsetRadians_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> noiseDist_;
    EncoderFailureMode failureMode_ = EncoderFailureMode::None;
    double lastReading_ = 0.0;
};

}  // namespace robot::sensors
