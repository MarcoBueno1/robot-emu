// include/robot/hardware/virtual_encoder.hpp
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
#include "robot/hardware/hardware_error.hpp"

namespace robot::hardware {

/// @brief Simulates a rotary encoder's finite resolution.
///
/// A real encoder can only report one of counts_per_revolution discrete
/// positions per revolution, not a continuous value. VirtualEncoder is
/// stateless given a position — it holds only its configured resolution,
/// so it has no update() method; call sample() with whatever "true"
/// position you want quantized.
class VirtualEncoder {
public:
    /// @brief Constructs an encoder with the given resolution.
    /// @param counts_per_revolution Discrete positions per full revolution
    ///        (2*pi radians). Must be > 0.
    /// @return A valid encoder, or HardwareError::InvalidConfiguration if
    ///         counts_per_revolution is 0.
    [[nodiscard]] static std::expected<VirtualEncoder, HardwareError> create(
        std::uint32_t counts_per_revolution) noexcept;

    /// @brief The angular distance, in radians, between two adjacent
    ///        reportable positions: 2*pi / counts_per_revolution.
    [[nodiscard]] double resolutionRadians() const noexcept;

    /// @brief The raw integer count nearest
    ///        true_position_radians / resolutionRadians(), rounded
    ///        half-away-from-zero (matches how real encoder counters behave
    ///        for both positive and negative positions).
    [[nodiscard]] std::int64_t counts(double true_position_radians) const noexcept;

    /// @brief The quantized position a real encoder reading would report:
    ///        counts(true_position_radians) * resolutionRadians().
    [[nodiscard]] double sample(double true_position_radians) const noexcept;

private:
    explicit VirtualEncoder(std::uint32_t counts_per_revolution) noexcept;

    std::uint32_t counts_per_revolution_;
};

}  // namespace robot::hardware
