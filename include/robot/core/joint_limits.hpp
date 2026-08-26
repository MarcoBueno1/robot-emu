// include/robot/core/joint_limits.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include "robot/core/joint_error.hpp"

namespace robot::core {

/// @brief Safety limits for a single joint: position range, and maximum
///        velocity/acceleration magnitudes.
///
/// A JointLimits instance is considered valid only if validate() succeeds;
/// Joint and Robot::create() rely on that check to reject malformed
/// configuration before any state is constructed.
struct JointLimits {
    double min_position;      ///< Minimum allowed position, in radians.
    double max_position;      ///< Maximum allowed position, in radians. Must be > min_position.
    double max_velocity;      ///< Maximum allowed velocity magnitude, in radians/second. Must be > 0.
    double max_acceleration;  ///< Maximum allowed acceleration magnitude, in radians/second^2. Must be > 0.

    /// @brief Validates internal consistency of the limits themselves
    ///        (min < max, positive velocity/acceleration bounds, etc.).
    /// @return Success, or JointError::InvalidConfiguration if the limits
    ///         are inconsistent.
    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

}
