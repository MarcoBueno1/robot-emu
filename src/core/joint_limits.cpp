// src/core/joint_limits.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/core/joint_limits.hpp"

namespace robot::core {

std::expected<void, JointError> JointLimits::validate() const noexcept {
    if (min_position >= max_position) {
        return std::unexpected(JointError::InvalidConfiguration);
    }
    if (max_velocity <= 0.0) {
        return std::unexpected(JointError::InvalidConfiguration);
    }
    if (max_acceleration <= 0.0) {
        return std::unexpected(JointError::InvalidConfiguration);
    }
    return {};
}

}
