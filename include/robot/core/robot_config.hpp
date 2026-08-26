// include/robot/core/robot_config.hpp
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
#include <string>
#include <vector>
#include "robot/core/joint_error.hpp"
#include "robot/core/joint_limits.hpp"

namespace robot::core {

/// @brief Declarative configuration used to build a Robot via Robot::create().
///
/// Not enforced to be valid by construction — call validate() (or go
/// through Robot::create(), which calls it internally) before relying on it.
struct RobotConfig {
    std::string name;                  ///< Human-readable robot name/model identifier.
    std::vector<JointLimits> joints;   ///< One entry per joint, in order J1..JN.

    /// @brief Validates the configuration as a whole: at least one joint,
    ///        and every joint's JointLimits individually valid.
    /// @return Success, or JointError::InvalidConfiguration (directly, or
    ///         propagated from the first invalid JointLimits::validate()).
    [[nodiscard]] std::expected<void, JointError> validate() const noexcept;
};

}
