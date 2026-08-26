// include/robot/core/joint_state.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::core {

/// @brief Instantaneous physical state of a single joint.
///
/// Plain data, no invariants enforced here — Joint is responsible for
/// keeping these values within JointLimits.
struct JointState {
    double position     = 0.0;  ///< Current position, in radians.
    double velocity     = 0.0;  ///< Current velocity, in radians/second.
    double acceleration = 0.0;  ///< Current acceleration, in radians/second^2.
    double torque       = 0.0;  ///< Current torque, in N·m (placeholder until Phase 5 — Virtual Hardware).
};

}
