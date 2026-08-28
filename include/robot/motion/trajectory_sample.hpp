// include/robot/motion/trajectory_sample.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::motion {

/// @brief One instant of a trajectory: position, velocity, and
///        acceleration, all signed consistently with the trajectory's
///        direction of travel (positive if moving from a lower to a
///        higher position, negative otherwise).
struct TrajectorySample {
    double position;      ///< Absolute position, same units/frame as the trajectory's start/target.
    double velocity;      ///< Signed velocity.
    double acceleration;  ///< Signed acceleration.
};

}  // namespace robot::motion
