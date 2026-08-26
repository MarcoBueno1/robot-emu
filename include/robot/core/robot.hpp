// include/robot/core/robot.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>
#include <expected>
#include <span>
#include <vector>
#include "robot/core/joint.hpp"
#include "robot/core/robot_config.hpp"

namespace robot::core {

/// @brief A robot: an ordered collection of Joint instances, built from a
///        RobotConfig.
///
/// Robot has no public constructor — use the create() named constructor,
/// which validates the configuration first so an invalid, partially
/// constructed Robot is never observable from the outside.
///
/// Thread-safety: none — like Joint, Robot does no internal synchronization
/// by design (see docs/task-briefs/phase-01-core.md, "Thread model"). A
/// single owner thread is expected to call update().
class Robot {
public:
    /// @brief Named constructor: builds a Robot from a validated RobotConfig.
    /// @param config Configuration to build from. Consumed (moved from) on success.
    /// @return A valid Robot, or the JointError returned by
    ///         config.validate() (e.g. InvalidConfiguration for zero joints
    ///         or inconsistent limits).
    [[nodiscard]] static std::expected<Robot, JointError> create(RobotConfig config);

    /// @brief Number of joints in this robot.
    [[nodiscard]] std::size_t jointCount() const noexcept { return joints_.size(); }

    /// @brief Joint at the given index (J1..JN order), mutable access.
    /// @param index Zero-based joint index; must be < jointCount().
    [[nodiscard]] Joint&       joint(std::size_t index) noexcept       { return joints_[index]; }
    /// @brief Joint at the given index (J1..JN order), read-only access.
    /// @param index Zero-based joint index; must be < jointCount().
    [[nodiscard]] const Joint& joint(std::size_t index) const noexcept { return joints_[index]; }

    /// @brief All joints, in stable J1..JN order, mutable access.
    [[nodiscard]] std::span<Joint>       joints() noexcept       { return joints_; }
    /// @brief All joints, in stable J1..JN order, read-only access.
    [[nodiscard]] std::span<const Joint> joints() const noexcept { return joints_; }

    /// @brief Advances all joints by dt, in stable order (J1..JN).
    /// @param dt Time step to advance, forwarded to each Joint::update().
    void update(std::chrono::nanoseconds dt) noexcept;

private:
    explicit Robot(std::string name, std::vector<Joint> joints) noexcept;

    std::string name_;
    std::vector<Joint> joints_;
};

}
