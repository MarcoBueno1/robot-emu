// bridges/ros2_bridge/include/robot_emulator_ros2/hardware_interface.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// NOT build-verified — see docs/task-briefs/phase-13-ros2-bridge.md's
// warning. Written against this project's best knowledge of the
// ros2_control hardware_interface::SystemInterface API; not compiled
// against real ROS2 headers. Expect adjustments to be needed against
// whatever ROS2 distribution is actually targeted.
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <optional>
#include <string>
#include <vector>

#include "robot/cli/client.hpp"

namespace robot_emulator_ros2 {

/// @brief ros2_control SystemInterface plugin for apps/robot-emulator
///        (docs/architecture.md section 3.16).
///
/// Connects via robot::cli::Client (Phase 7), exactly like robotctl — the
/// only place in this bridge that knows anything about ROS2 is this class
/// itself; robot_core/robot_cli/apps/robot-emulator remain untouched and
/// ROS2-unaware. One instance, one TCP connection to one running
/// apps/robot-emulator — see the task brief's Non-Goals for why.
class RobotEmulatorHardwareInterface : public hardware_interface::SystemInterface {
public:
    /// @brief Reads host/port from info.hardware_parameters (falling back
    ///        to 127.0.0.1:9000, matching apps/robot-emulator's own
    ///        default), and sizes the per-joint state/command buffers to
    ///        info.joints.size().
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    /// @brief Exports one position and one velocity StateInterface per joint.
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /// @brief Exports one position CommandInterface per joint. No
    ///        velocity/effort command interface — see the task brief's Non-Goals.
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /// @brief Connects to apps/robot-emulator via robot::cli::Client::connect().
    /// @return CallbackReturn::ERROR if the connection fails — never
    ///         crashes or silently proceeds disconnected.
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /// @brief Disconnects (resetting client_ closes the underlying
    ///        TcpConnection, per Phase 6).
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /// @brief One GET_STATUS request over the binary protocol, filling
    ///        every joint's position/velocity state interface.
    /// @return hardware_interface::return_type::ERROR if the request
    ///         fails (e.g. apps/robot-emulator connection lost).
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /// @brief One MoveJoint request per joint, converting each command
    ///        interface's radian value to degrees (robot::cli::Command's
    ///        unit — see Phase 7) before sending.
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::optional<robot::cli::Client> client_;
    std::string host_ = "127.0.0.1";
    std::uint16_t port_ = 9000;

    std::vector<double> hwPositions_;
    std::vector<double> hwVelocities_;
    std::vector<double> hwCommandsPosition_;
};

}  // namespace robot_emulator_ros2
