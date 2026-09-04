// bridges/ros2_bridge/src/hardware_interface.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// NOT build-verified — see docs/task-briefs/phase-13-ros2-bridge.md's
// warning and its Notes for the Implementer for known API-drift risk
// against real ros2_control headers.
#include "robot_emulator_ros2/hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <numbers>

namespace robot_emulator_ros2 {

using hardware_interface::CallbackReturn;

CallbackReturn RobotEmulatorHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    if (auto it = info.hardware_parameters.find("host"); it != info.hardware_parameters.end()) {
        host_ = it->second;
    }
    if (auto it = info.hardware_parameters.find("port"); it != info.hardware_parameters.end()) {
        port_ = static_cast<std::uint16_t>(std::stoi(it->second));
    }

    const std::size_t jointCount = info.joints.size();
    if (jointCount > 255) {
        // robot::cli::Command::jointIndex is std::uint8_t — see the task
        // brief's Notes for the Implementer.
        return CallbackReturn::ERROR;
    }

    hwPositions_.assign(jointCount, 0.0);
    hwVelocities_.assign(jointCount, 0.0);
    hwCommandsPosition_.assign(jointCount, 0.0);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotEmulatorHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 2);
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hwPositions_[i]);
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hwVelocities_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotEmulatorHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hwCommandsPosition_[i]);
    }
    return interfaces;
}

CallbackReturn RobotEmulatorHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto connected = robot::cli::Client::connect(host_, port_);
    if (!connected.has_value()) {
        return CallbackReturn::ERROR;
    }
    client_ = std::move(connected.value());
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotEmulatorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Resetting client_ destroys it, which closes the underlying
    // TcpConnection (Phase 6's RAII socket handle).
    client_.reset();
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotEmulatorHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/) {
    if (!client_.has_value()) {
        return hardware_interface::return_type::ERROR;
    }

    auto outcome = client_->execute(robot::cli::Command{.kind = robot::cli::CommandKind::Status});
    if (!outcome.has_value() || !outcome->statusPayload.has_value()) {
        return hardware_interface::return_type::ERROR;
    }

    const auto& joints = outcome->statusPayload->joints;
    for (std::size_t i = 0; i < hwPositions_.size() && i < joints.size(); ++i) {
        hwPositions_[i] = joints[i].positionRadians;
        hwVelocities_[i] = joints[i].velocityRadiansPerSecond;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotEmulatorHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                        const rclcpp::Duration& /*period*/) {
    if (!client_.has_value()) {
        return hardware_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < hwCommandsPosition_.size(); ++i) {
        robot::cli::Command command{
            .kind = robot::cli::CommandKind::MoveJoint,
            .jointIndex = static_cast<std::uint8_t>(i),
            .targetDegrees = hwCommandsPosition_[i] * 180.0 / std::numbers::pi,
        };
        auto outcome = client_->execute(command);
        if (!outcome.has_value()) {
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

}  // namespace robot_emulator_ros2

PLUGINLIB_EXPORT_CLASS(robot_emulator_ros2::RobotEmulatorHardwareInterface, hardware_interface::SystemInterface)
