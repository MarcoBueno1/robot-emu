// src/cli/command.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/cli/command.hpp"

#include <charconv>

namespace robot::cli {

namespace {

[[nodiscard]] std::expected<std::pair<std::string, std::uint16_t>, CliError> splitHostPort(
    std::string_view hostPort) noexcept {
    const auto colon = hostPort.rfind(':');
    if (colon == std::string_view::npos || colon == 0 || colon == hostPort.size() - 1) {
        return std::unexpected(CliError::InvalidHostPort);
    }

    const std::string_view hostPart = hostPort.substr(0, colon);
    const std::string_view portPart = hostPort.substr(colon + 1);

    std::uint16_t port = 0;
    const auto result = std::from_chars(portPart.data(), portPart.data() + portPart.size(), port);
    if (result.ec != std::errc{} || result.ptr != portPart.data() + portPart.size()) {
        return std::unexpected(CliError::InvalidHostPort);
    }

    return std::make_pair(std::string(hostPart), port);
}

[[nodiscard]] std::expected<double, CliError> parseDouble(std::string_view text) noexcept {
    double value = 0.0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
        return std::unexpected(CliError::InvalidArgument);
    }
    return value;
}

[[nodiscard]] std::expected<std::uint8_t, CliError> parseJointIndex(std::string_view text) noexcept {
    unsigned int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value > 255) {
        return std::unexpected(CliError::InvalidArgument);
    }
    return static_cast<std::uint8_t>(value);
}

}  // namespace

std::expected<ParsedInvocation, CliError> parseArgs(std::span<const std::string_view> args) noexcept {
    if (args.size() < 2) {
        return std::unexpected(args.empty() ? CliError::InvalidHostPort : CliError::MissingCommand);
    }

    auto hostPort = splitHostPort(args[0]);
    if (!hostPort.has_value()) {
        return std::unexpected(hostPort.error());
    }

    const std::string_view commandName = args[1];
    const std::span<const std::string_view> commandArgs = args.subspan(2);

    Command command;
    if (commandName == "status") {
        command.kind = CommandKind::Status;
    } else if (commandName == "enable") {
        command.kind = CommandKind::Enable;
    } else if (commandName == "disable") {
        command.kind = CommandKind::Disable;
    } else if (commandName == "home") {
        command.kind = CommandKind::Home;
    } else if (commandName == "stop") {
        command.kind = CommandKind::Stop;
    } else if (commandName == "estop") {
        command.kind = CommandKind::EmergencyStop;
    } else if (commandName == "move-joint") {
        if (commandArgs.size() < 2) {
            return std::unexpected(CliError::MissingArgument);
        }
        auto jointIndex = parseJointIndex(commandArgs[0]);
        if (!jointIndex.has_value()) {
            return std::unexpected(jointIndex.error());
        }
        auto targetDegrees = parseDouble(commandArgs[1]);
        if (!targetDegrees.has_value()) {
            return std::unexpected(targetDegrees.error());
        }
        command.kind = CommandKind::MoveJoint;
        command.jointIndex = jointIndex.value();
        command.targetDegrees = targetDegrees.value();
    } else {
        return std::unexpected(CliError::UnknownCommand);
    }

    return ParsedInvocation{std::move(hostPort->first), hostPort->second, command};
}

}  // namespace robot::cli
