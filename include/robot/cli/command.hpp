// include/robot/cli/command.hpp
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
#include <span>
#include <string>
#include <string_view>
#include "robot/cli/cli_error.hpp"

namespace robot::cli {

/// @brief The seven commands this phase's CLI supports.
///
/// See docs/task-briefs/phase-07-cli.md Non-Goals for why this is a subset
/// of robot::protocol::CommandType's full fourteen values.
enum class CommandKind {
    Status,
    Enable,
    Disable,
    Home,
    MoveJoint,
    Stop,
    EmergencyStop,
};

/// @brief One parsed command, with its command-specific arguments (if any).
struct Command {
    CommandKind kind;
    std::uint8_t jointIndex = 0;  ///< Only meaningful when kind == MoveJoint.
    double targetDegrees = 0.0;   ///< Only meaningful when kind == MoveJoint.
};

/// @brief A fully parsed `robotctl` invocation.
struct ParsedInvocation {
    std::string host;
    std::uint16_t port;
    Command command;
};

/// @brief Parses a robotctl invocation's arguments.
/// @param args Does NOT include argv[0] (the program name). Expected shape:
///        {"<host>:<port>", "<command-name>", "<command args...>"}.
/// @return A ParsedInvocation, or the specific CliError describing what was
///         wrong with args.
[[nodiscard]] std::expected<ParsedInvocation, CliError> parseArgs(std::span<const std::string_view> args) noexcept;

}  // namespace robot::cli
