// include/robot/cli/cli_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::cli {

/// @brief Errors returned by robot::cli's fallible operations
///        (parseArgs(), payload encode/decode, Client).
///
/// Used exclusively with std::expected — no exceptions on the control
/// path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class CliError {
    InvalidHostPort,         ///< "<host>:<port>" argument was malformed.
    MissingCommand,          ///< No command name was given.
    UnknownCommand,          ///< Command name didn't match any CommandKind.
    MissingArgument,         ///< A command that needs arguments (e.g. move-joint) didn't get enough.
    InvalidArgument,         ///< An argument was present but not parseable (e.g. non-numeric).
    TruncatedPayload,        ///< A payload buffer was shorter than its declared/fixed layout requires.
    TransportFailure,        ///< The underlying TCP connection failed.
    ProtocolFailure,         ///< FrameCodec::decode() failed on a response.
    UnexpectedResponseType,  ///< A response's CommandType didn't match the request's.
    ServerReportedError,     ///< The response's ResponseStatus was Error.
};

}  // namespace robot::cli
