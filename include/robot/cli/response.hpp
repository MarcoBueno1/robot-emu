// include/robot/cli/response.hpp
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

namespace robot::cli {

/// @brief The first payload byte of every response Frame in this phase's
///        response convention (see docs/task-briefs/phase-07-cli.md
///        section 6.2 — this convention is defined by this phase, not by
///        docs/architecture.md, which specifies requests only).
enum class ResponseStatus : std::uint8_t {
    Ok = 0,
    Error = 1,
};

}  // namespace robot::cli
