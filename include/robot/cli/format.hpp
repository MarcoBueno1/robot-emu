// include/robot/cli/format.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <string>
#include "robot/cli/status_payload.hpp"

namespace robot::cli {

/// @brief Renders status as human-readable text, in the spirit of
///        docs/architecture.md section 3.11's example `robotctl status`
///        output (robot name, controller state, a per-joint table).
///
/// Position/velocity are converted from radians to degrees (matching that
/// example's units). No Temperature column — that data doesn't exist in
/// this codebase yet (see the phase brief's Non-Goals).
[[nodiscard]] std::string formatStatusTable(const StatusPayload& status);

}  // namespace robot::cli
