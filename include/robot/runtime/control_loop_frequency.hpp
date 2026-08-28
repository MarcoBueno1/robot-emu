// include/robot/runtime/control_loop_frequency.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::runtime {

/// @brief The exact set of control loop frequencies allowed by
///        docs/architecture.md section 3.6.
///
/// An enum rather than a raw integer/double so that an invalid frequency is
/// a compile error, not a runtime one — no ControlLoopError case exists for
/// "invalid frequency" because it's structurally unrepresentable.
enum class ControlLoopFrequency {
    Hz500   = 500,    ///< 2 ms period.
    Hz1000  = 1000,   ///< 1 ms period.
    Hz2000  = 2000,   ///< 500 us period.
    Hz5000  = 5000,   ///< 200 us period.
    Hz10000 = 10000,  ///< 100 us period.
};

}  // namespace robot::runtime
