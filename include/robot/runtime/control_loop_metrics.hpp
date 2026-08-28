// include/robot/runtime/control_loop_metrics.hpp
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
#include <cstdint>

namespace robot::runtime {

/// @brief A point-in-time snapshot of a ControlLoop's timing statistics.
///
/// Individual fields are read from independent atomics inside ControlLoop
/// (see ControlLoop::metrics() documentation) — not captured as a single
/// atomic transaction. A caller reading this while the loop's background
/// thread is mid-cycle may observe a mix of "old" and "just-updated"
/// fields; this is a documented, acceptable imprecision for a
/// diagnostics/benchmark-facing snapshot, not a control-path correctness
/// issue.
struct ControlLoopMetrics {
    /// @brief Total number of completed step() calls (both direct calls
    ///        and calls made internally by the background thread).
    std::uint64_t cyclesExecuted = 0;

    /// @brief Mean wall-clock duration of one background-thread cycle.
    ///        Only updated by the threaded path (see ControlLoop::start());
    ///        remains zero if only step() has ever been called directly.
    std::chrono::nanoseconds averageCycleTime{0};

    /// @brief Largest observed absolute deviation between an actual
    ///        background-thread cycle period and the target period().
    ///        Only updated by the threaded path.
    std::chrono::nanoseconds maxJitter{0};

    /// @brief Number of background-thread cycles whose own work took
    ///        longer than period(), leaving no slack to sleep before the
    ///        next cycle's target time. Only updated by the threaded path.
    std::uint64_t deadlineMisses = 0;
};

}  // namespace robot::runtime
