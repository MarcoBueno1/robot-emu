// include/robot/fault/fault_injector.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>
#include "robot/fault/active_fault.hpp"
#include "robot/fault/fault_type.hpp"

namespace robot::fault {

/// @brief A decoupled fault registry and dispatcher.
///
/// FaultInjector records which faults are active and calls a registered
/// handler when one is injected — without knowing or caring what a
/// "motor" or "encoder" is. This mirrors robot::safety::Watchdog's
/// decoupling from robot::runtime::ControlLoop (Phase 8) exactly: a
/// generic mechanism, wired to something concrete by a caller this class
/// doesn't depend on. See docs/task-briefs/phase-10-fault-injection.md
/// Non-Goals for what this class deliberately does not do (simulate any
/// fault's physical effect itself).
///
/// Thread-safety: none — like ControllerStateMachine/TrapezoidalTrajectory,
/// this class spawns no thread and does no internal synchronization. A
/// single owner thread is expected to call its methods.
class FaultInjector {
public:
    using Handler = std::function<void(std::optional<std::size_t> jointIndex)>;

    FaultInjector() noexcept = default;

    /// @brief Registers handler to be called by every subsequent
    ///        inject(type, ...). Replaces any previously registered
    ///        handler for type — only one handler per FaultType is kept.
    void onFault(FaultType type, Handler handler);

    /// @brief Records (type, jointIndex) as active and, if a handler is
    ///        registered for type, invokes it with jointIndex.
    ///
    /// Idempotent with respect to activeFaults()/isActive(): injecting an
    /// already-active (type, jointIndex) pair again does not duplicate the
    /// entry — but the handler, if any, is still invoked every call, since
    /// a caller may deliberately want to re-trigger its effect.
    void inject(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt);

    /// @brief Clears one active fault. No-op, not an error, if it wasn't active.
    void clear(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt);

    /// @brief Clears every active fault. Does not affect registered handlers.
    void clearAll() noexcept;

    [[nodiscard]] bool isActive(FaultType type, std::optional<std::size_t> jointIndex = std::nullopt) const;

    /// @return Every currently active fault, in injection order.
    [[nodiscard]] std::vector<ActiveFault> activeFaults() const;

private:
    std::vector<ActiveFault> active_;
    // Keyed by the enum's underlying integer value rather than FaultType
    // itself — avoids needing a std::hash<FaultType> specialization for no
    // real benefit (see the task brief's Notes for the Implementer).
    std::unordered_map<int, Handler> handlers_;
};

}  // namespace robot::fault
