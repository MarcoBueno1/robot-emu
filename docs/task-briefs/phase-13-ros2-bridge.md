# Phase 13 — ROS2 Bridge (`ros2_control` Plugin)

**Status:** Written, **NOT build-verified** — see the boxed warning below. This is the only phase in this project with that status; every other phase was compiled, tested, and run for real before being marked `Done`.
**Prerequisites:** Phase 6 (Protocol), Phase 7 (CLI), and the server integration — all `Done`.

---

> ⚠️ **This phase could not be compiled or tested.** ROS2 (`rclcpp`, `hardware_interface`, `pluginlib`) is not installed in the environment this code was written in, and could not be installed there either (`packages.ros.org` is outside that environment's permitted network access). Every other phase in this project was built and run for real, with actual test output, before being called `Done` — this phase breaks that pattern, and says so explicitly rather than presenting untested code with the same confidence as validated code. **Before relying on this bridge, build it on a real ROS2 (Humble or newer) installation and treat the acceptance criteria below as a checklist to run, not a report of what already happened.**

## 1. Context

`docs/architecture.md` section 3.16 already specifies this phase in unusual detail — class name, method signatures, the exact CMake isolation block, even the README positioning statement. This brief follows that spec closely rather than making its own design decisions, the way earlier briefs did when the architecture doc left more open. Section 3.16's own framing: not a simple topic bridge (publish `JointState`, subscribe to a command topic), but a `ros2_control` **`hardware_interface::SystemInterface` plugin** — the same integration point real vendor drivers (UR, Fanuc) use, so any standard `ros2_control` controller (`joint_trajectory_controller`, `position_controllers`, MoveIt) works against this emulator with no additional code, not just RViz visualization.

The bridge's `RobotEmulatorClient` is section 3.16's own name for reusing `robotctl`'s protocol wrapper — this phase reuses `robot::cli::Client` (Phase 7) directly, unmodified, exactly as `apps/robot-viewer` (Phase 12) already did. `robot_core` gains zero ROS2/DDS dependency; the bridge is a separate `bridges/ros2_bridge/` ROS2 package that talks to a running `apps/robot-emulator` over the same TCP binary protocol every other client uses.

## 2. Goal

A `bridges/ros2_bridge/` ROS2 (`ament_cmake`) package exporting `robot_emulator_ros2::RobotEmulatorHardwareInterface`, a `pluginlib`-registered `hardware_interface::SystemInterface` that connects to a running `apps/robot-emulator` via `robot::cli::Client` and exposes its joints as standard `ros2_control` state/command interfaces — built only when explicitly opted into, and never affecting a build without ROS2 installed.

## 3. Non-Goals / Out of Scope

- **Compiling or running any of this code.** See the warning above. This phase produces source and build-description files only.
- **Continuous state-streaming instead of per-cycle `GET_STATUS` polling.** Section 3.16 itself frames this as a future possibility enabled by the protocol's reserved `FLAGS` field ("if needed"), not a requirement now — `read()` polls `GET_STATUS` once per `ros2_control` cycle, exactly as the spec's own illustrative code shows.
- **Velocity or effort command interfaces.** Only a position command interface is exported, matching what `robot::cli::Command`/`MoveJoint` actually supports (Phase 7 never added velocity-mode commands either).
- **Any change to `robot_core`, `robot_cli`, or `apps/robot-emulator`.** The bridge is purely an additive consumer of the existing `robot::cli::Client` — nothing in the core-facing code changes to accommodate it.
- **Multi-robot / multiple simultaneous `apps/robot-emulator` connections from one bridge instance.** One `RobotEmulatorHardwareInterface` instance, one TCP connection, matching every other client in this codebase's "one connection" scope.

## 4. Inputs

- `docs/architecture.md` section 3.16 (this phase's authoritative spec) and section 3.7 (protocol/`FLAGS` field context for the streaming-mode note above).
- `include/robot/cli/client.hpp` (Phase 7) — reused directly; `RobotEmulatorHardwareInterface` holds a `robot::cli::Client`.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
bridges/ros2_bridge/package.xml
bridges/ros2_bridge/CMakeLists.txt
bridges/ros2_bridge/robot_emulator_ros2.xml   (pluginlib plugin description)
bridges/ros2_bridge/include/robot_emulator_ros2/hardware_interface.hpp
bridges/ros2_bridge/src/hardware_interface.cpp

CMakeLists.txt   (repository root — extended, not replaced: the exact conditional block from section 3.16)
README.md        (MODIFIED — Phase 13 marked written-not-verified, section 3.16's positioning statement added)
```

## 6. Interfaces / Contracts

```cpp
namespace robot_emulator_ros2 {

/// @brief ros2_control SystemInterface plugin for apps/robot-emulator
///        (docs/architecture.md section 3.16). NOT build-verified — see
///        this brief's warning.
///
/// Connects via robot::cli::Client (Phase 7), exactly like robotctl — the
/// only place in this bridge that knows anything about ROS2 is this class
/// itself; robot_core/robot_cli/apps/robot-emulator remain untouched and
/// ROS2-unaware.
class RobotEmulatorHardwareInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /// GET_STATUS over the binary protocol (via robot::cli::Client) fills
    /// the position/velocity state interfaces, once per ros2_control cycle.
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /// The position command interfaces' current values → one MoveJoint
    /// request per joint, over the binary protocol.
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
```

`on_init` reads `host`/`port` from `info.hardware_parameters` (falling back to the defaults above if absent — matching `apps/robot-emulator`'s own default port), and sizes `hwPositions_`/`hwVelocities_`/`hwCommandsPosition_` to `info.joints.size()`. `on_activate` calls `robot::cli::Client::connect(host_, port_)`; `on_deactivate` resets `client_` (destroying it closes the underlying `TcpConnection`, per Phase 6). `write()` converts each joint's command interface value (radians, standard ROS convention) to degrees before constructing a `robot::cli::Command` (`targetDegrees`, per Phase 7's CLI-facing unit choice) — the same conversion `apps/robot-viewer` already does in the opposite direction for display.

## 7. Acceptance Criteria — **a checklist for whoever builds this on real ROS2, not a report of validation already performed**

- [ ] Builds cleanly against a real ROS2 (Humble or newer) + `ros2_control` installation with `-DROBOT_EMULATOR_BUILD_ROS2_BRIDGE=ON`.
- [ ] With the flag left at its default `OFF`, the rest of this repository builds exactly as it did before this phase — verify by building without the flag and confirming every existing target and test still succeeds (this part *can* and should be verified in any environment, ROS2 or not).
- [ ] `ros2 control list_hardware_interfaces` (or the equivalent for the ROS2 distribution in use) shows the expected position/velocity state interfaces and position command interface, one set per configured joint, after loading this plugin against a running `apps/robot-emulator`.
- [ ] A standard `joint_trajectory_controller` successfully commands at least one joint through this plugin, observably matching `apps/robot-emulator`'s own `robotctl status` output for that joint.
- [ ] `on_activate` failing to connect (e.g. `apps/robot-emulator` not running) is confirmed to return `CallbackReturn::ERROR`, not to crash or silently proceed.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.

## 8. Notes for the Implementer (and for whoever validates this later)

- The `hardware_interface::SystemInterface` method set/signatures here reflect this project's best knowledge of the `ros2_control` API as of this writing, not a verified match against any specific ROS2 distribution's actual headers — `ros2_control`'s API has changed across distributions before (e.g. lifecycle method signatures, `StateInterface`/`CommandInterface` constructor shapes). **Expect to need adjustments** against whatever ROS2 version is actually targeted; this is exactly the kind of drift that not being able to compile against real headers in this environment couldn't catch.
- `robot::cli::Command`'s `jointIndex` field is `std::uint8_t` — `write()` must guard against `hwCommandsPosition_.size()` exceeding 256 joints (astronomically unlikely for a real robot, but worth a runtime check rather than a silent truncation).
- Keep `read()`/`write()` free of anything that could block indefinitely — `robot::cli::Client::execute()` blocks on `TcpConnection::receive()` with no timeout (Phase 6 never added one). A `ros2_control` cycle expects bounded-time `read()`/`write()` calls; a future hardening pass on this bridge (not this phase) might add a receive timeout to `TcpConnection` itself, or a bridge-local watchdog, if this proves to be a real problem in practice — flagged here rather than silently worked around with unverified code.
- `bridges/ros2_bridge/CMakeLists.txt` is a normal `ament_cmake` package `CMakeLists.txt` (its own `find_package(rclcpp REQUIRED)` etc., `pluginlib_export_plugin_description_file`, `ament_package()`) — not part of this project's own root `CMakeLists.txt`'s target graph beyond the single `add_subdirectory()` call section 3.16 already specifies.
