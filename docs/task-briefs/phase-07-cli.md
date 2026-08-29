# Phase 07 — `robotctl` CLI

**Status:** Done
**Prerequisites:** Phase 6 (Binary Protocol over TCP) — `Done`. This phase depends on `robot_protocol` for `CommandType`/`Frame`/`FrameCodec`/`TcpConnection` — the first phase to build directly on Phase 6.

---

## 1. Context

`docs/architecture.md` section 3.11 shows an example `robotctl` session (`connect`, `status`, `move-joint`) but, like section 3.7 before it, specifies *requests* without specifying *responses* — there is no defined response wire format anywhere in the architecture doc, and no server implementing one exists yet (`apps/robot-emulator/` is still an empty placeholder directory in this repository). This phase resolves both gaps explicitly rather than guessing silently:

- **Response format**: this phase *defines* a minimal, generic response convention — a 1-byte status code followed by command-specific data — and documents it as this phase's own contract, not something lifted from an existing spec. A future phase building the real `apps/robot-emulator` server must honor this exact convention for `robotctl` to work against it unmodified (see section 8).
- **Session shape**: the doc's example shows `robotctl connect ...` as a separate invocation from `robotctl status`, implying persisted connection state across process invocations (a config/session file). This phase does **not** build that — see Non-Goals. Every `robotctl` invocation here is self-contained: `robotctl <host>:<port> <command> [args...]`, connect-execute-disconnect in one process run.
- **No real server exists to test against.** This phase's own tests use a minimal, test-only fake server (built directly on `robot_protocol`'s `TcpListener`, living only in the test binary) to verify `robotctl`'s wire-level correctness — it is not, and must not be mistaken for, a real robot controller implementation.

## 2. Goal

A `robot_cli` static library — argument parsing, the response wire format's encode/decode, a `Client` that executes one command over a real TCP connection, and status-table formatting — plus a real, runnable `robotctl` executable in `apps/robotctl/` built from it.

## 3. Non-Goals / Out of Scope

- **The real `apps/robot-emulator` server** — the process that would actually own a `Robot`/`ControllerStateMachine`/`ControlLoop`, listen via `TcpListener`, and execute commands for real. That's substantial, separate integration work (wiring together every module from Phases 1–6), not something this CLI-focused phase does incidentally. `robotctl` has nothing real to connect to after this phase — expected and worth being explicit about.
- **Persisted connection/session state across separate `robotctl` invocations** (a config file remembering the last `connect` target) — every invocation here takes `<host>:<port>` explicitly; see Context.
- **All fourteen `CommandType` values.** This phase covers seven, matching what the doc's example CLI session actually exercises plus the closest safety-relevant commands: `status`, `enable`, `disable`, `home`, `move-joint`, `stop`, `estop`. `MoveLinear`, `GetPosition`, `GetIO`, `SetIO`, `ResetFault`, `InjectFault`, and `Connect`-as-its-own-wire-command are not wired into `parseArgs()`/`Client` here — extending to them later means adding a new `CommandKind` case and payload type, not changing this phase's design.
- **Temperature in the status table.** The doc's example table has a `Temperature` column; that data doesn't exist anywhere in this codebase yet (Phase 9 — Sensors). This phase's `StatusPayload`/`formatStatusTable()` render `Position`/`Velocity` only.
- **Authentication/integrity** (`FLAG_AUTHENTICATED` from section 3.7's extensibility note) — Phase 14, same as noted in Phase 6's brief.

## 4. Inputs

- `docs/architecture.md` section 3.11 (example CLI session — informs argument shape and the status table's rendering, with the resolved gaps above) and section 3.7 (the underlying frame format this phase's responses reuse via `FrameCodec`).
- `include/robot/protocol/{command_type,frame,frame_codec,tcp_connection}.hpp` (Phase 6) — reused directly; this phase adds no new framing logic, only payload encode/decode for specific commands and a response-status convention on top of the existing `Frame`.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/cli/cli_error.hpp
include/robot/cli/command.hpp
include/robot/cli/response.hpp
include/robot/cli/status_payload.hpp
include/robot/cli/move_joint_payload.hpp
include/robot/cli/client.hpp
include/robot/cli/format.hpp

src/cli/command.cpp
src/cli/status_payload.cpp
src/cli/move_joint_payload.cpp
src/cli/client.cpp
src/cli/format.cpp

apps/robotctl/main.cpp

tests/cli/command_test.cpp
tests/cli/payload_test.cpp
tests/cli/format_test.cpp
tests/cli/client_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

### 6.1 Parsed invocation

```cpp
namespace robot::cli {

enum class CliError {
    InvalidHostPort, MissingCommand, UnknownCommand, MissingArgument, InvalidArgument,
    TransportFailure, ProtocolFailure, UnexpectedResponseType, ServerReportedError,
};

enum class CommandKind { Status, Enable, Disable, Home, MoveJoint, Stop, EmergencyStop };

struct Command {
    CommandKind kind;
    std::uint8_t jointIndex = 0;    // only meaningful when kind == MoveJoint
    double targetDegrees = 0.0;     // only meaningful when kind == MoveJoint
};

struct ParsedInvocation {
    std::string host;
    std::uint16_t port;
    Command command;
};

// args does NOT include argv[0] (the program name) — main.cpp strips it.
// Expected shape: {"<host>:<port>", "<command-name>", "<command args...>"}.
[[nodiscard]] std::expected<ParsedInvocation, CliError> parseArgs(std::span<const std::string_view> args) noexcept;

}
```

### 6.2 Response convention (this phase's own contract — see Context)

```cpp
namespace robot::cli {

enum class ResponseStatus : std::uint8_t { Ok = 0, Error = 1 };

}
```

Every response is a `robot::protocol::Frame` whose `type` echoes the request's `CommandType`, whose `payload[0]` is a `ResponseStatus`, and whose remaining payload bytes are command-specific: empty for a successful action command (`Enable`/`Disable`/`Home`/`MoveJoint`/`Stop`/`EmergencyStop`), or a serialized `StatusPayload` for a successful `GetStatus`. `ResponseStatus::Error` responses carry no further payload in this phase (no structured error code/message yet — `CliError::ServerReportedError` is all a caller gets).

### 6.3 `StatusPayload` — this phase's status wire format

```cpp
namespace robot::cli {

struct JointStatus {
    double positionRadians;
    double velocityRadiansPerSecond;
};

struct StatusPayload {
    std::string robotName;
    std::string controllerStateName;   // plain text, e.g. "Ready" — decoupled from
                                        // robot::controller::ControllerState; this
                                        // library does not depend on robot_controller.
    std::vector<JointStatus> joints;
};

[[nodiscard]] std::vector<std::byte> encodeStatusPayload(const StatusPayload& status);
[[nodiscard]] std::expected<StatusPayload, CliError> decodeStatusPayload(std::span<const std::byte> bytes);

}
```

Wire layout (big-endian, same convention as `FrameCodec`): `u16` name length + name bytes, `u16` state-name length + state-name bytes, `u16` joint count, then per joint `f64` position + `f64` velocity (each a big-endian-encoded IEEE-754 bit pattern, via `std::bit_cast<std::uint64_t>`).

### 6.4 `MoveJointPayload`

```cpp
namespace robot::cli {

struct MoveJointPayload {
    std::uint8_t jointIndex;
    double targetRadians;
};

[[nodiscard]] std::vector<std::byte> encodeMoveJointPayload(const MoveJointPayload& payload);
[[nodiscard]] std::expected<MoveJointPayload, CliError> decodeMoveJointPayload(std::span<const std::byte> bytes);

}
```

Wire layout: `u8` joint index, `f64` target (big-endian bit pattern, same as above).

### 6.5 `Client` and formatting

```cpp
namespace robot::cli {

class Client {
public:
    [[nodiscard]] static std::expected<Client, CliError> connect(const std::string& host, std::uint16_t port);

    struct Outcome {
        ResponseStatus status;
        std::optional<StatusPayload> statusPayload;  // set only for a successful Status command
    };

    // Encodes command, sends it, blocks for and decodes exactly one response
    // Frame. CliError::UnexpectedResponseType if the response's CommandType
    // doesn't match the request's.
    [[nodiscard]] std::expected<Outcome, CliError> execute(const Command& command);

private:
    explicit Client(robot::protocol::TcpConnection connection) noexcept;
    robot::protocol::TcpConnection connection_;
};

[[nodiscard]] std::string formatStatusTable(const StatusPayload& status);

}
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_cli` compiles as a static library depending on `robot_protocol` only (no `robot_core`/`robot_controller`/`robot_runtime`/`robot_motion`/`robot_hardware`).
- [ ] `parseArgs()` correctly parses all seven command kinds, including `move-joint`'s two extra arguments; rejects a malformed `host:port` (`InvalidHostPort`), a missing command name (`MissingCommand`), an unrecognized command name (`UnknownCommand`), and `move-joint` with too few/non-numeric arguments (`MissingArgument`/`InvalidArgument`) — each verified by test.
- [ ] `encodeStatusPayload()`/`decodeStatusPayload()` round-trip exactly for a `StatusPayload` with zero, one, and several joints, and for a robot name / state name containing non-trivial (but valid UTF-8/ASCII) text.
- [ ] `encodeMoveJointPayload()`/`decodeMoveJointPayload()` round-trip exactly, including a negative `targetRadians`.
- [ ] `decodeStatusPayload()`/`decodeMoveJointPayload()` both reject a truncated buffer (fewer bytes than the declared lengths/fixed layout require) with a `CliError` — verified by test with a hand-truncated buffer, not just round-trip cases.
- [ ] `Client::execute()` is verified end-to-end against a minimal test-only fake server (built on `robot_protocol::TcpListener` directly inside the test file) for at least: a `Status` command receiving a `StatusPayload` back, and an action command (e.g. `Enable`) receiving an empty-payload `Ok` response — both over a real loopback TCP connection, no sleeps.
- [ ] `Client::execute()` surfaces `CliError::ServerReportedError` when the fake server responds with `ResponseStatus::Error`, and `CliError::UnexpectedResponseType` when it responds with the wrong `CommandType` — both verified by test.
- [ ] `formatStatusTable()` output is verified by test to contain the robot name, controller state name, and each joint's position/velocity converted to degrees (matching the doc's example table's units) — an exact full-string match against the doc's illustrative table is not required (that example includes Temperature, which this phase doesn't render — see Non-Goals); check for the presence and correctness of the fields this phase does produce.
- [ ] `apps/robotctl` builds as a real executable linked against `robot_cli`, exits non-zero with a usage message on `parseArgs()` failure, and non-zero (with a message on stderr) if `Client::connect()`/`execute()` fails — not required to succeed against anything (there is nothing real to connect to yet; see Non-Goals).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers — including the fake-server TCP test) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_cli`, `robotctl`, and their tests successfully alongside the existing targets — Phases 1–6's tests must keep passing unmodified.
- [ ] GoogleTest suite (`robot_cli_tests`) covers all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- The `f64`-as-big-endian-bit-pattern encoding (`std::bit_cast<std::uint64_t>(double)`, then the same big-endian byte writing `FrameCodec` already uses for `u32`/`u16`) is duplicated logic between `status_payload.cpp` and `move_joint_payload.cpp` — small enough (a handful of lines) that a shared tiny header-only helper is reasonable, but don't reach back into `robot_protocol` to add it there; `FrameCodec` deals in frame headers only, and command payload formats are this phase's concern (see Non-Goals in Phase 6's own brief on why it doesn't interpret command contents).
- **Contract for whoever eventually builds `apps/robot-emulator`**: it must emit responses matching section 6.2/6.3/6.4 of this brief exactly (echo the request's `CommandType`, `payload[0]` as `ResponseStatus`, `StatusPayload`'s exact byte layout for `GetStatus`) for `robotctl` to work against it without any change to this phase's code. This isn't enforced by anything in this codebase yet — it's a contract carried only by this document until that future phase exists.
- `Client::execute()`'s fake test server does not need to be elaborate — decode one request `Frame` with `FrameCodec::decode()` (looping `TcpConnection::receive()` until enough bytes have arrived, same pattern as Phase 6's own `Tcp.ConnectAcceptSendReceiveRoundTrip` test), then hand-construct and send back whatever response the specific test case needs. Don't build a generic reusable fake-server abstraction for this — one inline lambda per test that needs one is enough, and keeps it obvious this is test scaffolding, not production code.
- `apps/robotctl/main.cpp` should be thin: convert `argv` to `std::vector<std::string>` (owning storage) then a `std::vector<std::string_view>` (what `parseArgs()` takes), call `parseArgs()` → `Client::connect()` → `Client::execute()`, and print/format the result. Keep any actual logic in the library, not in `main()`, so it's covered by `robot_cli_tests` rather than being untestable.
