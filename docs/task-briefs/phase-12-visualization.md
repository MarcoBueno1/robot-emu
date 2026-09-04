# Phase 12 — Visualization: `robot-viewer` (WebSocket + Web UI)

**Status:** Done
**Prerequisites:** Phase 6 (Protocol), Phase 7 (CLI) — `Done`, and the server integration (`docs/task-briefs/server-integration.md`) — `Done`. `robot-viewer` connects to `apps/robot-emulator` as a `robot::cli::Client` (Phase 7), exactly like `robotctl` does.

---

## 1. Context

`docs/architecture.md` section 3.14 marks visualization optional and technology-unspecified — candidates listed are "OpenGL, SDL, Vulkan, WebSocket + Web UI, bridge to Gazebo/Webots — all as external protocol clients, never linked into the core." This phase picks **WebSocket + Web UI**: the only candidate that needs no graphics library or display server this environment can't provide, and the most natural fit for "external protocol client" — a second, independent process that talks to `apps/robot-emulator` the same way `robotctl` already does.

No WebSocket library exists anywhere in this codebase's approved dependencies (section 3.2 restricts this project to the STL plus POSIX sockets), so this phase implements the minimum of RFC 6455 a modern browser's `WebSocket` client actually needs: the HTTP upgrade handshake (which needs SHA-1 and Base64 — neither in the STL either, so both are implemented here too, validated against known RFC/standard test vectors) and unmasked server-to-client text frames.

## 2. Goal

A `robot_websocket` static library (SHA-1, Base64, and a minimal server-side WebSocket handshake/frame layer over `robot::protocol::TcpConnection`), plus a real `apps/robot-viewer` executable: it polls `apps/robot-emulator` for live status via `robot::cli::Client` (Phase 7, exactly like `robotctl`) and broadcasts each reading as JSON to one connected browser tab rendering a simple live joint-position display.

## 3. Non-Goals / Out of Scope

- **Full RFC 6455 compliance.** Implemented: the handshake, and unmasked server→client text frames (frames a server sends are never masked — that part of the spec is mandatory and simple). Not implemented: binary frames, fragmentation, ping/pong keepalive, and full validation of every client-frame edge case — this phase reads just enough of an incoming frame to detect a close frame (so the server notices a closed tab) and otherwise ignores client-sent data, since this viewer never needs the browser to send it anything beyond opening/closing the connection.
- **Multiple simultaneous viewer tabs.** One WebSocket client at a time, matching every other server-side component's "one connection at a time" scope in this codebase (Phase 6, the server integration).
- **3D rendering, OpenGL, or a robot mesh.** A simple 2D `<canvas>` schematic (one bar per joint, angle as rotation or fill) — enough to see the robot move, not a faithful rendering.
- **`wss://` (TLS).** Plaintext `ws://`, a local/development tool, consistent with `robotctl`'s own plaintext-only scope (Phase 6/7 never added TLS either).
- **Any command sent from the browser back to the robot.** This is read-only telemetry — `robot-viewer` polls `GetStatus` and relays it; it never sends `MoveJoint`/`Enable`/anything else. Controlling the robot from a browser would be a separate, future addition.
- **Serving the HTML page over HTTP.** `apps/robot-viewer/index.html` is a static file opened directly in a browser (`file://`) — building an HTTP static-file server alongside the WebSocket one is unnecessary scope; `file://` pages can open outbound `ws://` connections without issue in every mainstream desktop browser.

## 4. Inputs

- `docs/architecture.md` section 3.14 (this phase's mandate and technology choice) and section 3.2 (confirms no external library dependency is permitted, motivating the from-scratch SHA-1/Base64/handshake).
- `include/robot/protocol/tcp_listener.hpp`/`tcp_connection.hpp` (Phase 6) — the raw transport `robot_websocket` builds the handshake/framing on top of.
- `include/robot/cli/client.hpp` (Phase 7) — `robot-viewer` polls status through this, unmodified, exactly as `robotctl` does.
- `CONTRIBUTING.md` — coding standards and the mandatory file header.

## 5. Deliverables

```
include/robot/websocket/sha1.hpp
include/robot/websocket/base64.hpp
include/robot/websocket/websocket_error.hpp
include/robot/websocket/websocket_connection.hpp

src/websocket/sha1.cpp
src/websocket/base64.cpp
src/websocket/websocket_connection.cpp

tests/websocket/sha1_test.cpp
tests/websocket/base64_test.cpp
tests/websocket/websocket_connection_test.cpp

apps/robot-viewer/main.cpp
apps/robot-viewer/index.html

CMakeLists.txt   (repository root — extended, not replaced)
README.md        (MODIFIED — Phase 12 marked done, Quick Start extended)
```

## 6. Interfaces / Contracts

### 6.1 `sha1` / `base64` — pure, validated against known test vectors

```cpp
namespace robot::websocket {

/// @return The 20-byte SHA-1 digest of input.
[[nodiscard]] std::array<std::uint8_t, 20> sha1(std::string_view input) noexcept;

/// @return Standard (RFC 4648) Base64 encoding of input, with padding.
[[nodiscard]] std::string base64Encode(std::span<const std::uint8_t> input);

}
```

### 6.2 `WebSocketConnection`

```cpp
namespace robot::websocket {

enum class WebSocketError { HandshakeFailed, TransportFailure, ConnectionClosed };

class WebSocketConnection {
public:
    /// @brief Performs the server-side RFC 6455 opening handshake on an
    ///        already-accepted TCP connection: reads the HTTP upgrade
    ///        request, computes Sec-WebSocket-Accept (base64(sha1(client's
    ///        Sec-WebSocket-Key + the RFC 6455 GUID))), and writes the
    ///        HTTP 101 response.
    [[nodiscard]] static std::expected<WebSocketConnection, WebSocketError> accept(
        robot::protocol::TcpConnection connection);

    /// @brief Sends text as one unmasked, unfragmented WebSocket text frame.
    [[nodiscard]] std::expected<void, WebSocketError> sendText(std::string_view text);

    /// @brief Blocks for one incoming frame from the client. Only used to
    ///        detect a close frame (-> WebSocketError::ConnectionClosed);
    ///        any other frame's payload is read and discarded (see Non-Goals).
    [[nodiscard]] std::expected<void, WebSocketError> pollForClose();

private:
    explicit WebSocketConnection(robot::protocol::TcpConnection connection) noexcept;
    robot::protocol::TcpConnection connection_;
};

}
```

### 6.3 `apps/robot-viewer`

Connects to `apps/robot-emulator` (default `127.0.0.1:9000`, same default as `robotctl`) as a `robot::cli::Client`; binds a second `TcpListener` (default port `9001`) for browser connections. Loop: `accept()` one browser tab, `WebSocketConnection::accept()` the handshake, then every ~100ms call `client.execute(Command{Status})` and `sendText()` the result as a small JSON object (`{"state": "...", "joints": [{"position_deg": ..., "velocity_deg_s": ...}, ...]}`) until the tab closes or the emulator connection fails, then wait for the next browser tab.

`index.html`: a single static file, no build step, no external JS libraries — a `<canvas>` with inline JavaScript that opens `ws://127.0.0.1:9001`, parses each JSON message, and redraws one bar per joint (angle-proportional rotation or length).

## 7. Acceptance Criteria (Definition of Done)

- [ ] `sha1()` matches the standard published test vectors for `"abc"` and the empty string (both well-known, widely published SHA-1 reference values) — verified by test.
- [ ] `base64Encode()` matches RFC 4648's own worked examples (`"f"` → `"Zg=="`, `"fo"` → `"Zm8="`, `"foo"` → `"Zm9v"`, etc.) — verified by test.
- [ ] `WebSocketConnection::accept()` is verified against a real loopback TCP connection where the "client" side is a hand-built HTTP upgrade request (a real browser's request format, constructed by the test — not `robot_protocol`'s own binary framing), confirming the server computes the correct `Sec-WebSocket-Accept` value and responds with a `101` status line.
- [ ] `sendText()` is verified to produce bytes a hand-decoded RFC 6455 text frame parser (written in the test, decoding just enough to check the opcode/length/payload) recognizes as a well-formed, unmasked, unfragmented text frame containing exactly the sent text.
- [ ] `pollForClose()` is verified to return `WebSocketError::ConnectionClosed` when the client sends a close frame.
- [ ] `apps/robot-viewer` builds as a real executable; run manually against a real running `apps/robot-emulator` plus `index.html` opened in an actual browser, confirmed to show live-updating joint values while issuing `robotctl move-joint` commands against the same running emulator — this is this phase's real end-to-end acceptance check, the same spirit as the server integration's own.
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_websocket` and `robot-viewer` successfully alongside every existing target — every prior test suite passes unmodified.
- [ ] GoogleTest suite (`robot_websocket_tests`) covers the criteria above.
- [ ] Every deliverable file (except `index.html`, which is markup/JS, not C++) carries the standard header from `CONTRIBUTING.md`; `index.html` carries an equivalent HTML-comment header.
- [ ] Doxygen comments on every public C++ type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- The RFC 6455 handshake's magic GUID (`258EAFA5-E914-47DA-95CA-C5AB0DC85B11`) is fixed and public — concatenate it directly onto the client's `Sec-WebSocket-Key` header value, SHA-1 the result, then Base64-encode the 20-byte digest. Get this exact, or every browser's `WebSocket` constructor will reject the handshake outright (no partial credit — it's a single exact-match comparison on the browser's side).
- A minimal server-to-client text frame is: one byte `0x81` (FIN=1, opcode=0x1 text), then the payload length (7 bits if ≤125, or 126 + 2-byte length if larger — this viewer's JSON payloads are small enough that the 2-byte extended-length path is unlikely to be exercised in practice, but implement it anyway rather than assuming), then the raw UTF-8 payload bytes, unmasked (masking is client→server only, per spec).
- `pollForClose()` needs to *unmask* an incoming frame if reading its payload (client→server frames are always masked, using a 4-byte key present right after the length field) — even though this phase discards the payload content, the masking key still has to be read and applied correctly to advance past the frame's bytes without corrupting the connection's byte stream for whatever comes after.
- Keep `index.html` genuinely simple: no bundler, no framework, one `<script>` tag, plain `CanvasRenderingContext2D` calls. This is a diagnostic viewer, not a product UI.
