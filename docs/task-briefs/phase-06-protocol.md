# Phase 06 — Binary Protocol over TCP

**Status:** Done
**Prerequisites:** None beyond Phase 1 being `Done` (repository/toolchain conventions only). This phase does not depend on `robot_core`, `robot_controller`, `robot_runtime`, `robot_motion`, or `robot_hardware` at the code level.

---

## 1. Context

`docs/architecture.md` section 3.7 defines a versioned binary frame: `MAGIC | VERSION | FLAGS | TYPE | LENGTH | PAYLOAD`, carrying one of fourteen named commands (`CONNECT`, `GET_STATUS`, `ENABLE`, ... `INJECT_FAULT`). Section 3.2 sanctions POSIX sockets as a core dependency (no Qt/Boost/ROS-style networking library). This phase has two genuinely separate concerns, and keeping them separate is itself part of the design:

1. **The frame format** — pure byte encoding/decoding, no I/O, fully deterministic and unit-testable with zero networking involved (`FrameCodec`).
2. **TCP transport** — real POSIX sockets moving bytes over a real loopback connection (`TcpListener`/`TcpConnection`).

Neither knows about the other beyond `FrameCodec` producing/consuming the `std::byte` buffers that `TcpConnection::send()`/`receive()` move. Neither knows what a `MOVE_JOINT` command *does* — that's deferred (see Non-Goals).

## 2. Goal

A `robot_protocol` static library exposing: `CommandType` (the fourteen commands, as a typed enum), `Frame`/`FrameCodec` (encode a `Frame` to wire bytes, decode wire bytes back to a `Frame`, distinguishing "not enough bytes yet" from a genuine protocol error), and `TcpListener`/`TcpConnection` (bind/accept/connect/send/receive over real loopback TCP, tested without any timing-based sleeps or flakiness).

## 3. Non-Goals / Out of Scope

- **Interpreting what a command means.** Nothing in this phase calls into `robot_core`/`robot_controller`/anything else — a decoded `MoveJoint` frame is just a `CommandType` value and a payload byte buffer to this phase. Mapping commands to actual robot behavior is `robotctl`/Phase 7's job (or a future dedicated command-dispatch phase).
- **Multi-client serving.** `TcpListener::accept()` returns one connection at a time; a real server accepting many concurrent clients (a thread pool, an event loop, wiring accepted connections into a `ControlLoop`) is future work, not this phase's.
- **Non-blocking/async I/O** (`epoll`, `io_uring`, coroutines). This phase uses simple blocking sockets — consistent with the project's "soft real-time, pragmatic" scope elsewhere (e.g. Phase 3's control loop also isn't hard-real-time). Section 3.2 mentions `epoll` as an available core dependency, but nothing in this phase's acceptance criteria requires it.
- **The authentication/integrity layer** described in section 3.7's extensibility note (`FLAG_AUTHENTICATED`, HMAC/MAC verification) — explicitly future work (Phase 14) per that same note. This phase's `FLAGS` byte exists in the wire format and round-trips correctly, but nothing reads or acts on any bit in it yet.
- **Windows/macOS support.** POSIX sockets only (`<sys/socket.h>`, `<netinet/in.h>`, `<unistd.h>`), matching this project's Linux/Ubuntu-only toolchain (section 3.1).
- **Multiple protocol versions.** `FrameCodec` decodes exactly `kProtocolVersion` (`1`) and rejects anything else with `ProtocolError::UnsupportedVersion` — the `VERSION` field exists in the wire format precisely so a *future* phase can add real backward-compatibility logic without changing the frame layout, but that logic doesn't exist yet.

## 4. Inputs

- `docs/architecture.md` section 3.7 (this phase's primary spec) and section 3.2 (confirms POSIX sockets are an approved core dependency).
- `CONTRIBUTING.md` — coding standards and the mandatory file header.
- No file from Phases 1–5 is required — this phase shares no code with them (see Non-Goals).

## 5. Deliverables

```
include/robot/protocol/command_type.hpp
include/robot/protocol/protocol_error.hpp
include/robot/protocol/frame.hpp
include/robot/protocol/frame_codec.hpp
include/robot/protocol/transport_error.hpp
include/robot/protocol/tcp_connection.hpp
include/robot/protocol/tcp_listener.hpp

src/protocol/frame_codec.cpp
src/protocol/tcp_connection.cpp
src/protocol/tcp_listener.cpp

tests/protocol/frame_codec_test.cpp
tests/protocol/tcp_test.cpp

CMakeLists.txt   (repository root — extended, not replaced)
```

## 6. Interfaces / Contracts

### 6.1 Wire format (authoritative)

```
┌─────────────┬─────────┬────────┬──────────┬─────────────┬─────────┐
│ MAGIC (u32) │VERSION  │ FLAGS  │ TYPE(u16)│ LENGTH (u32)│ PAYLOAD │
│             │  (u8)   │  (u8)  │          │             │(LENGTH  │
│             │         │        │          │             │ bytes)  │
└─────────────┴─────────┴────────┴──────────┴─────────────┴─────────┘
```

All multi-byte integer fields are **big-endian** (network byte order) — the conventional choice for wire protocols, and independent of host CPU endianness (this project's `FrameCodec` implements this with explicit shift/mask logic, not `htons`/`htonl`, so it has zero OS-header dependency, unlike the transport layer below). Header is exactly 12 bytes (`4+1+1+2+4`). `MAGIC` is a fixed constant, `kProtocolMagic = 0x524F424F` (ASCII `"ROBO"`); a frame with any other value is rejected, not silently accepted.

### 6.2 `CommandType`

```cpp
namespace robot::protocol {

enum class CommandType : std::uint16_t {
    Connect       = 1,   // 0 is deliberately unused — an all-zero header is
    GetStatus     = 2,   // never a valid CommandType, which helps distinguish
    Enable        = 3,   // a garbled/truncated frame from a real one.
    Disable       = 4,
    Home          = 5,
    MoveJoint     = 6,
    MoveLinear    = 7,
    Stop          = 8,
    EmergencyStop = 9,
    GetPosition   = 10,
    GetIO         = 11,
    SetIO         = 12,
    ResetFault    = 13,
    InjectFault   = 14,
};

[[nodiscard]] bool isKnownCommandType(std::uint16_t raw) noexcept;
[[nodiscard]] std::string_view toString(CommandType type) noexcept;

}
```

### 6.3 `Frame` / `FrameCodec` — pure, no I/O

```cpp
namespace robot::protocol {

inline constexpr std::uint32_t kProtocolMagic = 0x524F424F;
inline constexpr std::uint8_t kProtocolVersion = 1;

enum class ProtocolError {
    Incomplete,          // Not enough bytes yet to decode a full frame — not
                          // a malformed-frame error; the caller (typically a
                          // TCP receive loop) should buffer more and retry.
    InvalidMagic,
    UnsupportedVersion,
    UnknownCommandType,
    PayloadTooLarge,      // LENGTH exceeds FrameCodec::maxPayloadSize.
};

struct Frame {
    std::uint8_t flags = 0;
    CommandType type;
    std::vector<std::byte> payload;
};

class FrameCodec {
public:
    static constexpr std::size_t headerSize = 12;
    static constexpr std::uint32_t maxPayloadSize = 1'048'576;  // 1 MiB safety cap

    // Always uses kProtocolMagic/kProtocolVersion — a caller cannot encode
    // a frame with a different magic/version (there is currently only one
    // version to encode; see Non-Goals on multi-version support).
    [[nodiscard]] static std::vector<std::byte> encode(const Frame& frame);

    struct DecodeResult {
        Frame frame;
        std::size_t bytesConsumed;  // Exactly headerSize + payload.size().
    };

    // Decodes exactly one frame from the front of bytes. Trailing bytes
    // beyond the decoded frame are allowed and ignored (bytesConsumed
    // tells the caller where the next frame, if any, starts) — this is
    // what makes decode() usable directly against a streaming socket
    // receive buffer that may contain a partial next frame.
    [[nodiscard]] static std::expected<DecodeResult, ProtocolError> decode(
        std::span<const std::byte> bytes);
};

}
```

### 6.4 `TcpListener` / `TcpConnection` — real POSIX sockets

```cpp
namespace robot::protocol {

enum class TransportError {
    BindFailed, ListenFailed, AcceptFailed, ConnectFailed,
    SendFailed, ReceiveFailed, ConnectionClosed,
};

class TcpConnection {
public:
    ~TcpConnection();  // Closes the socket if still open.
    TcpConnection(const TcpConnection&) = delete;
    TcpConnection& operator=(const TcpConnection&) = delete;
    TcpConnection(TcpConnection&&) noexcept;
    TcpConnection& operator=(TcpConnection&&) noexcept;

    [[nodiscard]] static std::expected<TcpConnection, TransportError> connectTo(
        const std::string& host, std::uint16_t port);

    // Blocks until all of bytes is sent or an error occurs.
    [[nodiscard]] std::expected<void, TransportError> send(std::span<const std::byte> bytes);

    // Blocks until at least one byte is available, the peer closes
    // (-> TransportError::ConnectionClosed), or an error occurs. Returns
    // the number of bytes actually read into buffer (<= buffer.size()) —
    // the caller may need to call receive() again to fill a full frame.
    [[nodiscard]] std::expected<std::size_t, TransportError> receive(std::span<std::byte> buffer);

private:
    explicit TcpConnection(int fd) noexcept;
    int fd_ = -1;
    friend class TcpListener;
};

class TcpListener {
public:
    ~TcpListener();
    TcpListener(const TcpListener&) = delete;
    TcpListener& operator=(const TcpListener&) = delete;
    TcpListener(TcpListener&&) noexcept;
    TcpListener& operator=(TcpListener&&) noexcept;

    // port == 0 asks the OS for an ephemeral free port — the actual bound
    // port is then read back via port(), which is what lets tests avoid
    // hardcoding (and colliding on) a fixed port number.
    [[nodiscard]] static std::expected<TcpListener, TransportError> create(std::uint16_t port);

    [[nodiscard]] std::uint16_t port() const noexcept;

    // Blocks until one client connects.
    [[nodiscard]] std::expected<TcpConnection, TransportError> accept();

private:
    explicit TcpListener(int fd, std::uint16_t port) noexcept;
    int fd_ = -1;
    std::uint16_t port_;
};

}
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] `robot_protocol` compiles as a static library with no dependency on any other `robot_*` target.
- [ ] `FrameCodec::encode()` followed by `FrameCodec::decode()` round-trips a `Frame` exactly, for every `CommandType` and for both an empty and a non-empty payload.
- [ ] `decode()` on fewer than `headerSize` bytes, and on a complete header but an incomplete payload, both return `ProtocolError::Incomplete` — verified by test, including that decode() does not read past the end of the provided span in either case (no out-of-bounds access — this is exactly the kind of bug AddressSanitizer, already enabled in this project's Debug build, exists to catch).
- [ ] `decode()` rejects a wrong `MAGIC` (`InvalidMagic`), a wrong `VERSION` (`UnsupportedVersion`), an unrecognized `TYPE` (`UnknownCommandType`), and a `LENGTH` exceeding `maxPayloadSize` (`PayloadTooLarge`) — each verified by test, using a hand-built byte buffer (not `encode()`, since `encode()` can't itself produce these malformed cases).
- [ ] `decode()` correctly reports `bytesConsumed` and correctly ignores/leaves untouched any trailing bytes after one full frame — verified by decoding two concatenated frames back-to-back from one buffer.
- [ ] `TcpListener::create(0)` binds to an OS-assigned port, and `port()` returns that real, nonzero port — verified by test.
- [ ] A full connect/accept/send/receive round trip over real loopback TCP is verified by test: bind a listener on port 0, `accept()` on a background thread, `connectTo("127.0.0.1", listener.port())` from the test thread, send an encoded `Frame`, receive and decode it on the accepting side, assert equality with the original. No `sleep`/timing dependency — `accept()`/`receive()` block correctly, so there is nothing to race (unlike Phase 3's paced background loop, this test has no inherent timing flakiness to guard against).
- [ ] Closing (destroying) one end of a `TcpConnection` causes the other end's next `receive()` to return `TransportError::ConnectionClosed` — verified by test.
- [ ] `receive()` into a buffer smaller than the sender's message is verified to return a short read (fewer bytes than were sent), confirming callers cannot assume `receive()` always fills a whole frame in one call.
- [ ] No dynamic heap allocation inside `TcpConnection::send()`/`receive()` themselves (the caller-provided buffer is used as-is) — `FrameCodec::encode()`/`decode()` allocate for the returned/decoded payload buffer, which is expected and not part of this constraint (matches the same "one-time, not per-cycle" allocation category `DigitalIO`/`AnalogIO` used in Phase 5).
- [ ] All builds pass with `-Wall -Wextra -Wpedantic -Werror`, in both `Debug` (sanitizers — including the real-socket tests) and `Release`.
- [ ] `CMakeLists.txt` builds `robot_protocol` and its tests successfully alongside the existing targets — Phases 1–5's tests must keep passing unmodified.
- [ ] GoogleTest suites (`robot_protocol_tests`) cover all criteria above.
- [ ] Every deliverable file carries the standard header from `CONTRIBUTING.md`.
- [ ] Doxygen comments on every public type and method, consistent with the rest of the codebase.

## 8. Notes for the Implementer

- Write `FrameCodec`'s byte-order helpers (`writeU16BE`/`writeU32BE`/`readU16BE`/`readU32BE`, or similar) as small local functions in `frame_codec.cpp`'s anonymous namespace — explicit shifts and masks (`static_cast<std::byte>((value >> 8) & 0xFF)`, etc.), not `htons`/`htonl`/`ntohs`/`ntohl`. This keeps `FrameCodec` free of any OS networking header, which matters because it's meant to be testable and usable independent of whether a real socket is involved at all.
- `decode()`'s `Incomplete` case must be checked *before* trying to read fields that aren't fully present yet — check `bytes.size() >= headerSize` first, then (after parsing `LENGTH`) check `bytes.size() >= headerSize + length` before touching payload bytes. Getting this order backwards is exactly the kind of off-by-one that reads past the buffer; lean on the sanitizers here.
- For the TCP tests, prefer a `std::jthread` running `accept()` (same tool Phase 3 used for its background loop) over anything more elaborate — join it (implicitly, via `jthread`'s destructor, or explicitly) before the test function returns.
- `TcpConnection`/`TcpListener` need move constructors/assignment (unlike every value type introduced in Phases 1–5, which were either non-movable-by-default-being-fine or explicitly non-copyable-non-movable like `ControlLoop`) because `accept()` and `connectTo()` both need to *return* a `TcpConnection` by value — implement move by transferring the file descriptor and setting the moved-from instance's `fd_` to `-1` (so its destructor's `close()` becomes a no-op), the standard idiom for an RAII handle around an OS resource.
- Don't reach for `SO_REUSEADDR` speculatively "just in case" — it's a reasonable thing to set on a real long-lived server's listening socket, but isn't needed for this phase's ephemeral-port (`port 0`) test usage, and adding options with no test exercising them is exactly the kind of unverified code this project's testing discipline (section 3.15) argues against. Add it in a later phase if/when something concrete needs it (e.g. a `robotctl`/server phase binding a fixed, well-known port).
