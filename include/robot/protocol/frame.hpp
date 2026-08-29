// include/robot/protocol/frame.hpp
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
#include <vector>
#include "robot/protocol/command_type.hpp"

namespace robot::protocol {

/// @brief The in-memory representation of one protocol frame.
///
/// MAGIC and VERSION are not represented here — they are wire-only
/// concerns FrameCodec always writes/validates as its fixed constants
/// (kProtocolMagic/kProtocolVersion); a caller cannot construct a Frame
/// with a different magic or version. LENGTH is likewise not stored
/// explicitly — it's always payload.size() on encode.
struct Frame {
    std::uint8_t flags = 0;
    CommandType type;
    std::vector<std::byte> payload;
};

}  // namespace robot::protocol
