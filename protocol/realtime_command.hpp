#pragma once

#include <cstdint>

namespace ray2::protocol {

enum class RealtimeCommand : std::uint8_t {
    StatusReport,
    HardReset,
    FeedHold,
    CycleStart,
};

// Returns true only for a recognized single-byte Grbl realtime command.
bool realtime_command_from_byte(std::uint8_t byte, RealtimeCommand& command);

// Records a received realtime command. Protocol owns reporting and future control behavior.
void realtime_command_receive(RealtimeCommand command);

}  // namespace ray2::protocol
