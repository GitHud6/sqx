#pragma once

#include <cstdint>

namespace ray2::protocol {

enum class RealtimeCommand : std::uint8_t {
    StatusReport,
    FeedHold,
    CycleStart,
    Reset,
};

struct RealtimeCommandDiagnostics {
    std::uint64_t status_report_count = 0;
    std::uint64_t feed_hold_count = 0;
    std::uint64_t cycle_start_count = 0;
    std::uint64_t reset_count = 0;
};

// Returns true only for a recognized single-byte Grbl realtime command.
bool realtime_command_from_byte(std::uint8_t byte, RealtimeCommand& command);

// Stage one records a received realtime command without applying control behavior.
void realtime_command_receive(RealtimeCommand command);
RealtimeCommandDiagnostics realtime_command_diagnostics();

}  // namespace ray2::protocol
