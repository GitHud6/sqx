#include "realtime_command.hpp"

#include <atomic>

namespace ray2::protocol {
namespace {

std::atomic<std::uint64_t> s_status_report_count = 0;

}  // namespace

bool realtime_command_from_byte(const std::uint8_t byte, RealtimeCommand& command) {
    switch (byte) {
        case '?':
            command = RealtimeCommand::StatusReport;
            return true;
        default:
            return false;
    }
}

void realtime_command_receive(const RealtimeCommand command) {
    switch (command) {
        case RealtimeCommand::StatusReport:
            s_status_report_count.fetch_add(1, std::memory_order_relaxed);
            return;
    }
}

RealtimeCommandDiagnostics realtime_command_diagnostics() {
    return {
        .status_report_count = s_status_report_count.load(std::memory_order_relaxed),
    };
}

}  // namespace ray2::protocol
