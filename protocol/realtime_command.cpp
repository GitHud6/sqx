#include "realtime_command.hpp"

#include <atomic>

namespace ray2::protocol {
namespace {

std::atomic<std::uint64_t> s_status_report_count = 0;
std::atomic<std::uint64_t> s_feed_hold_count = 0;
std::atomic<std::uint64_t> s_cycle_start_count = 0;
std::atomic<std::uint64_t> s_reset_count = 0;

}  // namespace

bool realtime_command_from_byte(const std::uint8_t byte, RealtimeCommand& command) {
    switch (byte) {
        case '?':
            command = RealtimeCommand::StatusReport;
            return true;
        case '!':
            command = RealtimeCommand::FeedHold;
            return true;
        case '~':
            command = RealtimeCommand::CycleStart;
            return true;
        case 0x18:
            command = RealtimeCommand::Reset;
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
        case RealtimeCommand::FeedHold:
            s_feed_hold_count.fetch_add(1, std::memory_order_relaxed);
            return;
        case RealtimeCommand::CycleStart:
            s_cycle_start_count.fetch_add(1, std::memory_order_relaxed);
            return;
        case RealtimeCommand::Reset:
            s_reset_count.fetch_add(1, std::memory_order_relaxed);
            return;
    }
}

RealtimeCommandDiagnostics realtime_command_diagnostics() {
    return {
        .status_report_count = s_status_report_count.load(std::memory_order_relaxed),
        .feed_hold_count = s_feed_hold_count.load(std::memory_order_relaxed),
        .cycle_start_count = s_cycle_start_count.load(std::memory_order_relaxed),
        .reset_count = s_reset_count.load(std::memory_order_relaxed),
    };
}

}  // namespace ray2::protocol
