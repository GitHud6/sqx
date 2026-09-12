#include "realtime_command.hpp"

namespace ray2::protocol {

bool realtime_command_from_byte(const std::uint8_t byte, RealtimeCommand& command) {
    switch (byte) {
        case '?':
            command = RealtimeCommand::StatusReport;
            return true;
        case 0x18:
            command = RealtimeCommand::HardReset;
            return true;
        default:
            return false;
    }
}

void realtime_command_receive(const RealtimeCommand command) {
    switch (command) {
        case RealtimeCommand::StatusReport:
            return;
        case RealtimeCommand::HardReset:
            return;
    }
}

}  // namespace ray2::protocol
