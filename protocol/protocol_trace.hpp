#pragma once

#include <cstddef>
#include <cstdint>

#include "command_executor.hpp"

namespace ray2::protocol {

constexpr std::size_t kProtocolTraceCapacity = 24;
constexpr std::size_t kProtocolTracePayloadCapacity = 80;

struct ProtocolTraceEntry {
    std::uint64_t sequence = 0;
    command::Error result = command::Error::Ok;
    bool truncated = false;
    char payload[kProtocolTracePayloadCapacity] = {};
};

// Stores the final outcome of a normal UART command without writing logs.
void protocol_trace_record(const char* line, std::size_t length, command::Error result);

// Copies entries in oldest-to-newest order and returns the copied count.
std::size_t protocol_trace_snapshot(ProtocolTraceEntry* entries, std::size_t capacity);

}  // namespace ray2::protocol
