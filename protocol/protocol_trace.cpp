#include "protocol_trace.hpp"

#include <algorithm>
#include <cstring>

#include "freertos/FreeRTOS.h"

namespace ray2::protocol {
namespace {

portMUX_TYPE s_trace_lock = portMUX_INITIALIZER_UNLOCKED;
ProtocolTraceEntry s_entries[kProtocolTraceCapacity] = {};
std::size_t s_head = 0;
std::size_t s_count = 0;
std::uint64_t s_next_sequence = 1;

}  // namespace

void protocol_trace_record(const char* const line,
                           const std::size_t length,
                           const command::Error result) {
    if (line == nullptr) {
        return;
    }

    portENTER_CRITICAL(&s_trace_lock);
    ProtocolTraceEntry& entry = s_entries[s_head];
    entry = {};
    entry.sequence = s_next_sequence++;
    entry.result = result;
    const std::size_t copied_length = std::min(length, kProtocolTracePayloadCapacity - 1);
    std::memcpy(entry.payload, line, copied_length);
    entry.payload[copied_length] = '\0';
    entry.truncated = copied_length != length;
    s_head = (s_head + 1) % kProtocolTraceCapacity;
    s_count = std::min(s_count + 1, kProtocolTraceCapacity);
    portEXIT_CRITICAL(&s_trace_lock);
}

std::size_t protocol_trace_snapshot(ProtocolTraceEntry* const entries, const std::size_t capacity) {
    if (entries == nullptr || capacity == 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_trace_lock);
    const std::size_t copied_count = std::min(s_count, capacity);
    const std::size_t start = (s_head + kProtocolTraceCapacity - copied_count) % kProtocolTraceCapacity;
    for (std::size_t index = 0; index < copied_count; ++index) {
        entries[index] = s_entries[(start + index) % kProtocolTraceCapacity];
    }
    portEXIT_CRITICAL(&s_trace_lock);
    return copied_count;
}

}  // namespace ray2::protocol
