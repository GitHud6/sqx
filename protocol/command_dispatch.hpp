#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "command_executor.hpp"

namespace ray2::protocol {

// Initializes protocol-owned state required by both the normal command task
// and the UART realtime ingress path.
esp_err_t command_dispatch_start();

// Returns false only when the UART response could not be written. When
// capacity_blocked is set, the caller must retain and retry the same line
// after planner capacity becomes available; no response was emitted.
bool command_dispatch(const char* line, std::size_t length, bool& capacity_blocked);

// Sends the normal command response to a caller-owned sink instead of UART.
// The command still executes through the protocol-owned parser and service
// path, so alternate transports cannot bypass motion or safety semantics.
using CommandResponseWriter = bool (*)(const std::uint8_t* data, std::size_t length,
                                       void* user_data);
bool command_dispatch_with_response(const char* line, std::size_t length,
                                    bool& capacity_blocked, CommandResponseWriter response_writer,
                                    void* response_user_data);

// Emits one standard Grbl-style realtime status line without an `ok` response.
bool command_report_realtime_status();

// Formats the standard realtime status payload for an alternate transport.
bool command_format_realtime_status(char* response, std::size_t capacity,
                                    std::size_t& response_length);

// Formats the compatibility build-information response, including its final
// `ok` marker, for alternate transports.
bool command_format_build_info(char* response, std::size_t capacity,
                               std::size_t& response_length);

// Executes a G-code line with the protocol-owned ParserState without emitting a UART response.
command::Error command_dispatch_external(const char* line, std::size_t length);

// Synchronizes the protocol-owned ParserState after a system motion establishes
// a known machine coordinate. The protocol owns the mutex protecting this state.
command::Error command_sync_parser_position_from_machine();

// Resets modal/parser state after a realtime hard reset. The position is
// deliberately unavailable until a later homing operation establishes it.
void command_reset_after_hard_reset();

}  // namespace ray2::protocol
