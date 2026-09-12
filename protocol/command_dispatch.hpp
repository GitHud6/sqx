#pragma once

#include <cstddef>

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

// Emits one standard Grbl-style realtime status line without an `ok` response.
bool command_report_realtime_status();

// Executes a G-code line with the protocol-owned ParserState without emitting a UART response.
command::Error command_dispatch_external(const char* line, std::size_t length);

// Synchronizes the protocol-owned ParserState after a system motion establishes
// a known machine coordinate. The protocol owns the mutex protecting this state.
command::Error command_sync_parser_position_from_machine();

// Resets modal/parser state after a realtime hard reset. The position is
// deliberately unavailable until a later homing operation establishes it.
void command_reset_after_hard_reset();

}  // namespace ray2::protocol
