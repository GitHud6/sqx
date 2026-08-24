#pragma once

#include <cstddef>

#include "command_executor.hpp"

namespace ray2::protocol {

bool command_dispatch(const char* line, std::size_t length);

// Executes a G-code line with the protocol-owned ParserState without emitting a UART response.
command::Error command_dispatch_external(const char* line, std::size_t length);

}  // namespace ray2::protocol
