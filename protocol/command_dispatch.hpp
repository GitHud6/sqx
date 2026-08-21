#pragma once

#include <cstddef>

namespace ray2::protocol {

bool command_dispatch(const char* line, std::size_t length);

}  // namespace ray2::protocol
