#pragma once

#include <cstddef>

#include "file_reader.hpp"

namespace ray2::sd {

enum class SystemCommandType {
    Status,
    List,
    Run,
};

struct SystemCommand {
    SystemCommandType type = SystemCommandType::Status;
    char path[kFilePathCapacity] = {};
};

bool is_system_command(const char* line, std::size_t length);

// Parses the SD system-command namespace without executing a job.
bool parse_system_command(const char* line, std::size_t length, SystemCommand& command);

}  // namespace ray2::sd
