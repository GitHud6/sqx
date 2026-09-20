#include "sd_command_handler.hpp"

#include <cctype>
#include <cstring>

namespace laserctl::sd {
namespace {

bool equals_ignore_case(const char* left, const char* right, std::size_t length) {
    for (std::size_t index = 0; index < length; ++index) {
        const unsigned char left_char = static_cast<unsigned char>(left[index]);
        const unsigned char right_char = static_cast<unsigned char>(right[index]);
        if (std::toupper(left_char) != std::toupper(right_char)) {
            return false;
        }
    }
    return true;
}

bool valid_path(const char* path, std::size_t length) {
    if (length == 0 || length >= kFilePathCapacity) {
        return false;
    }
    if (std::strstr(path, "..") != nullptr) {
        return false;
    }
    return path[0] == '/';
}

}  // namespace

bool is_system_command(const char* line, std::size_t length) {
    return line != nullptr && length >= 4 && line[0] == '$' &&
           equals_ignore_case(line + 1, "SD/", 3);
}

bool parse_system_command(const char* line, std::size_t length, SystemCommand& command) {
    if (!is_system_command(line, length) || length < 8) {
        return false;
    }

    const char* const payload = line + 4;
    const std::size_t payload_length = length - 4;
    if (payload_length == 6 && equals_ignore_case(payload, "STATUS", payload_length)) {
        command = {};
        command.type = SystemCommandType::Status;
        return true;
    }
    if (payload_length == 4 && equals_ignore_case(payload, "LIST", payload_length)) {
        command = {};
        command.type = SystemCommandType::List;
        return true;
    }

    constexpr char kRunPrefix[] = "RUN=";
    if (payload_length <= sizeof(kRunPrefix) - 1 ||
        !equals_ignore_case(payload, kRunPrefix, sizeof(kRunPrefix) - 1)) {
        return false;
    }

    const char* const source_path = payload + sizeof(kRunPrefix) - 1;
    const std::size_t source_length = payload_length - (sizeof(kRunPrefix) - 1);
    char normalized_path[kFilePathCapacity] = {};
    if (source_length >= kFilePathCapacity) {
        return false;
    }

    const bool has_root = source_path[0] == '/';
    const std::size_t normalized_length = has_root ? source_length : source_length + 1;
    if (normalized_length >= kFilePathCapacity) {
        return false;
    }
    if (has_root) {
        std::memcpy(normalized_path, source_path, source_length);
    } else {
        normalized_path[0] = '/';
        std::memcpy(normalized_path + 1, source_path, source_length);
    }
    normalized_path[normalized_length] = '\0';

    if (!valid_path(normalized_path, normalized_length)) {
        return false;
    }

    command = {};
    command.type = SystemCommandType::Run;
    std::memcpy(command.path, normalized_path, normalized_length + 1);
    return true;
}

}  // namespace laserctl::sd
