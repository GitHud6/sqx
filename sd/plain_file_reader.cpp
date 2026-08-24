#include "plain_file_reader.hpp"

#include <cstdio>
#include <cstring>

#include "sdcard_driver.hpp"

namespace ray2::sd {
namespace {

constexpr char kMountPointPrefix[] = "/sd";

bool valid_path(const char* path) {
    return path != nullptr && path[0] == '/' && std::strstr(path, "..") == nullptr;
}

}  // namespace

bool PlainFileReader::open(const char* path) {
    close();
    if (!ray2::driver::sdcard_is_mounted() || !valid_path(path)) {
        error_ = true;
        return false;
    }

    char full_path[kFilePathCapacity + sizeof(kMountPointPrefix)] = {};
    const int length = std::snprintf(full_path, sizeof(full_path), "%s%s", kMountPointPrefix, path);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(full_path)) {
        error_ = true;
        return false;
    }

    file_ = std::fopen(full_path, "rb");
    if (file_ == nullptr) {
        error_ = true;
        return false;
    }
    eof_ = false;
    error_ = false;
    return true;
}

bool PlainFileReader::read_line(char* buffer, std::size_t capacity) {
    if (buffer == nullptr || capacity == 0 || file_ == nullptr || error_ || eof_) {
        return false;
    }

    auto* file = static_cast<std::FILE*>(file_);
    std::size_t length = 0;
    while (true) {
        const int value = std::fgetc(file);
        if (value == EOF) {
            if (std::feof(file) != 0) {
                eof_ = true;
                if (length == 0) {
                    buffer[0] = '\0';
                    return false;
                }
                buffer[length] = '\0';
                return true;
            }
            error_ = true;
            buffer[0] = '\0';
            return false;
        }
        if (value == '\n') {
            buffer[length] = '\0';
            return true;
        }
        if (value == '\r') {
            continue;
        }
        if (length + 1 >= capacity) {
            do {
                const int discarded = std::fgetc(file);
                if (discarded == '\n' || discarded == EOF) {
                    if (discarded == EOF && std::feof(file) != 0) {
                        eof_ = true;
                    }
                    break;
                }
            } while (true);
            buffer[0] = '\0';
            error_ = true;
            return false;
        }
        buffer[length++] = static_cast<char>(value);
    }
}

bool PlainFileReader::eof() const {
    return eof_;
}

bool PlainFileReader::has_error() const {
    return error_;
}

void PlainFileReader::close() {
    if (file_ != nullptr) {
        std::fclose(static_cast<std::FILE*>(file_));
        file_ = nullptr;
    }
    eof_ = false;
    error_ = false;
}

}  // namespace ray2::sd
