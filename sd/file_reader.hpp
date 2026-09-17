#pragma once

#include <cstddef>
#include <cstdint>

namespace ray2::sd {

constexpr std::size_t kFilePathCapacity = 128;
constexpr std::size_t kGcodeLineCapacity = 256;

class FileReader {
public:
    virtual ~FileReader() = default;

    virtual bool open(const char* path) = 0;
    virtual bool read_line(char* buffer, std::size_t capacity) = 0;
    virtual bool eof() const = 0;
    virtual bool has_error() const = 0;
    // These are input-stream units: file bytes for plain G-code and
    // uncompressed bytes for gzip G-code.
    virtual std::uint64_t bytes_consumed() const = 0;
    virtual std::uint64_t total_bytes() const = 0;
    virtual void close() = 0;
};

}  // namespace ray2::sd
