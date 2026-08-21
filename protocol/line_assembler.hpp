#pragma once

#include <cstddef>
#include <cstdint>

namespace ray2::protocol {

constexpr std::size_t kLineBufferSize = 256;

enum class LineEvent {
    None,
    Ready,
    Overflow,
};

class LineAssembler {
public:
    LineEvent push(std::uint8_t byte);

    const char* line() const;
    std::size_t length() const;
    void reset_line();

private:
    LineEvent finish_line();

    char buffer_[kLineBufferSize] = {};
    std::size_t length_ = 0;
    bool ignore_next_lf_ = false;
    bool discard_until_eol_ = false;
};

}  // namespace ray2::protocol
