#include "line_assembler.hpp"

namespace laserctl::protocol {

LineEvent LineAssembler::push(std::uint8_t byte) {
    if (ignore_next_lf_) {
        ignore_next_lf_ = false;
        if (byte == '\n') {
            return LineEvent::None;
        }
    }

    if (byte == '\r') {
        ignore_next_lf_ = true;
        return finish_line();
    }

    if (byte == '\n') {
        return finish_line();
    }

    if (discard_until_eol_) {
        return LineEvent::None;
    }

    if (length_ == kLineBufferSize - 1) {
        discard_until_eol_ = true;
        return LineEvent::None;
    }

    buffer_[length_++] = static_cast<char>(byte);
    return LineEvent::None;
}

const char* LineAssembler::line() const {
    return buffer_;
}

std::size_t LineAssembler::length() const {
    return length_;
}

void LineAssembler::reset_line() {
    length_ = 0;
    buffer_[0] = '\0';
}

LineEvent LineAssembler::finish_line() {
    if (discard_until_eol_) {
        discard_until_eol_ = false;
        reset_line();
        return LineEvent::Overflow;
    }

    if (length_ == 0) {
        return LineEvent::None;
    }

    buffer_[length_] = '\0';
    return LineEvent::Ready;
}

}  // namespace laserctl::protocol
