#include "command_executor.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace ray2::command {

namespace {

constexpr double kMillimetersPerInch = 25.4;

struct ParsedLine {
    bool has_motion = false;
    MotionMode motion_mode = MotionMode::Rapid;
    bool has_distance_mode = false;
    DistanceMode distance_mode = DistanceMode::Absolute;
    bool has_units = false;
    Units units = Units::Millimeters;
    bool has_axis[3] = {};
    double axis_value[3] = {};
    bool has_feed_rate = false;
    double feed_rate = 0.0;
    bool has_spindle_speed = false;
    double spindle_speed = 0.0;
};

int axis_index(char letter) {
    return letter == 'X' ? 0 : letter == 'Y' ? 1 : letter == 'Z' ? 2 : -1;
}

Error parse_g_word(double value, ParsedLine& parsed) {
    if (std::floor(value) != value) {
        return Error::UnsupportedCommand;
    }
    switch (static_cast<int>(value)) {
        case 0:
        case 1:
            if (parsed.has_motion) return Error::ModalGroupViolation;
            parsed.has_motion = true;
            parsed.motion_mode = value == 0.0 ? MotionMode::Rapid : MotionMode::Linear;
            return Error::Ok;
        case 20:
        case 21:
            if (parsed.has_units) return Error::ModalGroupViolation;
            parsed.has_units = true;
            parsed.units = value == 20.0 ? Units::Inches : Units::Millimeters;
            return Error::Ok;
        case 90:
        case 91:
            if (parsed.has_distance_mode) return Error::ModalGroupViolation;
            parsed.has_distance_mode = true;
            parsed.distance_mode = value == 90.0 ? DistanceMode::Absolute : DistanceMode::Incremental;
            return Error::Ok;
        default:
            return Error::UnsupportedCommand;
    }
}

Error parse_line(const char* line, std::size_t length, ParsedLine& parsed) {
    const char* cursor = line;
    const char* const end = line + length;
    bool in_comment = false;

    while (cursor < end) {
        const unsigned char raw = static_cast<unsigned char>(*cursor);
        if (in_comment) {
            in_comment = *cursor != ')';
            ++cursor;
            continue;
        }
        if (*cursor == ';') break;
        if (*cursor == '(') {
            in_comment = true;
            ++cursor;
            continue;
        }
        if (std::isspace(raw)) {
            ++cursor;
            continue;
        }
        if (!std::isalpha(raw)) return Error::ExpectedCommandLetter;

        const char letter = static_cast<char>(std::toupper(raw));
        ++cursor;
        char* number_end = nullptr;
        const double value = std::strtod(cursor, &number_end);
        if (number_end == cursor || number_end > end || !std::isfinite(value)) return Error::BadNumberFormat;
        cursor = number_end;

        if (letter == 'G') {
            const Error result = parse_g_word(value, parsed);
            if (result != Error::Ok) return result;
            continue;
        }
        const int axis = axis_index(letter);
        if (axis >= 0) {
            if (parsed.has_axis[axis]) return Error::WordRepeated;
            parsed.has_axis[axis] = true;
            parsed.axis_value[axis] = value;
            continue;
        }
        if (letter == 'F') {
            if (parsed.has_feed_rate) return Error::WordRepeated;
            parsed.has_feed_rate = true;
            parsed.feed_rate = value;
            continue;
        }
        if (letter == 'S') {
            if (parsed.has_spindle_speed) return Error::WordRepeated;
            parsed.has_spindle_speed = true;
            parsed.spindle_speed = value;
            continue;
        }
        return Error::UnsupportedCommand;
    }
    return in_comment ? Error::BadNumberFormat : Error::Ok;
}

}  // namespace

Error execute_line(const char* line, std::size_t length, ParserState& state) {
    if (length == 0) return Error::Ok;
    if (line[0] == '$' || line[0] == '[') return Error::SystemCommandUnsupported;
    if (state.system_state == SystemState::Alarm || state.system_state == SystemState::Jog) return Error::SystemGcLock;

    ParsedLine parsed = {};
    const Error result = parse_line(line, length, parsed);
    if (result != Error::Ok) return result;

    if (parsed.has_units) state.units = parsed.units;
    if (parsed.has_distance_mode) state.distance_mode = parsed.distance_mode;
    if (parsed.has_motion) state.motion_mode = parsed.motion_mode;

    const double unit_scale = state.units == Units::Inches ? kMillimetersPerInch : 1.0;
    if (parsed.has_feed_rate) state.feed_rate_mm_per_min = parsed.feed_rate * unit_scale;
    if (parsed.has_spindle_speed) state.spindle_speed = parsed.spindle_speed;

    bool has_axis = false;
    for (int axis = 0; axis < 3; ++axis) {
        if (!parsed.has_axis[axis]) continue;
        has_axis = true;
        const double value_mm = parsed.axis_value[axis] * unit_scale;
        state.position_mm[axis] = state.distance_mode == DistanceMode::Absolute
                                      ? value_mm
                                      : state.position_mm[axis] + value_mm;
    }
    return parsed.has_motion && !has_axis ? Error::NoAxisWords : Error::Ok;
}

const char* error_text(Error error) {
    switch (error) {
        case Error::Ok: return "ok";
        case Error::SystemCommandUnsupported: return "system-command-unsupported";
        case Error::SystemGcLock: return "system-gc-lock";
        case Error::ExpectedCommandLetter: return "expected-command-letter";
        case Error::BadNumberFormat: return "bad-number-format";
        case Error::UnsupportedCommand: return "unsupported-command";
        case Error::ModalGroupViolation: return "modal-group-violation";
        case Error::WordRepeated: return "word-repeated";
        case Error::NoAxisWords: return "no-axis-words";
    }
    return "unknown";
}

}  // namespace ray2::command
