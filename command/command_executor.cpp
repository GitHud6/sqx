#include "command_executor.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>

#include "motion_submit.hpp"
#include "planner.hpp"

namespace ray2::command {

namespace {

constexpr double kMillimetersPerInch = 25.4;
constexpr double kMaxFeedRateMmPerMin = 12000.0;
constexpr double kMaxSpindlePower = 1000.0;

int axis_index(char letter) {
    return letter == 'X' ? 0 : letter == 'Y' ? 1 : letter == 'Z' ? 2 : -1;
}

Error parse_g_word(double value, ParsedBlock& parsed) {
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
        case 17:
            if (parsed.has_plane) return Error::ModalGroupViolation;
            parsed.has_plane = true;
            parsed.plane = PlaneMode::XY;
            return Error::Ok;
        case 40:
            if (parsed.has_cutter_compensation) return Error::ModalGroupViolation;
            parsed.has_cutter_compensation = true;
            parsed.cutter_compensation = CutterCompensation::Disabled;
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

Error parse_m_word(double value, ParsedBlock& parsed) {
    if (std::floor(value) != value) {
        return Error::UnsupportedCommand;
    }
    if (parsed.has_spindle_mode) {
        return Error::ModalGroupViolation;
    }

    switch (static_cast<int>(value)) {
        case 3:
            parsed.spindle_mode = SpindleMode::Clockwise;
            break;
        case 4:
            parsed.spindle_mode = SpindleMode::CounterClockwise;
            break;
        case 5:
            parsed.spindle_mode = SpindleMode::Off;
            break;
        default:
            return Error::UnsupportedCommand;
    }
    parsed.has_spindle_mode = true;
    return Error::Ok;
}

Error parse_line(const char* line, std::size_t length, ParsedBlock& parsed) {
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
        if (letter == 'M') {
            const Error result = parse_m_word(value, parsed);
            if (result != Error::Ok) return result;
            continue;
        }
        const int axis = axis_index(letter);
        if (axis >= 0) {
            if (parsed.has_axis[axis]) return Error::WordRepeated;
            if (axis == 2) return Error::UnsupportedCommand;
            parsed.has_axis[axis] = true;
            parsed.axis_value[axis] = value;
            continue;
        }
        if (letter == 'F') {
            if (parsed.has_feed_rate) return Error::WordRepeated;
            if (value < 0.0) return Error::NegativeValue;
            parsed.has_feed_rate = true;
            parsed.feed_rate = value;
            continue;
        }
        if (letter == 'S') {
            if (parsed.has_spindle_speed) return Error::WordRepeated;
            if (value < 0.0) return Error::NegativeValue;
            parsed.has_spindle_speed = true;
            parsed.spindle_speed = value;
            continue;
        }
        return Error::UnsupportedCommand;
    }
    return in_comment ? Error::BadNumberFormat : Error::Ok;
}

motion::MotionType motion_type(MotionMode mode) {
    return mode == MotionMode::Linear ? motion::MotionType::Linear : motion::MotionType::Rapid;
}

motion::SpindleMode spindle_mode(SpindleMode mode) {
    switch (mode) {
        case SpindleMode::Off: return motion::SpindleMode::Off;
        case SpindleMode::Clockwise: return motion::SpindleMode::Clockwise;
        case SpindleMode::CounterClockwise: return motion::SpindleMode::CounterClockwise;
    }
    return motion::SpindleMode::Off;
}

Error submit_motion_request(motion::MotionRequest& request) {
    request.has_motion = true;
    const motion::MotionSubmitResult result = motion::motion_submit(request);
    switch (result) {
        case motion::MotionSubmitResult::Accepted:
            return Error::Ok;
        case motion::MotionSubmitResult::Busy:
            return Error::MotionBusy;
        case motion::MotionSubmitResult::Rejected:
            return Error::MotionRejected;
    }
    return Error::MotionRejected;
}

Error submit_spindle_disable() {
    switch (planner::planner_submit_spindle_disable()) {
        case planner::SubmitResult::Accepted:
            return Error::Ok;
        case planner::SubmitResult::Full:
            return Error::MotionBusy;
        case planner::SubmitResult::Rejected:
            return Error::MotionRejected;
    }
    return Error::MotionRejected;
}

void commit_parser_state(ParserState& state,
                         const ParserState& next_state,
                         const motion::MotionRequest& request,
                         bool has_motion) {
    ParserState committed_state = next_state;
    if (has_motion) {
        for (int axis = 0; axis < 3; ++axis) {
            committed_state.position_mm[axis] = request.target_mm[axis];
        }
    }
    state = committed_state;
}

}  // namespace

Error execute_line(const char* line, std::size_t length, ParserState& state) {
    if (length == 0) return Error::Ok;
    if (line[0] == '$' || line[0] == '[') return Error::SystemCommandUnsupported;
    if (state.system_state == SystemState::Alarm || state.system_state == SystemState::Jog) return Error::SystemGcLock;

    ParsedBlock parsed = {};
    const Error result = parse_line(line, length, parsed);
    if (result != Error::Ok) return result;

    ParserState next_state = state;
    if (parsed.has_units) next_state.units = parsed.units;
    if (parsed.has_distance_mode) next_state.distance_mode = parsed.distance_mode;
    if (parsed.has_motion) next_state.motion_mode = parsed.motion_mode;
    if (parsed.has_plane) next_state.plane = parsed.plane;
    if (parsed.has_cutter_compensation) {
        next_state.cutter_compensation = parsed.cutter_compensation;
    }
    if (parsed.has_spindle_mode) next_state.spindle_mode = parsed.spindle_mode;

    const double unit_scale = next_state.units == Units::Inches ? kMillimetersPerInch : 1.0;
    if (parsed.has_feed_rate) {
        next_state.feed_rate_mm_per_min = parsed.feed_rate * unit_scale;
        if (next_state.feed_rate_mm_per_min > kMaxFeedRateMmPerMin) return Error::ValueOutOfRange;
    }
    if (parsed.has_spindle_speed) next_state.spindle_speed = parsed.spindle_speed;
    if (next_state.spindle_speed > kMaxSpindlePower) return Error::ValueOutOfRange;

    bool has_axis = false;
    motion::MotionRequest request = {};
    request.motion_mode = motion_type(next_state.motion_mode);
    request.feed_rate_mm_per_min = next_state.feed_rate_mm_per_min;
    request.spindle_power = next_state.spindle_speed;
    request.spindle_mode = spindle_mode(next_state.spindle_mode);
    for (int axis = 0; axis < 3; ++axis) {
        request.start_mm[axis] = state.position_mm[axis];
        request.target_mm[axis] = state.position_mm[axis];
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (!parsed.has_axis[axis]) continue;
        has_axis = true;
        const double value_mm = parsed.axis_value[axis] * unit_scale;
        request.target_mm[axis] = next_state.distance_mode == DistanceMode::Absolute
                                      ? value_mm
                                      : state.position_mm[axis] + value_mm;
    }

    // A standalone rapid-mode word is a modal update in Grbl. It must not
    // create a zero-length planner block when a file header sends G00 without
    // axis words. Linear G1 remains strict until its file behavior is known.
    if (parsed.has_motion && !has_axis && parsed.motion_mode != MotionMode::Rapid) {
        return Error::NoAxisWords;
    }
    if (has_axis) {
        const Error submit_error = submit_motion_request(request);
        if (submit_error != Error::Ok) return submit_error;
    } else if (parsed.has_spindle_mode && parsed.spindle_mode == SpindleMode::Off) {
        const Error submit_error = submit_spindle_disable();
        if (submit_error != Error::Ok) return submit_error;
    }

    commit_parser_state(state, next_state, request, has_axis);
    return Error::Ok;
}

const char* error_text(Error error) {
    switch (error) {
        case Error::Ok: return "ok";
        case Error::SystemCommandUnsupported: return "system-command-unsupported";
        case Error::SystemGcLock: return "system-gc-lock";
        case Error::ExpectedCommandLetter: return "expected-command-letter";
        case Error::BadNumberFormat: return "bad-number-format";
        case Error::NegativeValue: return "negative-value";
        case Error::ValueOutOfRange: return "value-out-of-range";
        case Error::UnsupportedCommand: return "unsupported-command";
        case Error::ModalGroupViolation: return "modal-group-violation";
        case Error::WordRepeated: return "word-repeated";
        case Error::NoAxisWords: return "no-axis-words";
        case Error::MotionBusy: return "motion-busy";
        case Error::MotionRejected: return "motion-rejected";
    }
    return "unknown";
}

}  // namespace ray2::command
