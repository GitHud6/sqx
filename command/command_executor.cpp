#include "command_executor.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "motion_submit.hpp"
#include "planner.hpp"
#include "stepper_driver.hpp"

namespace ray2::command {

namespace {

constexpr double kMillimetersPerInch = 25.4;
constexpr double kMaxFeedRateMmPerMin = 12000.0;
constexpr double kMaxSpindlePower = 1000.0;
constexpr double kArcRadiusToleranceMm = 0.01;
constexpr double kArcCoincidentPointToleranceMm = 0.000001;

int axis_index(char letter) {
    return letter == 'X' ? 0 : letter == 'Y' ? 1 : letter == 'Z' ? 2 : -1;
}

// Scan one G-code decimal number before converting it. Calling strtod() on
// the original payload would allow a token such as "0X33.3" to be treated as
// a hexadecimal floating-point literal when the line contains "G0X33.3".
const char* parse_decimal_number(const char* cursor,
                                 const char* end,
                                 double& value) {
    const char* number_begin = cursor;
    if (cursor < end && (*cursor == '+' || *cursor == '-')) ++cursor;

    bool has_integer_digits = false;
    while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
        has_integer_digits = true;
        ++cursor;
    }

    bool has_fraction_digits = false;
    if (cursor < end && *cursor == '.') {
        ++cursor;
        while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
            has_fraction_digits = true;
            ++cursor;
        }
    }
    if (!has_integer_digits && !has_fraction_digits) return nullptr;

    if (cursor < end && (*cursor == 'e' || *cursor == 'E')) {
        ++cursor;
        if (cursor < end && (*cursor == '+' || *cursor == '-')) ++cursor;
        const char* exponent_digits = cursor;
        while (cursor < end && std::isdigit(static_cast<unsigned char>(*cursor))) {
            ++cursor;
        }
        if (cursor == exponent_digits) {
            // Do not silently split malformed input such as "X1eY".
            return nullptr;
        }
    }

    constexpr std::size_t kNumberBufferSize = 64;
    const std::size_t number_length = static_cast<std::size_t>(cursor - number_begin);
    if (number_length >= kNumberBufferSize) return nullptr;

    char number_buffer[kNumberBufferSize] = {};
    std::memcpy(number_buffer, number_begin, number_length);
    char* conversion_end = nullptr;
    value = std::strtod(number_buffer, &conversion_end);
    if (conversion_end != number_buffer + number_length || !std::isfinite(value)) {
        return nullptr;
    }
    return cursor;
}

Error parse_g_word(double value, ParsedBlock& parsed) {
    if (std::floor(value) != value) {
        return Error::UnsupportedCommand;
    }
    switch (static_cast<int>(value)) {
        case 0:
        case 1:
        case 2:
        case 3:
            if (parsed.has_motion) return Error::ModalGroupViolation;
            parsed.has_motion = true;
            parsed.motion_mode = value == 0.0 ? MotionMode::Rapid
                               : value == 1.0 ? MotionMode::Linear
                               : value == 2.0 ? MotionMode::ArcClockwise
                                              : MotionMode::ArcCounterClockwise;
            return Error::Ok;
        case 4:
            if (parsed.has_dwell) return Error::ModalGroupViolation;
            parsed.has_dwell = true;
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
        case 54:
            if (parsed.has_work_coordinate_system) return Error::ModalGroupViolation;
            parsed.has_work_coordinate_system = true;
            parsed.work_coordinate_system = WorkCoordinateSystem::G54;
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

    switch (static_cast<int>(value)) {
        case 0:
        case 1:
            if (parsed.has_program_pause) return Error::ModalGroupViolation;
            parsed.has_program_pause = true;
            return Error::Ok;
        case 2:
        case 30:
            if (parsed.has_program_end) return Error::ModalGroupViolation;
            parsed.has_program_end = true;
            return Error::Ok;
        case 3:
            if (parsed.has_spindle_mode) return Error::ModalGroupViolation;
            parsed.spindle_mode = SpindleMode::Clockwise;
            break;
        case 4:
            if (parsed.has_spindle_mode) return Error::ModalGroupViolation;
            parsed.spindle_mode = SpindleMode::CounterClockwise;
            break;
        case 5:
            if (parsed.has_spindle_mode) return Error::ModalGroupViolation;
            parsed.spindle_mode = SpindleMode::Off;
            break;
        case 8:
        case 9:
            if (parsed.has_coolant_command) return Error::ModalGroupViolation;
            // ray2-old accepts M8/M9 as Grbl coolant controls. RayCube has no
            // coolant hardware, so accept either command without touching laser state.
            parsed.has_coolant_command = true;
            return Error::Ok;
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
        double value = 0.0;
        const char* number_end = parse_decimal_number(cursor, end, value);
        if (number_end == nullptr) return Error::BadNumberFormat;
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
        if (letter == 'I' || letter == 'J') {
            const int arc_axis = letter == 'I' ? 0 : 1;
            if (parsed.has_arc_offset[arc_axis]) return Error::WordRepeated;
            parsed.has_arc_offset[arc_axis] = true;
            parsed.arc_offset[arc_axis] = value;
            continue;
        }
        if (letter == 'R') {
            if (parsed.has_arc_radius) return Error::WordRepeated;
            parsed.has_arc_radius = true;
            parsed.arc_radius = value;
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
        if (letter == 'P') {
            if (!parsed.has_dwell || parsed.has_dwell_parameter || value < 0.0) {
                return Error::UnsupportedCommand;
            }
            parsed.has_dwell_parameter = true;
            parsed.dwell_seconds = value;
            continue;
        }
        return Error::UnsupportedCommand;
    }
    return in_comment ? Error::BadNumberFormat : Error::Ok;
}

motion::MotionType motion_type(MotionMode mode) {
    return mode == MotionMode::Linear ? motion::MotionType::Linear : motion::MotionType::Rapid;
}

bool is_arc_motion(const MotionMode mode) {
    return mode == MotionMode::ArcClockwise || mode == MotionMode::ArcCounterClockwise;
}

Error validate_arc_request(const ParsedBlock& parsed,
                           const motion::MotionRequest& request,
                           const double unit_scale) {
    const bool has_ij = parsed.has_arc_offset[0] || parsed.has_arc_offset[1];
    if (parsed.has_arc_radius && has_ij) return Error::ArcInvalid;
    if (!parsed.has_arc_radius && !has_ij) return Error::ArcInvalid;

    const double start_x = request.start_mm[0];
    const double start_y = request.start_mm[1];
    const double end_x = request.target_mm[0];
    const double end_y = request.target_mm[1];
    const double chord = std::hypot(end_x - start_x, end_y - start_y);
    if (!std::isfinite(chord)) return Error::ArcInvalid;

    if (parsed.has_arc_radius) {
        const double radius = std::abs(parsed.arc_radius * unit_scale);
        if (!std::isfinite(radius) || radius <= kArcCoincidentPointToleranceMm ||
            chord <= kArcCoincidentPointToleranceMm || chord > 2.0 * radius + kArcRadiusToleranceMm) {
            return Error::ArcInvalid;
        }
        return Error::Ok;
    }

    const double offset_x = parsed.has_arc_offset[0] ? parsed.arc_offset[0] * unit_scale : 0.0;
    const double offset_y = parsed.has_arc_offset[1] ? parsed.arc_offset[1] * unit_scale : 0.0;
    const double radius = std::hypot(offset_x, offset_y);
    if (!std::isfinite(radius) || radius <= kArcCoincidentPointToleranceMm) return Error::ArcInvalid;

    const double center_x = start_x + offset_x;
    const double center_y = start_y + offset_y;
    const double end_radius = std::hypot(end_x - center_x, end_y - center_y);
    if (!std::isfinite(end_radius) || std::abs(end_radius - radius) > kArcRadiusToleranceMm) {
        return Error::ArcInvalid;
    }
    return Error::Ok;
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

Error submit_program_pause() {
    switch (planner::planner_submit_program_pause()) {
        case planner::SubmitResult::Accepted: return Error::Ok;
        case planner::SubmitResult::Full: return Error::MotionBusy;
        case planner::SubmitResult::Rejected: return Error::MotionRejected;
    }
    return Error::MotionRejected;
}

Error submit_dwell(const double seconds) {
    if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > 3600.0) {
        return Error::ValueOutOfRange;
    }
    const double milliseconds = seconds * 1000.0;
    switch (planner::planner_submit_dwell(static_cast<std::uint32_t>(std::ceil(milliseconds)))) {
        case planner::SubmitResult::Accepted: return Error::Ok;
        case planner::SubmitResult::Full: return Error::MotionBusy;
        case planner::SubmitResult::Rejected: return Error::MotionRejected;
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
            committed_state.planned_machine_position_mm[axis] = request.target_mm[axis];
        }
    }
    state = committed_state;
}

}  // namespace

Error execute_line_internal(const char* line,
                            std::size_t length,
                            ParserState& state,
                            const bool is_jog) {
    if (length == 0) return Error::Ok;
    if (!is_jog && (line[0] == '$' || line[0] == '[')) return Error::SystemCommandUnsupported;
    if (!is_jog && (state.system_state == SystemState::Alarm || state.system_state == SystemState::Jog)) {
        return Error::SystemGcLock;
    }

    ParsedBlock parsed = {};
    const Error result = parse_line(line, length, parsed);
    if (result != Error::Ok) return result;

    const bool has_axis_word = parsed.has_axis[0] || parsed.has_axis[1] || parsed.has_axis[2];
    if (is_jog) {
        if (!parsed.has_distance_mode || parsed.distance_mode != DistanceMode::Incremental ||
            !parsed.has_feed_rate || parsed.feed_rate <= 0.0 || !has_axis_word ||
            (parsed.has_motion && parsed.motion_mode != MotionMode::Linear) || parsed.has_plane ||
            parsed.has_cutter_compensation || parsed.has_work_coordinate_system ||
            parsed.has_spindle_mode || parsed.has_spindle_speed || parsed.has_coolant_command ||
            parsed.has_program_end) {
            return Error::InvalidJogCommand;
        }
    }

    const bool has_arc_parameters = parsed.has_arc_offset[0] || parsed.has_arc_offset[1] ||
                                    parsed.has_arc_radius;
    const bool arc_motion = parsed.has_motion && is_arc_motion(parsed.motion_mode);
    if (has_arc_parameters && !arc_motion) return Error::UnsupportedCommand;

    ParserState next_state = state;
    if (is_jog) {
        next_state.distance_mode = DistanceMode::Incremental;
        next_state.motion_mode = MotionMode::Linear;
        if (parsed.has_units) next_state.units = parsed.units;
    } else {
        if (parsed.has_units) next_state.units = parsed.units;
        if (parsed.has_distance_mode) next_state.distance_mode = parsed.distance_mode;
        if (parsed.has_motion) next_state.motion_mode = parsed.motion_mode;
        if (parsed.has_plane) next_state.plane = parsed.plane;
        if (parsed.has_cutter_compensation) {
            next_state.cutter_compensation = parsed.cutter_compensation;
        }
        if (parsed.has_work_coordinate_system) {
            next_state.work_coordinate_system = parsed.work_coordinate_system;
            // G54 is currently the machine-origin work coordinate system. Keep
            // the offset explicit so a persisted work offset can be added later.
            next_state.work_offset_mm[0] = 0.0;
            next_state.work_offset_mm[1] = 0.0;
            next_state.work_offset_mm[2] = 0.0;
        }
        if (parsed.has_spindle_mode) next_state.spindle_mode = parsed.spindle_mode;
    }

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
    if (is_jog) {
        request.spindle_power = 0.0;
        request.spindle_mode = motion::SpindleMode::Off;
    }
    for (int axis = 0; axis < 3; ++axis) {
        request.start_mm[axis] = state.planned_machine_position_mm[axis];
        request.target_mm[axis] = state.planned_machine_position_mm[axis];
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (!parsed.has_axis[axis]) continue;
        has_axis = true;
        const double value_mm = parsed.axis_value[axis] * unit_scale;
        request.target_mm[axis] = next_state.distance_mode == DistanceMode::Absolute
                                      ? value_mm + next_state.work_offset_mm[axis]
                                      : state.planned_machine_position_mm[axis] + value_mm;
    }

    if (arc_motion) {
        // Phase one deliberately stops after parser and geometry validation.
        // Until phase two can atomically reserve and submit every chord, a
        // valid arc must not update modal position or submit a partial path.
        if (stepper::machine_position_state() != stepper::PositionState::Known) {
            return Error::PositionUnknown;
        }
        const Error arc_error = validate_arc_request(parsed, request, unit_scale);
        return arc_error == Error::Ok ? Error::UnsupportedCommand : arc_error;
    }

    if (parsed.has_program_end) {
        if (has_axis) return Error::UnsupportedCommand;
        const Error submit_error = submit_spindle_disable();
        if (submit_error != Error::Ok) return submit_error;
        next_state.motion_mode = MotionMode::Linear;
        next_state.plane = PlaneMode::XY;
        next_state.distance_mode = DistanceMode::Absolute;
        next_state.work_coordinate_system = WorkCoordinateSystem::G54;
        next_state.work_offset_mm[0] = 0.0;
        next_state.work_offset_mm[1] = 0.0;
        next_state.work_offset_mm[2] = 0.0;
        next_state.spindle_mode = SpindleMode::Off;
        motion::MotionRequest no_motion_request = {};
        commit_parser_state(state, next_state, no_motion_request, false);
        return Error::ProgramEnd;
    }

    if (parsed.has_program_pause) {
        if (has_axis || parsed.has_dwell || parsed.has_spindle_mode || parsed.has_spindle_speed) {
            return Error::UnsupportedCommand;
        }
        const Error submit_error = submit_program_pause();
        if (submit_error != Error::Ok) return submit_error;
        commit_parser_state(state, next_state, request, false);
        return Error::Ok;
    }

    if (parsed.has_dwell) {
        if (has_axis || !parsed.has_dwell_parameter || parsed.has_spindle_mode ||
            parsed.has_spindle_speed) {
            return Error::UnsupportedCommand;
        }
        const Error submit_error = submit_dwell(parsed.dwell_seconds);
        if (submit_error != Error::Ok) return submit_error;
        commit_parser_state(state, next_state, request, false);
        return Error::Ok;
    }

    // Modal-only G00 is valid in file headers. The observed LightBurn trailer
    // also emits G1 S0 to update laser power without an XY move. Neither form
    // may create a zero-length planner block.
    const bool linear_power_only = parsed.has_motion &&
                                   parsed.motion_mode == MotionMode::Linear &&
                                   parsed.has_spindle_speed;
    if (parsed.has_motion && !has_axis && parsed.motion_mode != MotionMode::Rapid &&
        !linear_power_only) {
        return Error::NoAxisWords;
    }
    if (has_axis && stepper::machine_position_state() != stepper::PositionState::Known) {
        return Error::PositionUnknown;
    }
    if (has_axis) {
        request.has_motion = true;
        motion::MotionExecutionPlan validation_plan = {};
        if (!motion::build_execution_plan(request, validation_plan)) {
            return Error::MotionRejected;
        }
        if (validation_plan.steps[0] == 0 && validation_plan.steps[1] == 0) {
            // No STEP pulse will be emitted. Keep the planned machine position
            // unchanged while committing modal words.
            if (is_jog) return Error::Ok;
            commit_parser_state(state, next_state, request, false);
            return Error::Ok;
        }
        const Error submit_error = submit_motion_request(request);
        if (submit_error != Error::Ok) return submit_error;
    } else if (parsed.has_spindle_mode && parsed.spindle_mode == SpindleMode::Off) {
        const Error submit_error = submit_spindle_disable();
        if (submit_error != Error::Ok) return submit_error;
    }

    if (is_jog) {
        if (has_axis) {
            for (int axis = 0; axis < 3; ++axis) {
                state.planned_machine_position_mm[axis] = request.target_mm[axis];
            }
        }
        return Error::Ok;
    }

    commit_parser_state(state, next_state, request, has_axis);
    return Error::Ok;
}

Error execute_line(const char* line, const std::size_t length, ParserState& state) {
    return execute_line_internal(line, length, state, false);
}

Error execute_jog_line(const char* line, const std::size_t length, ParserState& state) {
    return execute_line_internal(line, length, state, true);
}

Error sync_parser_position_from_machine(ParserState& state) {
    const stepper::MachinePosition position = stepper::machine_position();
    if (position.state != stepper::PositionState::Known) {
        return Error::PositionUnknown;
    }

    state.planned_machine_position_mm[0] = position.millimeters[0];
    state.planned_machine_position_mm[1] = position.millimeters[1];
    state.planned_machine_position_mm[2] = 0.0;
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
        case Error::ArcInvalid: return "arc-invalid";
        case Error::PositionUnknown: return "position-unknown";
        case Error::InvalidJogCommand: return "invalid-jog-command";
        case Error::ProgramEnd: return "program-end";
    }
    return "unknown";
}

}  // namespace ray2::command
