#pragma once

#include <cstddef>

namespace ray2::command {

enum class Error {
    Ok,
    SystemCommandUnsupported,
    SystemGcLock,
    ExpectedCommandLetter,
    BadNumberFormat,
    UnsupportedCommand,
    ModalGroupViolation,
    WordRepeated,
    NoAxisWords,
};

enum class SystemState { Idle, Alarm, Jog };
enum class DistanceMode { Absolute, Incremental };
enum class Units { Millimeters, Inches };
enum class MotionMode { Rapid, Linear };

struct ParserState {
    SystemState system_state = SystemState::Idle;
    DistanceMode distance_mode = DistanceMode::Absolute;
    Units units = Units::Millimeters;
    MotionMode motion_mode = MotionMode::Rapid;
    double position_mm[3] = {};
    double feed_rate_mm_per_min = 0.0;
    double spindle_speed = 0.0;
};

Error execute_line(const char* line, std::size_t length, ParserState& state);
const char* error_text(Error error);

}  // namespace ray2::command
