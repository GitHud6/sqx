#pragma once

#include <cstddef>

namespace ray2::command {

enum class Error {
    Ok,
    SystemCommandUnsupported,
    SystemGcLock,
    ExpectedCommandLetter,
    BadNumberFormat,
    NegativeValue,
    ValueOutOfRange,
    UnsupportedCommand,
    ModalGroupViolation,
    WordRepeated,
    NoAxisWords,
    MotionBusy,
    MotionRejected,
};

enum class SystemState { Idle, Alarm, Jog };
enum class DistanceMode { Absolute, Incremental };
enum class Units { Millimeters, Inches };
enum class MotionMode { Rapid, Linear };
enum class PlaneMode { XY };
enum class SpindleMode { Off, Clockwise, CounterClockwise };

struct ParserState {
    SystemState system_state = SystemState::Idle;
    DistanceMode distance_mode = DistanceMode::Absolute;
    Units units = Units::Millimeters;
    MotionMode motion_mode = MotionMode::Rapid;
    PlaneMode plane = PlaneMode::XY;
    double position_mm[3] = {};
    double feed_rate_mm_per_min = 0.0;
    double spindle_speed = 0.0;
    SpindleMode spindle_mode = SpindleMode::Off;
};

struct ParsedBlock {
    bool has_motion = false;
    MotionMode motion_mode = MotionMode::Rapid;
    bool has_distance_mode = false;
    DistanceMode distance_mode = DistanceMode::Absolute;
    bool has_units = false;
    Units units = Units::Millimeters;
    bool has_plane = false;
    PlaneMode plane = PlaneMode::XY;
    bool has_axis[3] = {};
    double axis_value[3] = {};
    bool has_feed_rate = false;
    double feed_rate = 0.0;
    bool has_spindle_speed = false;
    double spindle_speed = 0.0;
    bool has_spindle_mode = false;
    SpindleMode spindle_mode = SpindleMode::Off;
};

Error execute_line(const char* line, std::size_t length, ParserState& state);
const char* error_text(Error error);

}  // namespace ray2::command
