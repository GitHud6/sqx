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
    PositionUnknown,
    InvalidJogCommand,
    ProgramEnd,
};

enum class SystemState { Idle, Alarm, Jog };
enum class DistanceMode { Absolute, Incremental };
enum class Units { Millimeters, Inches };
enum class MotionMode { Rapid, Linear };
enum class PlaneMode { XY };
enum class CutterCompensation { Disabled };
enum class WorkCoordinateSystem { G54 };
enum class SpindleMode { Off, Clockwise, CounterClockwise };

struct ParserState {
    SystemState system_state = SystemState::Idle;
    DistanceMode distance_mode = DistanceMode::Absolute;
    Units units = Units::Millimeters;
    MotionMode motion_mode = MotionMode::Rapid;
    PlaneMode plane = PlaneMode::XY;
    CutterCompensation cutter_compensation = CutterCompensation::Disabled;
    WorkCoordinateSystem work_coordinate_system = WorkCoordinateSystem::G54;
    double work_offset_mm[3] = {};
    double planned_machine_position_mm[3] = {};
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
    bool has_cutter_compensation = false;
    CutterCompensation cutter_compensation = CutterCompensation::Disabled;
    bool has_work_coordinate_system = false;
    WorkCoordinateSystem work_coordinate_system = WorkCoordinateSystem::G54;
    bool has_axis[3] = {};
    double axis_value[3] = {};
    bool has_feed_rate = false;
    double feed_rate = 0.0;
    bool has_spindle_speed = false;
    double spindle_speed = 0.0;
    bool has_spindle_mode = false;
    SpindleMode spindle_mode = SpindleMode::Off;
    // RayCube has no coolant hardware. This remains separate from M3/M4/M5
    // so compatibility M8/M9 commands cannot alter laser state.
    bool has_coolant_command = false;
    bool has_program_end = false;
    bool has_program_pause = false;
    bool has_dwell = false;
    bool has_dwell_parameter = false;
    double dwell_seconds = 0.0;
};

Error execute_line(const char* line, std::size_t length, ParserState& state);
// Executes a constrained `$J=` payload without changing normal G-code modal state.
Error execute_jog_line(const char* line, std::size_t length, ParserState& state);
Error sync_parser_position_from_machine(ParserState& state);
const char* error_text(Error error);

}  // namespace ray2::command
