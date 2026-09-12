#include "command_dispatch.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "command_executor.hpp"
#include "homing_service.hpp"
#include "limit_switch_driver.hpp"
#include "sd_command_handler.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "planner.hpp"
#include "protocol_tx.hpp"
#include "segment_ring.hpp"
#include "stepper_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace ray2::protocol {

namespace {

bool write_all(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) return false;
    // The TX pool deliberately uses small fixed packets. Split verbose
    // engineering responses here instead of growing every queued packet.
    while (length != 0) {
        const std::size_t chunk_length = std::min(length, kProtocolTxMessageCapacity);
        if (!protocol_tx_enqueue_normal(data, chunk_length)) return false;
        data += chunk_length;
        length -= chunk_length;
    }
    return true;
}

bool write_text(const char* text) {
    return write_all(reinterpret_cast<const std::uint8_t*>(text), std::strlen(text));
}

bool report_status(command::Error error) {
    if (error == command::Error::Ok || error == command::Error::ProgramEnd) {
        return write_text("ok\r\n");
    }

    char response[80] = {};
    const int length = std::snprintf(response, sizeof(response), "error:%s\r\n", command::error_text(error));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

command::ParserState s_parser_state = {};
SemaphoreHandle_t s_parser_mutex = nullptr;

enum class PositionCommandType {
    Status,
    Set,
};

struct PositionCommand {
    PositionCommandType type = PositionCommandType::Status;
    double machine_position_mm[machine::kAxisCount] = {};
};

bool equals_ignore_case(const char* text, std::size_t length, const char* expected) {
    const std::size_t expected_length = std::strlen(expected);
    if (length != expected_length) {
        return false;
    }
    for (std::size_t index = 0; index < length; ++index) {
        if (std::toupper(static_cast<unsigned char>(text[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) {
            return false;
        }
    }
    return true;
}

bool snapshot_parser_state(command::ParserState& state) {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) return false;
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    state = s_parser_state;
    xSemaphoreGive(s_parser_mutex);
    return true;
}

const char* realtime_state_text(const stepper::MachinePosition& position) {
    if (homing::homing_is_active()) return "Home";
    if (position.state != stepper::PositionState::Known) return "Alarm";
    return planner::planner_is_idle() ? "Idle" : "Run";
}

const char* motion_mode_text(const command::MotionMode mode) {
    switch (mode) {
        case command::MotionMode::Rapid: return "G0";
        case command::MotionMode::Linear: return "G1";
        case command::MotionMode::ArcClockwise: return "G2";
        case command::MotionMode::ArcCounterClockwise: return "G3";
    }
    return "G1";
}

const char* distance_mode_text(const command::DistanceMode mode) {
    return mode == command::DistanceMode::Absolute ? "G90" : "G91";
}

const char* units_text(const command::Units units) {
    return units == command::Units::Millimeters ? "G21" : "G20";
}

const char* spindle_mode_text(const command::SpindleMode mode) {
    switch (mode) {
        case command::SpindleMode::Clockwise: return "M3";
        case command::SpindleMode::CounterClockwise: return "M4";
        case command::SpindleMode::Off: return "M5";
    }
    return "M5";
}

bool report_build_info() {
    static constexpr char kBuildInfo[] =
        "[VER:1.1f.20260826:RayCube-IDF]\r\n"
        "[OPT:V,32,256]\r\n"
        "ok\r\n";
    return write_text(kBuildInfo);
}

bool report_gcode_modes() {
    command::ParserState state = {};
    if (!snapshot_parser_state(state)) return write_text("error:protocol-no-mem\r\n");

    const double feed_rate = state.units == command::Units::Inches
                                 ? state.feed_rate_mm_per_min / 25.4
                                 : state.feed_rate_mm_per_min;
    static char response[192] = {};
    const int length = state.units == command::Units::Inches
                           ? std::snprintf(response, sizeof(response),
                                           "[GC:%s G54 G17 %s %s G94 %s M9 T0 F%.1f S%.0f]\r\nok\r\n",
                                           motion_mode_text(state.motion_mode), units_text(state.units),
                                           distance_mode_text(state.distance_mode),
                                           spindle_mode_text(state.spindle_mode), feed_rate,
                                           state.spindle_speed)
                           : std::snprintf(response, sizeof(response),
                                           "[GC:%s G54 G17 %s %s G94 %s M9 T0 F%.0f S%.0f]\r\nok\r\n",
                                           motion_mode_text(state.motion_mode), units_text(state.units),
                                           distance_mode_text(state.distance_mode),
                                           spindle_mode_text(state.spindle_mode), feed_rate,
                                           state.spindle_speed);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool report_coordinate_parameters() {
    command::ParserState state = {};
    if (!snapshot_parser_state(state)) return write_text("error:protocol-no-mem\r\n");

    static char response[192] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[G54:%.3f,%.3f,%.3f]\r\n[G92:0.000,0.000,0.000]\r\n[TLO:0.000]\r\nok\r\n",
        state.work_offset_mm[0], state.work_offset_mm[1], state.work_offset_mm[2]);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool report_grbl_settings() {
    const auto& x_axis = machine::raycube_axis(machine::Axis::X);
    const auto& y_axis = machine::raycube_axis(machine::Axis::Y);
    const auto& laser = machine::raycube_laser();
    static char response[512] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "$0=%u\r\n$1=250\r\n$10=1\r\n$11=%.3f\r\n$20=1\r\n$22=1\r\n$23=0\r\n"
        "$30=%lu\r\n$31=0\r\n$32=1\r\n$100=%.3f\r\n$101=%.3f\r\n"
        "$110=%.3f\r\n$111=%.3f\r\n$120=%.3f\r\n$121=%.3f\r\n"
        "$130=%.3f\r\n$131=%.3f\r\nok\r\n",
        machine::kStepPulseMicroseconds, planner::kJunctionDeviationMm,
        static_cast<unsigned long>(laser.max_power), static_cast<double>(x_axis.steps_per_mm),
        static_cast<double>(y_axis.steps_per_mm), static_cast<double>(x_axis.max_rate_mm_per_min),
        static_cast<double>(y_axis.max_rate_mm_per_min),
        static_cast<double>(x_axis.acceleration_mm_per_sec2),
        static_cast<double>(y_axis.acceleration_mm_per_sec2), static_cast<double>(x_axis.max_travel_mm),
        static_cast<double>(y_axis.max_travel_mm));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool parse_jog_command(const char* line,
                       const std::size_t length,
                       const char*& payload,
                       std::size_t& payload_length) {
    if (line == nullptr || length <= 3 || line[0] != '$' ||
        std::toupper(static_cast<unsigned char>(line[1])) != 'J' || line[2] != '=') {
        return false;
    }
    payload = line + 3;
    payload_length = length - 3;
    return true;
}

bool is_jog_command(const char* line, const std::size_t length) {
    return line != nullptr && length >= 2 && line[0] == '$' &&
           std::toupper(static_cast<unsigned char>(line[1])) == 'J';
}

bool execute_jog_command(const char* payload, const std::size_t payload_length) {
    if (homing::homing_is_active() || sd::sd_job_is_active() || !planner::planner_is_idle()) {
        return report_status(command::Error::MotionBusy);
    }
    if (stepper::machine_position_state() != stepper::PositionState::Known) {
        return report_status(command::Error::PositionUnknown);
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_jog_line(payload, payload_length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    return report_status(result);
}

bool is_position_command(const char* line, std::size_t length) {
    return line != nullptr && length >= 5 && line[0] == '$' &&
           std::toupper(static_cast<unsigned char>(line[1])) == 'P' &&
           std::toupper(static_cast<unsigned char>(line[2])) == 'O' &&
           std::toupper(static_cast<unsigned char>(line[3])) == 'S' && line[4] == '/';
}

bool parse_position_command(const char* line, std::size_t length, PositionCommand& command) {
    if (!is_position_command(line, length)) {
        return false;
    }

    const char* const payload = line + 5;
    const std::size_t payload_length = length - 5;
    if (equals_ignore_case(payload, payload_length, "STATUS")) {
        command = {};
        command.type = PositionCommandType::Status;
        return true;
    }

    constexpr char kSetPrefix[] = "SET=";
    if (payload_length <= sizeof(kSetPrefix) - 1 ||
        !equals_ignore_case(payload, sizeof(kSetPrefix) - 1, kSetPrefix)) {
        return false;
    }

    const char* const coordinates = payload + sizeof(kSetPrefix) - 1;
    const std::size_t coordinates_length = payload_length - (sizeof(kSetPrefix) - 1);
    if (coordinates_length == 0 || coordinates_length >= 64) {
        return false;
    }

    char buffer[64] = {};
    std::memcpy(buffer, coordinates, coordinates_length);
    char* value_end = nullptr;
    const double x = std::strtod(buffer, &value_end);
    if (value_end == buffer || *value_end != ',') {
        return false;
    }
    const double y = std::strtod(value_end + 1, &value_end);
    if (*value_end != '\0' || !std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }

    command = {};
    command.type = PositionCommandType::Set;
    command.machine_position_mm[0] = x;
    command.machine_position_mm[1] = y;
    return true;
}

const char* position_state_text(stepper::PositionState state) {
    switch (state) {
        case stepper::PositionState::Unknown: return "unknown";
        case stepper::PositionState::Known: return "known";
        case stepper::PositionState::Invalidated: return "invalidated";
    }
    return "invalid";
}

const char* position_invalidation_reason_text(stepper::PositionInvalidationReason reason) {
    switch (reason) {
        case stepper::PositionInvalidationReason::None: return "none";
        case stepper::PositionInvalidationReason::Startup: return "startup";
        case stepper::PositionInvalidationReason::Reset: return "reset";
        case stepper::PositionInvalidationReason::RmtFailure: return "rmt-failure";
        case stepper::PositionInvalidationReason::ExecutionFailure: return "execution-failure";
    }
    return "unknown";
}

bool is_homing_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$H");
}

bool is_homing_status_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$H/STATUS");
}

bool parse_homing_seek_test_command(const char* line, const std::size_t length,
                                    homing::TestAxis& axis) {
    if (equals_ignore_case(line, length, "$H/TEST=X")) {
        axis = homing::TestAxis::X;
        return true;
    }
    if (equals_ignore_case(line, length, "$H/TEST=Y")) {
        axis = homing::TestAxis::Y;
        return true;
    }
    return false;
}

bool report_homing_status() {
    const homing::HomingState state = homing::homing_state();
    const driver::LimitSwitchState limits = driver::limit_switch_driver_state();
    char response[128] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[HOMING:state=%s|limit=(x=%d,y=%d)|debounce=%lu]\r\nok\r\n",
        homing::homing_state_text(state), limits.triggered[0] ? 1 : 0,
        limits.triggered[1] ? 1 : 0, static_cast<unsigned long>(limits.debounce_events));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool start_homing() {
    switch (homing::homing_start()) {
        case homing::StartResult::Accepted:
            return write_text("ok\r\n");
        case homing::StartResult::Busy:
            return write_text("error:homing-busy\r\n");
        case homing::StartResult::Unavailable:
            return write_text("error:homing-unavailable\r\n");
    }
    return write_text("error:homing-unavailable\r\n");
}

bool start_homing_seek_test(const homing::TestAxis axis) {
    switch (homing::homing_start_seek_test(axis)) {
        case homing::StartResult::Accepted:
            return write_text("ok\r\n");
        case homing::StartResult::Busy:
            return write_text("error:homing-busy\r\n");
        case homing::StartResult::Unavailable:
            return write_text("error:homing-unavailable\r\n");
    }
    return write_text("error:homing-unavailable\r\n");
}

bool report_position_status() {
    const stepper::MachinePosition position = stepper::machine_position();
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const double planned_x = s_parser_state.planned_machine_position_mm[0];
    const double planned_y = s_parser_state.planned_machine_position_mm[1];
    xSemaphoreGive(s_parser_mutex);

    char response[256] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[POS:state=%s|reason=%s|steps=(%lld,%lld)|machine=(%.4f,%.4f)|planned=(%.4f,%.4f)|ahead=(%.4f,%.4f)]\r\nok\r\n",
        position_state_text(position.state),
        position_invalidation_reason_text(position.invalidation_reason),
        static_cast<long long>(position.steps[0]),
        static_cast<long long>(position.steps[1]),
        position.millimeters[0], position.millimeters[1], planned_x, planned_y,
        planned_x - position.millimeters[0], planned_y - position.millimeters[1]);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}


bool set_known_position(const PositionCommand& command) {
    if (!planner::planner_is_idle() || sd::sd_job_is_active()) {
        return write_text("error:position-busy\r\n");
    }
    if (stepper::stepper_set_machine_position_mm(command.machine_position_mm) != ESP_OK) {
        return write_text("error:position-invalid\r\n");
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error sync_result = command::sync_parser_position_from_machine(s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    if (sync_result != command::Error::Ok) {
        return report_status(sync_result);
    }
    return report_position_status();
}

struct SdListResponseContext {
    bool write_failed = false;
};

bool write_sd_list_entry(const char* path, std::uintmax_t size, bool is_directory, void* user_data) {
    auto* context = static_cast<SdListResponseContext*>(user_data);
    char response[192] = {};
    const int length = std::snprintf(response, sizeof(response),
                                     "[FILE:%s|SIZE:%ju|TYPE:%s]\r\n",
                                     path, size, is_directory ? "DIR" : "FILE");
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(response) ||
        !write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length))) {
        context->write_failed = true;
        return false;
    }
    return true;
}

}  // namespace

esp_err_t command_dispatch_start() {
    if (s_parser_mutex != nullptr) {
        return ESP_OK;
    }

    s_parser_mutex = xSemaphoreCreateMutex();
    return s_parser_mutex == nullptr ? ESP_ERR_NO_MEM : ESP_OK;
}

bool command_report_realtime_status() {
    command::ParserState parser_state = {};
    if (!snapshot_parser_state(parser_state)) return write_text("error:protocol-no-mem\r\n");

    const stepper::MachinePosition position = stepper::machine_position();
    const stepper::SegmentExecutionDiagnostics segment = stepper::stepper_segment_diagnostics();
    const double feed_rate = segment.active
                                 ? static_cast<double>(segment.active_end_speed_tenths_mm_per_min) / 10.0
                                 : 0.0;
    static char response[160] = {};
    const int length = std::snprintf(
        response, sizeof(response), "<%s|MPos:%.3f,%.3f|FS:%.0f,%.0f>\r\n",
        realtime_state_text(position), position.millimeters[0], position.millimeters[1], feed_rate,
        parser_state.spindle_speed);
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           protocol_tx_publish_realtime_status(
               reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool command_dispatch(const char* line, std::size_t length, bool& capacity_blocked) {
    capacity_blocked = false;
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) {
            return write_text("error:protocol-no-mem\r\n");
        }
    }

    if (equals_ignore_case(line, length, "$I")) {
        return report_build_info();
    }
    if (equals_ignore_case(line, length, "$G")) {
        return report_gcode_modes();
    }
    if (equals_ignore_case(line, length, "$#")) {
        return report_coordinate_parameters();
    }
    if (equals_ignore_case(line, length, "$$")) {
        return report_grbl_settings();
    }
    if (equals_ignore_case(line, length, "$X")) {
        // ray2-old returns ok outside Alarm as a no-op. RayCube currently has
        // no separate Alarm lock, so this must not promote an unknown position.
        return homing::homing_is_active() ? write_text("error:homing-busy\r\n") : write_text("ok\r\n");
    }
    const char* jog_payload = nullptr;
    std::size_t jog_payload_length = 0;
    if (parse_jog_command(line, length, jog_payload, jog_payload_length)) {
        return execute_jog_command(jog_payload, jog_payload_length);
    }
    if (is_jog_command(line, length)) {
        return report_status(command::Error::InvalidJogCommand);
    }

    if (is_homing_status_command(line, length)) {
        return report_homing_status();
    }
    homing::TestAxis homing_test_axis = {};
    if (parse_homing_seek_test_command(line, length, homing_test_axis)) {
        return start_homing_seek_test(homing_test_axis);
    }
    if (is_homing_command(line, length)) {
        return start_homing();
    }
    if (homing::homing_is_active()) {
        return write_text("error:homing-busy\r\n");
    }

    PositionCommand position_command = {};
    if (parse_position_command(line, length, position_command)) {
        return position_command.type == PositionCommandType::Status
                   ? report_position_status()
                   : set_known_position(position_command);
    }
    if (is_position_command(line, length)) {
        return write_text("error:position-invalid-command\r\n");
    }

    ray2::sd::SystemCommand sd_command = {};
    if (ray2::sd::parse_system_command(line, length, sd_command)) {
        switch (sd_command.type) {
            case ray2::sd::SystemCommandType::Status:
                if (!ray2::driver::sdcard_is_mounted()) {
                    return write_text("error:sd-not-mounted\r\n");
                }
                {
                    char response[192] = {};
                    const int response_length = std::snprintf(
                        response, sizeof(response),
                        "[SD:mounted path=/sd|job=%s|line=%u|fault=%s|error=%s]\r\nok\r\n",
                        ray2::sd::sd_job_state_text(ray2::sd::sd_job_state()),
                        static_cast<unsigned>(ray2::sd::sd_job_current_line()),
                        ray2::sd::sd_job_fault_text(ray2::sd::sd_job_fault()),
                        command::error_text(ray2::sd::sd_job_command_error()));
                    return response_length > 0 && static_cast<std::size_t>(response_length) < sizeof(response) &&
                           write_all(reinterpret_cast<const std::uint8_t*>(response),
                                     static_cast<std::size_t>(response_length));
                }
            case ray2::sd::SystemCommandType::List:
                if (!ray2::driver::sdcard_is_mounted()) {
                    return write_text("error:sd-not-mounted\r\n");
                }
                {
                    SdListResponseContext context = {};
                    if (!ray2::sd::list_root(write_sd_list_entry, &context) || context.write_failed) {
                        return write_text("error:sd-list-failed\r\n");
                    }
                    return write_text("ok\r\n");
                }
            case ray2::sd::SystemCommandType::Run: {
                if (ray2::stepper::machine_position_state() != ray2::stepper::PositionState::Known) {
                    return write_text("error:position-unknown\r\n");
                }
                const esp_err_t result = ray2::sd::sd_job_start(sd_command.path);
                if (result == ESP_ERR_INVALID_STATE) {
                    if (!ray2::driver::sdcard_is_mounted()) {
                        return write_text("error:sd-not-mounted\r\n");
                    }
                    return write_text("error:sd-job-active\r\n");
                }
                if (result != ESP_OK) {
                    return write_text("error:sd-run-failed\r\n");
                }
                char response[192] = {};
                const int response_length = std::snprintf(
                    response, sizeof(response), "[SD:run accepted path=%s]\r\nok\r\n", sd_command.path);
                return response_length > 0 && static_cast<std::size_t>(response_length) < sizeof(response) &&
                       write_all(reinterpret_cast<const std::uint8_t*>(response),
                                 static_cast<std::size_t>(response_length));
            }
        }
    }
    if (ray2::sd::is_system_command(line, length)) {
        return write_text("error:sd-invalid-command\r\n");
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_line(line, length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    if (result == command::Error::MotionBusy) {
        capacity_blocked = true;
        return true;
    }
    return report_status(result);
}

command::Error command_dispatch_external(const char* line, std::size_t length) {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) {
            return command::Error::MotionRejected;
        }
    }

    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::execute_line(line, length, s_parser_state);
    xSemaphoreGive(s_parser_mutex);
    return result;
}

command::Error command_sync_parser_position_from_machine() {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) return command::Error::MotionRejected;
    }
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const command::Error result = command::sync_parser_position_from_machine(s_parser_state);
    if (result == command::Error::Ok) {
        s_parser_state.system_state = command::SystemState::Idle;
    }
    xSemaphoreGive(s_parser_mutex);
    return result;
}

void command_reset_after_hard_reset() {
    if (s_parser_mutex == nullptr) return;
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    s_parser_state = {};
    s_parser_state.system_state = command::SystemState::Alarm;
    xSemaphoreGive(s_parser_mutex);
}

}  // namespace ray2::protocol
