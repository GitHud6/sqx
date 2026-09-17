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
#include "service.hpp"
#include "planner.hpp"
#include "protocol_tx.hpp"
#include "segment_ring.hpp"
#include "stepper_driver.hpp"
#include "wifi_service.hpp"
#include "httpd_service.hpp"
#include "websocket_service.hpp"
#include "wifi_settings.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace ray2::protocol {

namespace {

struct ResponseTarget {
    TaskHandle_t owner = nullptr;
    CommandResponseWriter writer = nullptr;
    void* user_data = nullptr;
};

portMUX_TYPE s_response_target_lock = portMUX_INITIALIZER_UNLOCKED;
ResponseTarget s_response_target = {};
SemaphoreHandle_t s_response_mutex = nullptr;
SemaphoreHandle_t s_build_info_mutex = nullptr;

// `$I` runs from both the UART protocol task and the bounded network task.
// Keep its compatibility payload out of either task's stack while preserving
// a single complete response for alternate transports.
char s_build_info_response[512] = {};

bool write_all(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) return false;

    ResponseTarget response_target = {};
    portENTER_CRITICAL(&s_response_target_lock);
    if (s_response_target.owner == xTaskGetCurrentTaskHandle()) {
        response_target = s_response_target;
    }
    portEXIT_CRITICAL(&s_response_target_lock);

    if (response_target.writer != nullptr) {
        return response_target.writer(data, length, response_target.user_data);
    }

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

bool starts_with_ignore_case(const char* text, const std::size_t length, const char* prefix) {
    const std::size_t prefix_length = std::strlen(prefix);
    return length >= prefix_length && equals_ignore_case(text, prefix_length, prefix);
}

enum class WifiSettingCommandType {
    StaSsid,
    StaPassword,
    RadioMode,
};

struct WifiSettingCommand {
    WifiSettingCommandType type = WifiSettingCommandType::StaSsid;
    const char* value = nullptr;
    std::size_t value_length = 0;
    bool has_value = false;
};

bool parse_wifi_setting_path(const char* line, const std::size_t length, const char* path,
                             WifiSettingCommandType type, WifiSettingCommand& command) {
    const std::size_t path_length = std::strlen(path);
    if (!starts_with_ignore_case(line, length, path)) return false;
    if (length == path_length) {
        command = {type, nullptr, 0, false};
        return true;
    }
    if (line[path_length] != '=') return false;

    command = {type, line + path_length + 1, length - path_length - 1, true};
    return true;
}

bool is_wifi_setting_command(const char* line, const std::size_t length) {
    return starts_with_ignore_case(line, length, "$STA/SSID") ||
           starts_with_ignore_case(line, length, "$STA/PASSWORD") ||
           starts_with_ignore_case(line, length, "$RADIO/MODE");
}

bool is_wifi_begin_command(const char* line, const std::size_t length) {
    return equals_ignore_case(line, length, "$WIFI/BEGIN");
}

bool is_wifi_begin_path(const char* line, const std::size_t length) {
    return starts_with_ignore_case(line, length, "$WIFI/BEGIN");
}

bool is_work_command_path(const char* line, const std::size_t length) {
    static constexpr char kWorkPath[] = "$WORK";
    return starts_with_ignore_case(line, length, kWorkPath) &&
           (length == sizeof(kWorkPath) - 1 || line[sizeof(kWorkPath) - 1] == '/');
}

bool execute_work_command(const char* const line, const std::size_t length) {
    if (equals_ignore_case(line, length, "$WORK") ||
        equals_ignore_case(line, length, "$WORK/MODE")) {
        return write_text("[MSG:work/mode=1]\r\nok\r\n");
    }

    // This machine currently has only the planar XY configuration. Mode 1
    // is deliberately a no-op, so the App may send it during connection
    // setup even after a Ctrl-X without changing motion configuration.
    if (equals_ignore_case(line, length, "$WORK/MODE=1")) {
        return write_text("[MSG:Switch to carving mode]\r\n[MSG:ok]\r\nok\r\n");
    }
    return write_text("error:work-mode-unsupported\r\n");
}

bool parse_wifi_setting_command(const char* line, const std::size_t length,
                                WifiSettingCommand& command) {
    return parse_wifi_setting_path(line, length, "$STA/SSID", WifiSettingCommandType::StaSsid,
                                   command) ||
           parse_wifi_setting_path(line, length, "$STA/PASSWORD",
                                   WifiSettingCommandType::StaPassword, command) ||
           parse_wifi_setting_path(line, length, "$RADIO/MODE", WifiSettingCommandType::RadioMode,
                                   command);
}

bool report_wifi_settings_unavailable() {
    return write_text("error:wifi-settings-unavailable\r\n");
}

bool report_wifi_setting(const WifiSettingCommandType type) {
    wifi::WifiSettings settings = {};
    if (wifi::wifi_settings_load(settings) != ESP_OK) return report_wifi_settings_unavailable();

    char response[128] = {};
    int length = 0;
    switch (type) {
        case WifiSettingCommandType::StaSsid:
            length = std::snprintf(response, sizeof(response), "[STA:SSID=%s]\r\nok\r\n",
                                   settings.ssid);
            break;
        case WifiSettingCommandType::StaPassword:
            length = std::snprintf(response, sizeof(response),
                                   "[STA:PASSWORD=set=%u|length=%u]\r\nok\r\n",
                                   settings.password[0] == '\0' ? 0U : 1U,
                                   static_cast<unsigned>(std::strlen(settings.password)));
            break;
        case WifiSettingCommandType::RadioMode:
            length = std::snprintf(response, sizeof(response), "[RADIO:MODE=%s]\r\nok\r\n",
                                   wifi::wifi_radio_mode_text(settings.radio_mode));
            break;
    }
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
}

bool copy_wifi_setting_value(const WifiSettingCommand& command, char* buffer,
                             const std::size_t capacity) {
    if (!command.has_value || buffer == nullptr || command.value_length >= capacity) return false;
    std::memcpy(buffer, command.value, command.value_length);
    buffer[command.value_length] = '\0';
    return true;
}

bool execute_wifi_setting_command(const WifiSettingCommand& command) {
    if (!command.has_value) return report_wifi_setting(command.type);

    switch (command.type) {
        case WifiSettingCommandType::StaSsid: {
            char ssid[wifi::kStaSsidCapacity] = {};
            if (!copy_wifi_setting_value(command, ssid, sizeof(ssid))) {
                return write_text("error:wifi-invalid-value\r\n");
            }
            const esp_err_t result = wifi::wifi_settings_store_sta_ssid(ssid);
            if (result == ESP_OK) return write_text("ok\r\n");
            return result == ESP_ERR_INVALID_ARG ? write_text("error:wifi-invalid-value\r\n")
                                               : report_wifi_settings_unavailable();
        }
        case WifiSettingCommandType::StaPassword: {
            char password[wifi::kStaPasswordCapacity] = {};
            if (!copy_wifi_setting_value(command, password, sizeof(password))) {
                return write_text("error:wifi-invalid-value\r\n");
            }
            const esp_err_t result = wifi::wifi_settings_store_sta_password(password);
            if (result == ESP_OK) return write_text("ok\r\n");
            return result == ESP_ERR_INVALID_ARG ? write_text("error:wifi-invalid-value\r\n")
                                               : report_wifi_settings_unavailable();
        }
        case WifiSettingCommandType::RadioMode:
            if (equals_ignore_case(command.value, command.value_length, "AP")) {
                return wifi::wifi_settings_store_radio_mode(wifi::RadioMode::Ap) == ESP_OK
                           ? write_text("ok\r\n")
                           : report_wifi_settings_unavailable();
            }
            if (equals_ignore_case(command.value, command.value_length, "STA")) {
                return wifi::wifi_settings_store_radio_mode(wifi::RadioMode::Sta) == ESP_OK
                           ? write_text("ok\r\n")
                           : report_wifi_settings_unavailable();
            }
            if (equals_ignore_case(command.value, command.value_length, "AP_STA")) {
                return wifi::wifi_settings_store_radio_mode(wifi::RadioMode::ApSta) == ESP_OK
                           ? write_text("ok\r\n")
                           : report_wifi_settings_unavailable();
            }
            return write_text("error:wifi-invalid-mode\r\n");
    }
    return write_text("error:wifi-invalid-command\r\n");
}

bool execute_wifi_begin_command() {
    wifi::WifiSettings settings = {};
    if (wifi::wifi_settings_load(settings) != ESP_OK) return report_wifi_settings_unavailable();

    const esp_err_t result = wifi::wifi_restart();
    if (result != ESP_OK) return write_text("error:wifi-begin-failed\r\n");

    const esp_err_t httpd_result = httpd::httpd_service_start();
    if (httpd_result != ESP_OK) return write_text("error:httpd-start-failed\r\n");

    const esp_err_t websocket_result = websocket::websocket_service_start();
    if (websocket_result != ESP_OK) return write_text("error:websocket-start-failed\r\n");

    char response[64] = {};
    const int length = std::snprintf(response, sizeof(response), "[WIFI:BEGIN=%s]\r\nok\r\n",
                                     wifi::wifi_radio_mode_text(settings.radio_mode));
    return length > 0 && static_cast<std::size_t>(length) < sizeof(response) &&
           write_all(reinterpret_cast<const std::uint8_t*>(response), static_cast<std::size_t>(length));
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

const char* realtime_state_text() {
    if (homing::homing_is_active()) return "Home";
    const service::State service_state = service::service_state();
    if (service_state == service::State::Aborted || service_state == service::State::Faulted) {
        return "Alarm";
    }
    if (service_state == service::State::HoldRequested || service_state == service::State::Held) {
        return "Hold";
    }
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

bool format_build_info(char* const response, const std::size_t capacity,
                       std::size_t& response_length) {
    response_length = 0;
    if (response == nullptr || capacity == 0) return false;

    const auto& x_axis = machine::raycube_axis(machine::Axis::X);
    const auto& y_axis = machine::raycube_axis(machine::Axis::Y);
    static constexpr char kBuildInfo[] =
        "[VER:1.0.0:RayCube-IDF]\r\n"
        "[OPT:V,32,256]\r\n"
        "[Board:Longer Laser Ray2 V1.0]\r\n"
        "[Machine:Ray2]\r\n"
        "[Software:1.0.0]\r\n"
        "[Wavelength:450nm]\r\n"
        "[Power:7.5W]\r\n";

    wifi::RuntimeInfo wifi_info = {};
    char sta_message[192] = {};
    char ap_message[128] = {};
    if (!wifi::wifi_get_runtime_info(wifi_info) || !wifi_info.started) {
        std::snprintf(sta_message, sizeof(sta_message), "[MSG:Mode=OFF]\r\n");
    } else if (wifi_info.sta_enabled && wifi_info.sta_connection_enabled) {
        const char* const status = wifi_info.sta_connected ? "Connected" : "Connecting";
        std::snprintf(sta_message, sizeof(sta_message),
                      "[MSG:Mode=STA:SSID=%s:Status=%s:IP=%s:MAC=%s]\r\n",
                      wifi_info.sta_ssid[0] == '\0' ? "<unset>" : wifi_info.sta_ssid, status,
                      wifi_info.sta_ip[0] == '\0' ? "0.0.0.0" : wifi_info.sta_ip,
                      wifi_info.sta_mac[0] == '\0' ? "<unavailable>" : wifi_info.sta_mac);
    }
    if (wifi_info.ap_enabled) {
        std::snprintf(ap_message, sizeof(ap_message), "[MSG:Mode=AP:SSDI=%s:IP=%s:MAC=%s]\r\n",
                      wifi_info.ap_ssid[0] == '\0' ? "<unset>" : wifi_info.ap_ssid,
                      wifi_info.ap_ip[0] == '\0' ? "0.0.0.0" : wifi_info.ap_ip,
                      wifi_info.ap_mac[0] == '\0' ? "<unavailable>" : wifi_info.ap_mac);
    }
    const int length = std::snprintf(
        response, capacity, "%s[Working Size:XSIZE=%.3f:YSIZE=%.3f]\r\n%s%sok\r\n", kBuildInfo,
        static_cast<double>(x_axis.max_travel_mm), static_cast<double>(y_axis.max_travel_mm),
        sta_message, ap_message);
    if (length <= 0 || static_cast<std::size_t>(length) >= capacity) return false;
    response_length = static_cast<std::size_t>(length);
    return true;
}

bool report_build_info() {
    if (s_build_info_mutex == nullptr ||
        xSemaphoreTake(s_build_info_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    std::size_t response_length = 0;
    const bool result = format_build_info(s_build_info_response, sizeof(s_build_info_response),
                                          response_length) &&
                        write_all(reinterpret_cast<const std::uint8_t*>(s_build_info_response),
                                  response_length);
    xSemaphoreGive(s_build_info_mutex);
    return result;
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
        case stepper::PositionInvalidationReason::HardLimit: return "hard-limit";
        case stepper::PositionInvalidationReason::MotionAlarm: return "motion-alarm";
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

    char response[320] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[POS:state=%s|reason=%s|service=(state=%s,fault=%s)|steps=(%lld,%lld)|machine=(%.4f,%.4f)|planned=(%.4f,%.4f)|ahead=(%.4f,%.4f)]\r\nok\r\n",
        position_state_text(position.state),
        position_invalidation_reason_text(position.invalidation_reason),
        ray2::service::service_state_text(ray2::service::service_state()),
        ray2::service::service_fault_reason_text(ray2::service::service_fault_reason()),
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
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) return ESP_ERR_NO_MEM;
    }
    if (s_response_mutex == nullptr) {
        s_response_mutex = xSemaphoreCreateMutex();
        if (s_response_mutex == nullptr) return ESP_ERR_NO_MEM;
    }
    if (s_build_info_mutex == nullptr) {
        s_build_info_mutex = xSemaphoreCreateMutex();
        if (s_build_info_mutex == nullptr) return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool command_format_realtime_status(char* const response, const std::size_t capacity,
                                    std::size_t& response_length) {
    response_length = 0;
    if (response == nullptr || capacity == 0) return false;

    command::ParserState parser_state = {};
    if (!snapshot_parser_state(parser_state)) return false;

    const stepper::MachinePosition position = stepper::machine_position();
    const stepper::SegmentExecutionDiagnostics segment = stepper::stepper_segment_diagnostics();
    const double feed_rate = segment.active
                                  ? static_cast<double>(segment.active_end_speed_tenths_mm_per_min) / 10.0
                                  : 0.0;
    const std::uint32_t sd_progress_tenths = ray2::sd::sd_job_progress_tenths_percent();
    const int length = std::snprintf(
        response, capacity, "<%s|MPos:%.3f,%.3f|FS:%.0f,%.0f|SD:%s,%u.%u,%u>\r\n",
        realtime_state_text(), position.millimeters[0], position.millimeters[1], feed_rate,
        parser_state.spindle_speed, ray2::sd::sd_job_state_text(ray2::sd::sd_job_state()),
        static_cast<unsigned>(sd_progress_tenths / 10U),
        static_cast<unsigned>(sd_progress_tenths % 10U),
        static_cast<unsigned>(ray2::sd::sd_job_current_line()));
    if (length <= 0 || static_cast<std::size_t>(length) >= capacity) return false;
    response_length = static_cast<std::size_t>(length);
    return true;
}

bool command_format_build_info(char* const response, const std::size_t capacity,
                               std::size_t& response_length) {
    if (s_build_info_mutex == nullptr ||
        xSemaphoreTake(s_build_info_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    const bool result = format_build_info(response, capacity, response_length);
    xSemaphoreGive(s_build_info_mutex);
    return result;
}

bool command_report_realtime_status() {
    char response[160] = {};
    std::size_t response_length = 0;
    if (!command_format_realtime_status(response, sizeof(response), response_length)) {
        return write_text("error:protocol-no-mem\r\n");
    }
    return protocol_tx_publish_realtime_status(reinterpret_cast<const std::uint8_t*>(response),
                                               response_length);
}

bool command_dispatch_impl(const char* line, std::size_t length, bool& capacity_blocked) {
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
        if (homing::homing_is_active()) return write_text("error:homing-busy\r\n");
        return service::service_request_alarm_unlock() ? write_text("ok\r\n")
                                                       : write_text("error:unlock-busy\r\n");
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

    WifiSettingCommand wifi_command = {};
    if (parse_wifi_setting_command(line, length, wifi_command)) {
        return execute_wifi_setting_command(wifi_command);
    }
    if (is_wifi_setting_command(line, length)) {
        return write_text("error:wifi-invalid-command\r\n");
    }
    if (is_wifi_begin_command(line, length)) {
        return execute_wifi_begin_command();
    }
    if (is_wifi_begin_path(line, length)) {
        return write_text("error:wifi-invalid-command\r\n");
    }
    if (is_work_command_path(line, length)) {
        return execute_work_command(line, length);
    }
    ray2::sd::SystemCommand sd_command = {};
    if (ray2::sd::parse_system_command(line, length, sd_command)) {
        switch (sd_command.type) {
            case ray2::sd::SystemCommandType::Status:
                {
                    char response[256] = {};
                    const std::uint32_t progress_tenths = ray2::sd::sd_job_progress_tenths_percent();
                    const int response_length = std::snprintf(
                        response, sizeof(response),
                        "[SD:inserted=%u|mounted=%u|path=/sd|job=%s|progress=%u.%u|line=%u|fault=%s|error=%s]\r\nok\r\n",
                        ray2::driver::sdcard_is_inserted() ? 1U : 0U,
                        ray2::driver::sdcard_is_mounted() ? 1U : 0U,
                        ray2::sd::sd_job_state_text(ray2::sd::sd_job_state()),
                        static_cast<unsigned>(progress_tenths / 10U),
                        static_cast<unsigned>(progress_tenths % 10U),
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

bool command_dispatch(const char* const line, const std::size_t length, bool& capacity_blocked) {
    return command_dispatch_impl(line, length, capacity_blocked);
}

bool command_dispatch_with_response(const char* const line, const std::size_t length,
                                    bool& capacity_blocked,
                                    const CommandResponseWriter response_writer,
                                    void* const response_user_data) {
    capacity_blocked = false;
    if (line == nullptr || response_writer == nullptr) return false;
    if (command_dispatch_start() != ESP_OK) return false;
    if (xSemaphoreTake(s_response_mutex, portMAX_DELAY) != pdTRUE) return false;

    portENTER_CRITICAL(&s_response_target_lock);
    s_response_target = {xTaskGetCurrentTaskHandle(), response_writer, response_user_data};
    portEXIT_CRITICAL(&s_response_target_lock);

    const bool result = command_dispatch_impl(line, length, capacity_blocked);

    portENTER_CRITICAL(&s_response_target_lock);
    s_response_target = {};
    portEXIT_CRITICAL(&s_response_target_lock);
    xSemaphoreGive(s_response_mutex);
    return result;
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
