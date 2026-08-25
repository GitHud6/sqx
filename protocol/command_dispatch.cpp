#include "command_dispatch.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "command_executor.hpp"
#include "sd_command_handler.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "planner.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"
#include "uart0_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace ray2::protocol {

namespace {

bool write_all(const std::uint8_t* data, std::size_t length) {
    std::size_t offset = 0;
    while (offset < length) {
        const int written = ray2::driver::uart0_write(data + offset, length - offset);
        if (written <= 0) {
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }
    return true;
}

bool write_text(const char* text) {
    return write_all(reinterpret_cast<const std::uint8_t*>(text), std::strlen(text));
}

bool report_status(command::Error error) {
    if (error == command::Error::Ok) return write_text("ok\r\n");

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

bool report_position_status() {
    const stepper::MachinePosition position = stepper::machine_position();
    xSemaphoreTake(s_parser_mutex, portMAX_DELAY);
    const double planned_x = s_parser_state.planned_machine_position_mm[0];
    const double planned_y = s_parser_state.planned_machine_position_mm[1];
    xSemaphoreGive(s_parser_mutex);
    const std::size_t queued_blocks = planner::kPlannerQueueLength - planner::planner_free_blocks();
    const stepper::SegmentExecutionDiagnostics segment = stepper::stepper_segment_diagnostics();

    char response[512] = {};
    const int length = std::snprintf(
        response, sizeof(response),
        "[POS:state=%s|reason=%s|steps=(%lld,%lld)|machine=(%.4f,%.4f)|planned=(%.4f,%.4f)|ahead=(%.4f,%.4f)|planner=(queued=%zu,completed=%llu)|segment=(queued=%zu,capacity=%zu,high_water=%zu,underflows=%llu,active=%d,id=%llu,completed_pending=%zu)]\r\nok\r\n",
        position_state_text(position.state),
        position_invalidation_reason_text(position.invalidation_reason),
        static_cast<long long>(position.steps[0]),
        static_cast<long long>(position.steps[1]),
        position.millimeters[0], position.millimeters[1], planned_x, planned_y,
        planned_x - position.millimeters[0], planned_y - position.millimeters[1],
        queued_blocks, static_cast<unsigned long long>(planner::planner_completed_block_count()),
        segment.queued_segments, segment.segment_capacity, segment.high_water_segments,
        static_cast<unsigned long long>(segment.underflow_count), segment.active ? 1 : 0,
        static_cast<unsigned long long>(segment.active_execution_id),
        segment.pending_completions);
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

bool command_dispatch(const char* line, std::size_t length) {
    if (s_parser_mutex == nullptr) {
        s_parser_mutex = xSemaphoreCreateMutex();
        if (s_parser_mutex == nullptr) {
            return write_text("error:protocol-no-mem\r\n");
        }
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

}  // namespace ray2::protocol
