#include "command_dispatch.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>

#include "command_executor.hpp"
#include "sd_command_handler.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
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
