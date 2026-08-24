#include "command_dispatch.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>

#include "esp_log.h"

#include "command_executor.hpp"
#include "sd_command_handler.hpp"
#include "sd_file_service.hpp"
#include "sdcard_driver.hpp"
#include "uart0_driver.hpp"

namespace ray2::protocol {

namespace {

constexpr char kProtocolTag[] = "protocol";

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
    if (line != nullptr && length > 0 && line[0] == '$') {
        ESP_LOGI(kProtocolTag,
                 "system command length=%u text='%.*s'",
                 static_cast<unsigned>(length),
                 static_cast<int>(length),
                 line);
        ESP_LOG_BUFFER_HEXDUMP(kProtocolTag, line, length, ESP_LOG_INFO);
    }

    ray2::sd::SystemCommand sd_command = {};
    if (ray2::sd::parse_system_command(line, length, sd_command)) {
        switch (sd_command.type) {
            case ray2::sd::SystemCommandType::Status:
                if (!ray2::driver::sdcard_is_mounted()) {
                    return write_text("error:sd-not-mounted\r\n");
                }
                return write_text("[SD:mounted path=/sd]\r\nok\r\n");
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
                char response[192] = {};
                const int response_length = std::snprintf(
                    response, sizeof(response), "[SD:run reserved path=%s]\r\nok\r\n", sd_command.path);
                return response_length > 0 && static_cast<std::size_t>(response_length) < sizeof(response) &&
                       write_all(reinterpret_cast<const std::uint8_t*>(response),
                                 static_cast<std::size_t>(response_length));
            }
        }
    }
    if (ray2::sd::is_system_command(line, length)) {
        return write_text("error:sd-invalid-command\r\n");
    }
    return report_status(command::execute_line(line, length, s_parser_state));
}

}  // namespace ray2::protocol
