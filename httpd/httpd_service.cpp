#include "httpd_service.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>

#include "esp_http_server.h"
#include "esp_log.h"

#include "command_dispatch.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "service.hpp"
#include "wifi_service.hpp"

namespace ray2::httpd {
namespace {

constexpr char kLogTag[] = "httpd";
constexpr std::size_t kCommandBodyCapacity = 1024;
constexpr std::size_t kCommandResponseCapacity = 2048;

httpd_handle_t s_server = nullptr;

esp_err_t send_text(httpd_req_t* const request, const char* const body, const char* const content_type,
                    const char* const status = nullptr) {
    if (request == nullptr || body == nullptr || content_type == nullptr) return ESP_ERR_INVALID_ARG;
    if (status != nullptr) {
        const esp_err_t status_result = httpd_resp_set_status(request, status);
        if (status_result != ESP_OK) return status_result;
    }
    esp_err_t result = httpd_resp_set_type(request, content_type);
    if (result != ESP_OK) return result;
    result = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (result != ESP_OK) return result;
    return httpd_resp_sendstr(request, body);
}

esp_err_t send_json_error(httpd_req_t* const request, const char* const status,
                          const char* const error) {
    char body[128] = {};
    const int length = std::snprintf(body, sizeof(body), "{\"error\":\"%s\"}\n", error);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(body)) return ESP_FAIL;
    return send_text(request, body, "application/json", status);
}

struct ResponseCapture {
    char text[kCommandResponseCapacity] = {};
    std::size_t length = 0;
    bool overflowed = false;
};

bool capture_command_response(const std::uint8_t* const data, const std::size_t length,
                              void* const user_data) {
    auto* const capture = static_cast<ResponseCapture*>(user_data);
    if (capture == nullptr || data == nullptr || length == 0 ||
        length > sizeof(capture->text) - 1 - capture->length) {
        if (capture != nullptr) capture->overflowed = true;
        return false;
    }
    std::memcpy(capture->text + capture->length, data, length);
    capture->length += length;
    capture->text[capture->length] = '\0';
    return true;
}

void trim_ascii(const char*& text, std::size_t& length) {
    while (length != 0 && std::isspace(static_cast<unsigned char>(*text)) != 0) {
        ++text;
        --length;
    }
    while (length != 0 && std::isspace(static_cast<unsigned char>(text[length - 1])) != 0) {
        --length;
    }
}

bool equals_ignore_case(const char* const text, const std::size_t length, const char* const expected) {
    const std::size_t expected_length = std::strlen(expected);
    if (length != expected_length) return false;
    for (std::size_t index = 0; index < length; ++index) {
        if (std::toupper(static_cast<unsigned char>(text[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) {
            return false;
        }
    }
    return true;
}

esp_err_t root_get_handler(httpd_req_t* const request) {
    return send_text(request, "", "text/plain");
}

bool append_realtime_response(const char* const command, const std::size_t length,
                              ResponseCapture& capture) {
    if (length == 1 && command[0] == '?') {
        char status[160] = {};
        std::size_t status_length = 0;
        return protocol::command_format_realtime_status(status, sizeof(status), status_length) &&
               capture_command_response(reinterpret_cast<const std::uint8_t*>(status), status_length,
                                        &capture);
    }

    const bool reset_text = equals_ignore_case(command, length, "18") ||
                            equals_ignore_case(command, length, "0x18") ||
                            equals_ignore_case(command, length, "%18");
    const bool reset_byte = length == 1 && static_cast<std::uint8_t>(command[0]) == 0x18;
    const bool hold = length == 1 && command[0] == '!';
    const bool resume = length == 1 && command[0] == '~';
    const bool accepted = (reset_text || reset_byte) ? service::service_request_hard_reset()
                          : hold                  ? service::service_request_feed_hold()
                          : resume                ? service::service_request_cycle_start()
                                                  : false;
    return accepted && capture_command_response(reinterpret_cast<const std::uint8_t*>("ok\r\n"), 4,
                                                 &capture);
}

enum class CommandRequestResult {
    Ok,
    Busy,
    Invalid,
    ResponseOverflow,
    DispatchFailed,
};

CommandRequestResult dispatch_command_text(char* const body, const std::size_t body_length,
                                           ResponseCapture& capture) {
    if (body == nullptr || body_length == 0 || std::memchr(body, '\0', body_length) != nullptr) {
        return CommandRequestResult::Invalid;
    }

    bool dispatched_any = false;
    std::size_t line_start = 0;
    while (line_start < body_length) {
        std::size_t line_end = line_start;
        while (line_end < body_length && body[line_end] != '\n') {
            ++line_end;
        }

        const char* line = body + line_start;
        std::size_t line_length = line_end - line_start;
        trim_ascii(line, line_length);
        if (line_length != 0) {
            dispatched_any = true;
            const bool realtime = (line_length == 1 &&
                                   (line[0] == '?' || line[0] == '!' || line[0] == '~' ||
                                    static_cast<std::uint8_t>(line[0]) == 0x18)) ||
                                  equals_ignore_case(line, line_length, "18") ||
                                  equals_ignore_case(line, line_length, "0x18") ||
                                  equals_ignore_case(line, line_length, "%18");
            if (realtime) {
                if (!append_realtime_response(line, line_length, capture)) {
                    return capture.overflowed ? CommandRequestResult::ResponseOverflow
                                              : CommandRequestResult::DispatchFailed;
                }
            } else {
                bool capacity_blocked = false;
                if (!protocol::command_dispatch_with_response(
                        line, line_length, capacity_blocked, capture_command_response, &capture)) {
                    return capture.overflowed ? CommandRequestResult::ResponseOverflow
                                              : CommandRequestResult::DispatchFailed;
                }
                if (capacity_blocked) return CommandRequestResult::Busy;
            }
        }

        line_start = line_end + 1;
    }

    if (!dispatched_any) return CommandRequestResult::Invalid;

    return CommandRequestResult::Ok;
}

bool receive_command_body(httpd_req_t* const request, char* const body, std::size_t& body_length) {
    body_length = 0;
    if (request == nullptr || body == nullptr) return false;
    if (request->content_len > 0) {
        if (request->content_len >= static_cast<int>(kCommandBodyCapacity)) return false;
        while (body_length < static_cast<std::size_t>(request->content_len)) {
            const int received = httpd_req_recv(
                request, body + body_length,
                static_cast<std::size_t>(request->content_len) - body_length);
            if (received <= 0) return false;
            body_length += static_cast<std::size_t>(received);
        }
        body[body_length] = '\0';
        return true;
    }

    const std::size_t query_length = httpd_req_get_url_query_len(request);
    if (query_length == 0 || query_length >= kCommandBodyCapacity) return false;
    char query[kCommandBodyCapacity] = {};
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, "commandText", body, kCommandBodyCapacity) != ESP_OK &&
        httpd_query_key_value(query, "plain", body, kCommandBodyCapacity) != ESP_OK) {
        return false;
    }
    body_length = std::strlen(body);
    return body_length != 0;
}

esp_err_t command_get_handler(httpd_req_t* const request) {
    char body[kCommandBodyCapacity] = {};
    std::size_t body_length = 0;
    if (!receive_command_body(request, body, body_length)) {
        return send_json_error(request, "400 Bad Request", "invalid-command-body");
    }

    ResponseCapture capture = {};
    switch (dispatch_command_text(body, body_length, capture)) {
        case CommandRequestResult::Ok:
            return send_text(request, capture.text, "text/plain");
        case CommandRequestResult::Busy:
            return send_json_error(request, "409 Conflict", "motion-busy");
        case CommandRequestResult::Invalid:
            return send_json_error(request, "400 Bad Request", "invalid-command");
        case CommandRequestResult::ResponseOverflow:
            return send_json_error(request, "507 Insufficient Storage", "response-too-large");
        case CommandRequestResult::DispatchFailed:
            return send_json_error(request, "500 Internal Server Error", "command-dispatch-failed");
    }
    return ESP_FAIL;
}

bool append_json_path(const char* const source, char* const target, const std::size_t capacity,
                      std::size_t& length) {
    length = 0;
    if (source == nullptr || target == nullptr || capacity == 0) return false;
    for (const unsigned char* cursor = reinterpret_cast<const unsigned char*>(source); *cursor != '\0'; ++cursor) {
        if (*cursor < 0x20) return false;
        if (*cursor == '"' || *cursor == '\\') {
            if (length + 2 >= capacity) return false;
            target[length++] = '\\';
        } else if (length + 1 >= capacity) {
            return false;
        }
        target[length++] = static_cast<char>(*cursor);
    }
    target[length] = '\0';
    return true;
}

struct FileListResponseContext {
    httpd_req_t* request = nullptr;
    bool first = true;
    bool write_failed = false;
};

bool write_file_list_entry(const char* const path, const std::uintmax_t size, const bool is_directory,
                           void* const user_data) {
    auto* const context = static_cast<FileListResponseContext*>(user_data);
    if (context == nullptr || context->request == nullptr) return false;

    char escaped_path[512] = {};
    std::size_t escaped_length = 0;
    if (!append_json_path(path, escaped_path, sizeof(escaped_path), escaped_length)) {
        context->write_failed = true;
        return false;
    }
    char entry[640] = {};
    const int entry_length = std::snprintf(
        entry, sizeof(entry), "%s{\"path\":\"%s\",\"size\":%llu,\"directory\":%s}",
        context->first ? "" : ",", escaped_path, static_cast<unsigned long long>(size),
        is_directory ? "true" : "false");
    if (entry_length <= 0 || static_cast<std::size_t>(entry_length) >= sizeof(entry) ||
        httpd_resp_send_chunk(context->request, entry, entry_length) != ESP_OK) {
        context->write_failed = true;
        return false;
    }
    context->first = false;
    return true;
}

esp_err_t files_get_handler(httpd_req_t* const request) {
    if (!driver::sdcard_is_mounted()) {
        return send_json_error(request, "503 Service Unavailable", "sd-not-mounted");
    }
    if (sd::sd_job_is_active()) {
        return send_json_error(request, "409 Conflict", "sd-job-active");
    }
    esp_err_t result = httpd_resp_set_type(request, "application/json");
    if (result != ESP_OK) return result;
    result = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (result != ESP_OK) return result;
    if (httpd_resp_send_chunk(request, "[", 1) != ESP_OK) return ESP_FAIL;

    FileListResponseContext context = {};
    context.request = request;
    if (!sd::list_root(write_file_list_entry, &context) || context.write_failed) {
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(request, "]\n", 2) == ESP_OK
               ? httpd_resp_send_chunk(request, nullptr, 0)
               : ESP_FAIL;
}

const httpd_uri_t kRootUri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kCommandUri = {
    .uri = "/command",
    .method = HTTP_GET,
    .handler = command_get_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kFilesUri = {
    .uri = "/files",
    .method = HTTP_GET,
    .handler = files_get_handler,
    .user_ctx = nullptr,
};

}  // namespace

esp_err_t httpd_service_start() {
    if (s_server != nullptr) return ESP_OK;
    if (!wifi::wifi_is_started()) return ESP_ERR_INVALID_STATE;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.task_priority = 4;
    config.server_port = 8848;

    httpd_handle_t server = nullptr;
    esp_err_t result = ::httpd_start(&server, &config);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "start failed: %s", esp_err_to_name(result));
        return result;
    }
    result = httpd_register_uri_handler(server, &kRootUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kCommandUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kFilesUri);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "route registration failed: %s", esp_err_to_name(result));
        (void)::httpd_stop(server);
        return result;
    }

    s_server = server;
    ESP_LOGI(kLogTag, "started port=%u routes=/,/command,/files",
             static_cast<unsigned>(config.server_port));
    return ESP_OK;
}

esp_err_t httpd_service_stop() {
    if (s_server == nullptr) return ESP_OK;
    const esp_err_t result = ::httpd_stop(s_server);
    if (result == ESP_OK) {
        s_server = nullptr;
        ESP_LOGI(kLogTag, "stopped");
    }
    return result;
}

bool httpd_service_is_started() {
    return s_server != nullptr;
}

}  // namespace ray2::httpd
