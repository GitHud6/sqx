#include "httpd_service.hpp"

#include <cstdio>
#include <cstring>

#include "esp_http_server.h"
#include "esp_log.h"

#include "network_command_adapter.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "wifi_service.hpp"

namespace ray2::httpd {
namespace {

constexpr char kLogTag[] = "httpd";
constexpr std::size_t kCommandBodyCapacity = 1024;

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

esp_err_t root_get_handler(httpd_req_t* const request) {
    return send_text(request, "", "text/plain");
}

int hex_value(const char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

bool url_decode_in_place(char* const text, std::size_t& length) {
    if (text == nullptr) return false;
    std::size_t read = 0;
    std::size_t write = 0;
    while (read < length) {
        const char value = text[read++];
        if (value == '+') {
            text[write++] = ' ';
            continue;
        }
        if (value != '%') {
            text[write++] = value;
            continue;
        }
        if (read + 1 >= length) return false;
        const int high = hex_value(text[read++]);
        const int low = hex_value(text[read++]);
        if (high < 0 || low < 0) return false;
        const char decoded = static_cast<char>((high << 4) | low);
        if (decoded == '\0') return false;
        text[write++] = decoded;
    }
    text[write] = '\0';
    length = write;
    return write != 0;
}

bool receive_command_body(httpd_req_t* const request, char* const body, std::size_t& body_length,
                          bool& command_text) {
    body_length = 0;
    command_text = false;
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
        command_text = true;
        return body_length != 0;
    }

    const std::size_t query_length = httpd_req_get_url_query_len(request);
    if (query_length == 0 || query_length >= kCommandBodyCapacity) return false;
    char query[kCommandBodyCapacity] = {};
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, "commandText", body, kCommandBodyCapacity) == ESP_OK) {
        command_text = true;
    } else if (httpd_query_key_value(query, "plain", body, kCommandBodyCapacity) == ESP_OK) {
        command_text = false;
    } else {
        return false;
    }
    body_length = std::strlen(body);
    return body_length != 0 && url_decode_in_place(body, body_length);
}

esp_err_t command_get_handler(httpd_req_t* const request) {
    char body[kCommandBodyCapacity] = {};
    std::size_t body_length = 0;
    bool command_text = false;
    if (!receive_command_body(request, body, body_length, command_text)) {
        return send_json_error(request, "400 Bad Request", "invalid-command-body");
    }

    switch (network::network_command_adapter_submit(body, body_length, command_text)) {
        case network::SubmitResult::Accepted:
            return send_text(request, "", "text/plain");
        case network::SubmitResult::Busy:
            return send_json_error(request, "409 Conflict", "network-input-busy");
        case network::SubmitResult::Invalid:
            return send_json_error(request, "400 Bad Request", "invalid-command");
        case network::SubmitResult::Unavailable:
            return send_json_error(request, "503 Service Unavailable", "network-input-unavailable");
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
