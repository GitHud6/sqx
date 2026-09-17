#include "httpd_service.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "network_command_adapter.hpp"
#include "sd_file_service.hpp"
#include "sd_job_service.hpp"
#include "sdcard_driver.hpp"
#include "wifi_service.hpp"

namespace ray2::httpd {
namespace {

constexpr char kLogTag[] = "httpd";
constexpr std::size_t kCommandBodyCapacity = 1024;
// LongerStudio opens several short-lived HTTP requests while the persistent
// command connection is still active. Match the product-side server budget.
constexpr std::size_t kMaximumHttpSessions = 8;
constexpr std::size_t kFileTransferBufferCapacity = 1024;
constexpr std::size_t kMaximumUploadBytes = 16U * 1024U * 1024U;
// Start small: this handler runs beside the WebSocket and several App setup
// connections, so a fixed 16 KiB contiguous allocation is unnecessarily
// fragile. The response expands only when the actual directory requires it.
constexpr std::size_t kUploadListingInitialCapacity = 1024;
constexpr std::size_t kUploadListingMaximumCapacity = 16U * 1024U;
// App upload metadata consists of path, size and file headers. Keep this
// bounded so an HTTP handler cannot exhaust the server task's 8 KiB stack.
constexpr std::size_t kMultipartHeaderCapacity = 2048;

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

esp_err_t send_empty_accepted(httpd_req_t* const request) {
    if (request == nullptr) return ESP_ERR_INVALID_ARG;

    // ESP-IDF emits each HTTP response header fragment through a separate
    // send(). Disabling Nagle for this short response prevents the final CRLF
    // from waiting on an ACK from the preceding tiny packet.
    const int fd = httpd_req_to_sockfd(request);
    int no_delay = 1;
    const bool no_delay_set =
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) == 0;
    if (!no_delay_set) {
        ESP_LOGW(kLogTag, "TCP_NODELAY unavailable fd=%d", fd);
    }

    esp_err_t result = httpd_resp_set_type(request, "text/plain");
    if (result == ESP_OK) result = httpd_resp_send(request, nullptr, 0);

    if (no_delay_set) {
        no_delay = 0;
        if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) != 0) {
            ESP_LOGW(kLogTag, "TCP_NODELAY restore failed fd=%d", fd);
        }
    }
    return result;
}

esp_err_t send_json_error(httpd_req_t* const request, const char* const status,
                          const char* const error) {
    char body[128] = {};
    const int length = std::snprintf(body, sizeof(body), "{\"error\":\"%s\"}\n", error);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(body)) return ESP_FAIL;
    return send_text(request, body, "application/json", status);
}

esp_err_t send_file_error(httpd_req_t* const request, const sd::FileResult result) {
    switch (result) {
        case sd::FileResult::InvalidPath:
            return send_json_error(request, "400 Bad Request", sd::file_result_text(result));
        case sd::FileResult::NotMounted:
            return send_json_error(request, "503 Service Unavailable", sd::file_result_text(result));
        case sd::FileResult::Busy:
            return send_json_error(request, "409 Conflict", sd::file_result_text(result));
        case sd::FileResult::NotFound:
            return send_json_error(request, "404 Not Found", sd::file_result_text(result));
        case sd::FileResult::NotDirectory:
        case sd::FileResult::IsDirectory:
        case sd::FileResult::AlreadyExists:
            return send_json_error(request, "409 Conflict", sd::file_result_text(result));
        case sd::FileResult::IoError:
            return send_json_error(request, "500 Internal Server Error", sd::file_result_text(result));
        case sd::FileResult::Ok: return ESP_OK;
    }
    return ESP_FAIL;
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

bool receive_path_query(httpd_req_t* const request, char* const path, const std::size_t capacity) {
    if (request == nullptr || path == nullptr || capacity == 0) return false;
    const std::size_t query_length = httpd_req_get_url_query_len(request);
    if (query_length == 0) {
        if (capacity < 2) return false;
        path[0] = '/';
        path[1] = '\0';
        return true;
    }
    if (query_length >= capacity) return false;

    char query[kCommandBodyCapacity] = {};
    if (query_length >= sizeof(query) ||
        httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "path", path, capacity) != ESP_OK) {
        return false;
    }
    std::size_t path_length = std::strlen(path);
    return path_length != 0 && url_decode_in_place(path, path_length);
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
            return send_empty_accepted(request);
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

void format_human_size(const std::uintmax_t size, char* const target, const std::size_t capacity) {
    if (target == nullptr || capacity == 0) return;

    static constexpr const char* kUnits[] = {"B", "KB", "MB", "GB", "TB"};
    double value = static_cast<double>(size);
    std::size_t unit = 0;
    while (value >= 1024.0 && unit + 1 < sizeof(kUnits) / sizeof(kUnits[0])) {
        value /= 1024.0;
        ++unit;
    }
    if (unit == 0) {
        (void)std::snprintf(target, capacity, "%llu %s", static_cast<unsigned long long>(size),
                            kUnits[unit]);
    } else {
        (void)std::snprintf(target, capacity, "%.2f %s", value, kUnits[unit]);
    }
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

struct UploadListResponseContext {
    char* response = nullptr;
    std::size_t capacity = 0;
    std::size_t length = 0;
    bool first = true;
    bool write_failed = false;
};

bool append_upload_list_response(UploadListResponseContext& context, const char* const text,
                                 const std::size_t text_length) {
    if (context.response == nullptr || text == nullptr || context.length > context.capacity) {
        context.write_failed = true;
        return false;
    }

    const std::size_t required_capacity = context.length + text_length + 1;
    if (required_capacity > context.capacity) {
        std::size_t expanded_capacity = context.capacity;
        while (expanded_capacity < required_capacity &&
               expanded_capacity < kUploadListingMaximumCapacity) {
            expanded_capacity *= 2;
        }
        if (expanded_capacity < required_capacity) {
            context.write_failed = true;
            return false;
        }
        char* const expanded_response = static_cast<char*>(
            heap_caps_realloc(context.response, expanded_capacity, MALLOC_CAP_8BIT));
        if (expanded_response == nullptr) {
            context.write_failed = true;
            return false;
        }
        context.response = expanded_response;
        context.capacity = expanded_capacity;
    }
    std::memcpy(context.response + context.length, text, text_length);
    context.length += text_length;
    context.response[context.length] = '\0';
    return true;
}

bool write_upload_list_entry(const char* const path, const std::uintmax_t size, const bool is_directory,
                             void* const user_data) {
    auto* const context = static_cast<UploadListResponseContext*>(user_data);
    if (context == nullptr || context->response == nullptr || path == nullptr) return false;

    const char* const name = std::strrchr(path, '/');
    if (name == nullptr || name[1] == '\0') {
        context->write_failed = true;
        return false;
    }
    char escaped_name[512] = {};
    std::size_t escaped_length = 0;
    if (!append_json_path(name + 1, escaped_name, sizeof(escaped_name), escaped_length)) {
        context->write_failed = true;
        return false;
    }
    char size_text[32] = {};
    if (is_directory) {
        std::strcpy(size_text, "-1");
    } else {
        format_human_size(size, size_text, sizeof(size_text));
    }
    char entry[1152] = {};
    const int entry_length = std::snprintf(
        entry, sizeof(entry),
        "%s{\"name\":\"%s\",\"shortname\":\"%s\",\"size\":\"%s\",\"datetime\":\"\"}",
        context->first ? "" : ",", escaped_name, escaped_name, size_text);
    if (entry_length <= 0 || static_cast<std::size_t>(entry_length) >= sizeof(entry) ||
        !append_upload_list_response(*context, entry, static_cast<std::size_t>(entry_length))) {
        context->write_failed = true;
        return false;
    }
    context->first = false;
    return true;
}

esp_err_t upload_get_handler(httpd_req_t* const request) {
    const sd::FileResult validation = sd::validate_directory("/");
    if (validation != sd::FileResult::Ok) return send_file_error(request, validation);

    UploadListResponseContext context = {};
    context.response = static_cast<char*>(
        heap_caps_malloc(kUploadListingInitialCapacity, MALLOC_CAP_8BIT));
    context.capacity = kUploadListingInitialCapacity;
    if (context.response == nullptr) {
        return send_json_error(request, "503 Service Unavailable", "upload-list-unavailable");
    }
    context.response[0] = '\0';
    static constexpr char kResponsePrefix[] = "{\"files\":[";
    if (!append_upload_list_response(context, kResponsePrefix, sizeof(kResponsePrefix) - 1)) {
        heap_caps_free(context.response);
        return send_json_error(request, "503 Service Unavailable", "upload-list-unavailable");
    }
    const sd::FileResult list_result = sd::list_directory("/", write_upload_list_entry, &context);
    static constexpr char kResponseTail[] =
        "],\"path\":\"/\",\"total\":\"-1\",\"used\":\"-1\",\"occupation\":\"-1\","
        "\"mode\":\"direct\",\"status\":\"Ok\"}";
    if (list_result != sd::FileResult::Ok || context.write_failed ||
        !append_upload_list_response(context, kResponseTail, sizeof(kResponseTail) - 1)) {
        ESP_LOGW(kLogTag, "upload list failed result=%s", sd::file_result_text(list_result));
        heap_caps_free(context.response);
        return list_result == sd::FileResult::Ok
                   ? send_json_error(request, "507 Insufficient Storage", "upload-list-too-large")
                   : send_file_error(request, list_result);
    }

    esp_err_t result = httpd_resp_set_type(request, "application/json");
    if (result == ESP_OK) result = httpd_resp_set_hdr(request, "Cache-Control", "no-cache");
    if (result == ESP_OK) result = httpd_resp_send(request, context.response, context.length);
    heap_caps_free(context.response);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "upload list send failed: %s", esp_err_to_name(result));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t file_download_handler(httpd_req_t* const request, const char* const path) {
    sd::FileReadSession session = {};
    std::uintmax_t size = 0;
    const sd::FileResult open_result = sd::file_read_open(path, session, size);
    if (open_result != sd::FileResult::Ok) return send_file_error(request, open_result);

    esp_err_t result = httpd_resp_set_type(request, "application/octet-stream");
    if (result == ESP_OK) result = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    char transfer_buffer[kFileTransferBufferCapacity] = {};
    while (result == ESP_OK) {
        std::size_t length = 0;
        const sd::FileResult read_result =
            sd::file_read_next(session, transfer_buffer, sizeof(transfer_buffer), length);
        if (read_result != sd::FileResult::Ok) {
            ESP_LOGW(kLogTag, "file download failed path=%s result=%s", path,
                     sd::file_result_text(read_result));
            result = ESP_FAIL;
            break;
        }
        if (length == 0) break;
        result = httpd_resp_send_chunk(request, transfer_buffer, length);
    }
    sd::file_read_close(session);
    if (result != ESP_OK) return result;
    return httpd_resp_send_chunk(request, nullptr, 0);
}

esp_err_t files_get_handler(httpd_req_t* const request) {
    char requested_path[sd::kFilePathCapacity] = {};
    if (!receive_path_query(request, requested_path, sizeof(requested_path))) {
        return send_json_error(request, "400 Bad Request", "invalid-path");
    }
    const sd::FileResult validation = sd::validate_directory(requested_path);
    if (validation == sd::FileResult::NotDirectory) {
        return file_download_handler(request, requested_path);
    }
    if (validation != sd::FileResult::Ok) return send_file_error(request, validation);

    esp_err_t result = httpd_resp_set_type(request, "application/json");
    if (result != ESP_OK) return result;
    result = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (result != ESP_OK) return result;
    if (httpd_resp_send_chunk(request, "[", 1) != ESP_OK) return ESP_FAIL;

    FileListResponseContext context = {};
    context.request = request;
    const sd::FileResult list_result =
        sd::list_directory(requested_path, write_file_list_entry, &context);
    if (list_result != sd::FileResult::Ok || context.write_failed) {
        ESP_LOGW(kLogTag, "file list failed path=%s result=%s", requested_path,
                 sd::file_result_text(list_result));
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(request, "]\n", 2) == ESP_OK
               ? httpd_resp_send_chunk(request, nullptr, 0)
               : ESP_FAIL;
}

bool request_is_multipart_form(httpd_req_t* const request) {
    if (request == nullptr) return false;
    constexpr char kHeaderName[] = "Content-Type";
    char content_type[128] = {};
    const std::size_t length = httpd_req_get_hdr_value_len(request, kHeaderName);
    return length > 0 && length < sizeof(content_type) &&
           httpd_req_get_hdr_value_str(request, kHeaderName, content_type, sizeof(content_type)) == ESP_OK &&
           std::strstr(content_type, "multipart/form-data") != nullptr;
}

bool parse_decimal_size(const char* const text, std::size_t& value) {
    value = 0;
    if (text == nullptr || text[0] == '\0') return false;
    for (const char* cursor = text; *cursor != '\0'; ++cursor) {
        if (*cursor < '0' || *cursor > '9') return false;
        const std::size_t digit = static_cast<std::size_t>(*cursor - '0');
        if (value > (kMaximumUploadBytes - digit) / 10U) return false;
        value = value * 10U + digit;
    }
    return true;
}

bool parse_multipart_text_field(char* const body, const char* const field, char* const value,
                                const std::size_t value_capacity) {
    if (body == nullptr || field == nullptr || value == nullptr || value_capacity == 0) return false;
    char marker[64] = {};
    const int marker_length = std::snprintf(marker, sizeof(marker), "name=\"%s\"", field);
    if (marker_length <= 0 || static_cast<std::size_t>(marker_length) >= sizeof(marker)) return false;
    const char* const part = std::strstr(body, marker);
    if (part == nullptr) return false;
    const char* const data = std::strstr(part, "\r\n\r\n");
    if (data == nullptr) return false;
    const char* const end = std::strstr(data + 4, "\r\n");
    if (end == nullptr || end < data + 4) return false;
    const std::size_t length = static_cast<std::size_t>(end - (data + 4));
    if (length == 0 || length >= value_capacity) return false;
    std::memcpy(value, data + 4, length);
    value[length] = '\0';
    return true;
}

struct MultipartUploadInfo {
    char directory[sd::kFilePathCapacity] = "/";
    char filename[sd::kFilePathCapacity] = {};
    std::size_t file_size = 0;
    std::size_t file_payload_offset = 0;
};

bool filename_is_valid(const char* const filename) {
    if (filename == nullptr || filename[0] == '\0' || std::strcmp(filename, ".") == 0 ||
        std::strcmp(filename, "..") == 0) {
        return false;
    }
    for (const unsigned char* cursor = reinterpret_cast<const unsigned char*>(filename); *cursor != '\0';
         ++cursor) {
        if (*cursor < 0x20U || *cursor == '/' || *cursor == '\\') return false;
    }
    return true;
}

bool parse_multipart_upload_info(char* const body, MultipartUploadInfo& info) {
    char size_text[32] = {};
    char path[sd::kFilePathCapacity] = {};
    if (parse_multipart_text_field(body, "path", path, sizeof(path))) {
        std::strcpy(info.directory, path);
    }
    if (!parse_multipart_text_field(body, "size", size_text, sizeof(size_text)) ||
        !parse_decimal_size(size_text, info.file_size)) {
        return false;
    }

    const char* const file_part = std::strstr(body, "name=\"file\"");
    if (file_part == nullptr) return false;
    const char* const filename_marker = std::strstr(file_part, "filename=\"");
    if (filename_marker == nullptr) return false;
    const char* const filename_start = filename_marker + std::strlen("filename=\"");
    const char* const filename_end = std::strchr(filename_start, '\"');
    if (filename_end == nullptr) return false;
    const std::size_t filename_length = static_cast<std::size_t>(filename_end - filename_start);
    if (filename_length == 0 || filename_length >= sizeof(info.filename)) return false;
    std::memcpy(info.filename, filename_start, filename_length);
    info.filename[filename_length] = '\0';
    if (!filename_is_valid(info.filename)) return false;

    const char* const payload = std::strstr(filename_end, "\r\n\r\n");
    if (payload == nullptr) return false;
    info.file_payload_offset = static_cast<std::size_t>((payload + 4) - body);
    return true;
}

bool build_multipart_target_path(const MultipartUploadInfo& info, char* const target,
                                 const std::size_t target_capacity) {
    if (target == nullptr || target_capacity == 0) return false;
    const int length = std::snprintf(target, target_capacity, "%s%s%s", info.directory,
                                     std::strcmp(info.directory, "/") == 0 ? "" : "/", info.filename);
    if (length <= 0 || static_cast<std::size_t>(length) >= target_capacity) return false;
    char normalized[sd::kFilePathCapacity] = {};
    if (sd::normalize_path(target, normalized, sizeof(normalized)) != sd::FileResult::Ok) return false;
    std::strcpy(target, normalized);
    return true;
}

bool receive_multipart_header(httpd_req_t* const request, char* const header,
                              std::size_t& header_length, std::size_t& unread_length,
                              MultipartUploadInfo& info) {
    header_length = 0;
    unread_length = static_cast<std::size_t>(request->content_len);
    header[0] = '\0';
    while (unread_length > 0 && header_length + 1 < kMultipartHeaderCapacity) {
        const std::size_t free_length = kMultipartHeaderCapacity - 1 - header_length;
        const std::size_t request_length = std::min(unread_length, free_length);
        const int received = httpd_req_recv(request, header + header_length, request_length);
        if (received <= 0) return false;
        const std::size_t received_length = static_cast<std::size_t>(received);
        header_length += received_length;
        unread_length -= received_length;
        header[header_length] = '\0';
        if (parse_multipart_upload_info(header, info)) return true;
    }
    return false;
}

bool write_upload_bytes(httpd_req_t* const request, sd::FileUploadSession& session,
                        char* const transfer_buffer, std::size_t& unread_length,
                        std::size_t file_length) {
    if (file_length > unread_length) return false;
    while (file_length > 0) {
        const std::size_t request_length = std::min(file_length, sizeof(transfer_buffer));
        const int received = httpd_req_recv(request, transfer_buffer, request_length);
        if (received <= 0) return false;
        const std::size_t received_length = static_cast<std::size_t>(received);
        if (received_length > unread_length ||
            sd::file_upload_write(session, transfer_buffer, received_length) != sd::FileResult::Ok) {
            return false;
        }
        unread_length -= received_length;
        file_length -= received_length;
    }
    return true;
}

bool discard_upload_bytes(httpd_req_t* const request, char* const transfer_buffer,
                          std::size_t& unread_length) {
    while (unread_length > 0) {
        const std::size_t request_length = std::min(unread_length, sizeof(transfer_buffer));
        const int received = httpd_req_recv(request, transfer_buffer, request_length);
        if (received <= 0) return false;
        const std::size_t received_length = static_cast<std::size_t>(received);
        if (received_length > unread_length) return false;
        unread_length -= received_length;
    }
    return true;
}

esp_err_t upload_raw_post_handler(httpd_req_t* const request) {
    char requested_path[sd::kFilePathCapacity] = {};
    if (!receive_path_query(request, requested_path, sizeof(requested_path))) {
        return send_json_error(request, "400 Bad Request", "invalid-path");
    }
    sd::FileUploadSession session = {};
    const sd::FileResult begin_result = sd::file_upload_begin(requested_path, session);
    if (begin_result != sd::FileResult::Ok) return send_file_error(request, begin_result);

    char transfer_buffer[kFileTransferBufferCapacity] = {};
    std::size_t remaining = static_cast<std::size_t>(request->content_len);
    while (remaining > 0) {
        const int received = httpd_req_recv(
            request, transfer_buffer,
            remaining < sizeof(transfer_buffer)
                ? remaining
                : sizeof(transfer_buffer));
        if (received <= 0 ||
            sd::file_upload_write(session, transfer_buffer, static_cast<std::size_t>(received)) !=
                sd::FileResult::Ok) {
            sd::file_upload_abort(session);
            return send_json_error(request, "500 Internal Server Error", "upload-failed");
        }
        remaining -= static_cast<std::size_t>(received);
    }
    const sd::FileResult commit_result = sd::file_upload_commit(session);
    if (commit_result != sd::FileResult::Ok) return send_file_error(request, commit_result);
    return send_text(request, "{\"result\":\"ok\"}\n", "application/json");
}

esp_err_t upload_multipart_post_handler(httpd_req_t* const request) {
    char multipart_header[kMultipartHeaderCapacity] = {};
    std::size_t header_length = 0;
    std::size_t unread_length = 0;
    MultipartUploadInfo info = {};
    if (!receive_multipart_header(request, multipart_header, header_length, unread_length, info)) {
        return send_json_error(request, "400 Bad Request", "invalid-multipart-upload");
    }
    if (info.file_size > kMaximumUploadBytes || info.file_payload_offset > header_length) {
        return send_json_error(request, "413 Payload Too Large", "invalid-upload-length");
    }

    char target_path[sd::kFilePathCapacity] = {};
    if (!build_multipart_target_path(info, target_path, sizeof(target_path))) {
        return send_json_error(request, "400 Bad Request", "invalid-path");
    }

    sd::FileUploadSession session = {};
    const sd::FileResult begin_result = sd::file_upload_begin(target_path, session);
    if (begin_result != sd::FileResult::Ok) return send_file_error(request, begin_result);

    const std::size_t buffered_file_length =
        std::min(info.file_size, header_length - info.file_payload_offset);
    bool write_ok = true;
    if (buffered_file_length > 0) {
        write_ok = sd::file_upload_write(session, multipart_header + info.file_payload_offset,
                                         buffered_file_length) == sd::FileResult::Ok;
    }
    char transfer_buffer[kFileTransferBufferCapacity] = {};
    if (write_ok) {
        write_ok = write_upload_bytes(request, session, transfer_buffer, unread_length,
                                      info.file_size - buffered_file_length);
    }
    if (write_ok) write_ok = discard_upload_bytes(request, transfer_buffer, unread_length);
    if (!write_ok) {
        sd::file_upload_abort(session);
        return send_json_error(request, "500 Internal Server Error", "upload-failed");
    }

    const sd::FileResult commit_result = sd::file_upload_commit(session);
    if (commit_result != sd::FileResult::Ok) return send_file_error(request, commit_result);
    return send_text(request, "{\"result\":\"ok\"}\n", "application/json");
}

esp_err_t upload_post_handler(httpd_req_t* const request) {
    if (request == nullptr || request->content_len <= 0 ||
        static_cast<std::size_t>(request->content_len) > kMaximumUploadBytes) {
        return send_json_error(request, "413 Payload Too Large", "invalid-upload-length");
    }
    if (request_is_multipart_form(request)) return upload_multipart_post_handler(request);
    return upload_raw_post_handler(request);
}

esp_err_t files_delete_handler(httpd_req_t* const request) {
    char requested_path[sd::kFilePathCapacity] = {};
    if (!receive_path_query(request, requested_path, sizeof(requested_path))) {
        return send_json_error(request, "400 Bad Request", "invalid-path");
    }
    const sd::FileResult result = sd::file_delete(requested_path);
    if (result != sd::FileResult::Ok) return send_file_error(request, result);
    return send_text(request, "{\"result\":\"ok\"}\n", "application/json");
}

esp_err_t files_put_handler(httpd_req_t* const request) {
    char requested_path[sd::kFilePathCapacity] = {};
    if (!receive_path_query(request, requested_path, sizeof(requested_path)) || request->content_len != 0) {
        return send_json_error(request, "400 Bad Request", "invalid-directory-request");
    }
    const sd::FileResult result = sd::directory_create(requested_path);
    if (result != sd::FileResult::Ok) return send_file_error(request, result);
    return send_text(request, "{\"result\":\"ok\"}\n", "application/json");
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

const httpd_uri_t kFilesDeleteUri = {
    .uri = "/files",
    .method = HTTP_DELETE,
    .handler = files_delete_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kFilesPutUri = {
    .uri = "/files",
    .method = HTTP_PUT,
    .handler = files_put_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kUploadUri = {
    .uri = "/upload",
    .method = HTTP_POST,
    .handler = upload_post_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kUploadGetUri = {
    .uri = "/upload",
    .method = HTTP_GET,
    .handler = upload_get_handler,
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
    // The WebSocket service has an independent three-client pool on port
    // 8849. HTTP still needs sufficient room for the App's concurrent setup
    // requests on port 8848.
    config.max_open_sockets = kMaximumHttpSessions;
    // App startup issues several concurrent requests.  On an AP/STA radio a
    // TCP transmit window may be temporarily unavailable; retain the socket
    // long enough for lwIP to make it writable instead of failing a complete
    // JSON response with EAGAIN.
    config.backlog_conn = kMaximumHttpSessions;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    httpd_handle_t server = nullptr;
    esp_err_t result = ::httpd_start(&server, &config);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "start failed: %s", esp_err_to_name(result));
        return result;
    }
    result = httpd_register_uri_handler(server, &kRootUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kCommandUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kFilesUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kFilesDeleteUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kFilesPutUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kUploadUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kUploadGetUri);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "route registration failed: %s", esp_err_to_name(result));
        (void)::httpd_stop(server);
        return result;
    }

    s_server = server;
    ESP_LOGI(kLogTag, "started port=%u routes=/,/command,/files,/upload",
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
