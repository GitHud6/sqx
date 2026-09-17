#include "websocket_service.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_http_server.h"
#include "esp_log.h"

#include "command_dispatch.hpp"
#include "network_command_adapter.hpp"
#include "runtime_affinity.hpp"
#include "wifi_service.hpp"

namespace ray2::websocket {
namespace {

constexpr char kLogTag[] = "websocket";
constexpr std::uint16_t kServerPort = 8849;
constexpr std::uint16_t kControlPort = 32769;
constexpr std::size_t kMaximumClients = 3;
constexpr std::size_t kMaximumInboundFrameBytes = 1024;
constexpr std::size_t kMaximumOutboundMessageBytes = 1600;
constexpr std::size_t kOutboundQueueLength = 32;
constexpr std::size_t kStatusMessageCapacity = 256;
constexpr std::size_t kNormalMessagesPerStatus = 4;
constexpr int kBroadcastTarget = -1;

struct OutboundMessage {
    int target_fd = kBroadcastTarget;
    httpd_ws_type_t type = HTTPD_WS_TYPE_BINARY;
    std::size_t length = 0;
    std::uint8_t data[kMaximumOutboundMessageBytes] = {};
};

struct PendingStatus {
    std::size_t length = 0;
    std::uint8_t data[kStatusMessageCapacity] = {};
    bool pending = false;
};

httpd_handle_t s_server = nullptr;
std::array<int, kMaximumClients> s_client_fds = {-1, -1, -1};
int s_active_client_fd = -1;
QueueHandle_t s_free_messages = nullptr;
QueueHandle_t s_ready_messages = nullptr;
TaskHandle_t s_tx_task = nullptr;
OutboundMessage s_messages[kOutboundQueueLength] = {};
portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
PendingStatus s_pending_status = {};

void remove_client(const int fd) {
    for (int& client_fd : s_client_fds) {
        if (client_fd == fd) client_fd = -1;
    }
    if (s_active_client_fd == fd) {
        s_active_client_fd = -1;
        for (const int client_fd : s_client_fds) {
            if (client_fd >= 0) {
                s_active_client_fd = client_fd;
                break;
            }
        }
    }
}

void prune_disconnected_clients() {
    if (s_server == nullptr) return;
    for (const int client_fd : s_client_fds) {
        if (client_fd >= 0 &&
            httpd_ws_get_fd_info(s_server, client_fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            remove_client(client_fd);
        }
    }
}

bool add_client(const int fd) {
    prune_disconnected_clients();
    for (const int client_fd : s_client_fds) {
        if (client_fd == fd) return true;
    }
    for (int& client_fd : s_client_fds) {
        if (client_fd < 0) {
            client_fd = fd;
            if (s_active_client_fd < 0) s_active_client_fd = fd;
            return true;
        }
    }
    return false;
}

void send_message_work(void* const user_data) {
    auto* const message = static_cast<OutboundMessage*>(user_data);
    if (message == nullptr) return;

    if (s_server != nullptr) {
        prune_disconnected_clients();
        httpd_ws_frame_t frame = {};
        frame.type = message->type;
        frame.payload = message->data;
        frame.len = message->length;
        if (message->target_fd >= 0) {
            if (httpd_ws_get_fd_info(s_server, message->target_fd) == HTTPD_WS_CLIENT_WEBSOCKET &&
                httpd_ws_send_frame_async(s_server, message->target_fd, &frame) != ESP_OK) {
                ESP_LOGW(kLogTag, "send failed fd=%d", message->target_fd);
                remove_client(message->target_fd);
            }
        } else {
            for (const int client_fd : s_client_fds) {
                if (client_fd < 0) continue;
                if (httpd_ws_send_frame_async(s_server, client_fd, &frame) != ESP_OK) {
                    ESP_LOGW(kLogTag, "send failed fd=%d", client_fd);
                    remove_client(client_fd);
                }
            }
        }
    }

    if (s_free_messages != nullptr) (void)xQueueSend(s_free_messages, &message, 0);
    if (s_tx_task != nullptr) xTaskNotifyGive(s_tx_task);
}

bool queue_message(const int target_fd, const httpd_ws_type_t type, const std::uint8_t* const data,
                   const std::size_t length) {
    if (data == nullptr || length == 0 || length > kMaximumOutboundMessageBytes ||
        s_free_messages == nullptr || s_ready_messages == nullptr || s_tx_task == nullptr) {
        return false;
    }
    OutboundMessage* message = nullptr;
    if (xQueueReceive(s_free_messages, &message, 0) != pdTRUE) return false;
    message->target_fd = target_fd;
    message->type = type;
    message->length = length;
    std::memcpy(message->data, data, length);
    if (xQueueSend(s_ready_messages, &message, 0) != pdTRUE) {
        (void)xQueueSend(s_free_messages, &message, 0);
        return false;
    }
    xTaskNotifyGive(s_tx_task);
    return true;
}

bool take_pending_status(OutboundMessage& message) {
    portENTER_CRITICAL(&s_status_lock);
    if (!s_pending_status.pending) {
        portEXIT_CRITICAL(&s_status_lock);
        return false;
    }
    message.target_fd = kBroadcastTarget;
    message.type = HTTPD_WS_TYPE_BINARY;
    message.length = s_pending_status.length;
    std::memcpy(message.data, s_pending_status.data, message.length);
    s_pending_status.pending = false;
    s_pending_status.length = 0;
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}

bool queue_work(OutboundMessage* const message) {
    return s_server != nullptr && message != nullptr &&
           httpd_queue_work(s_server, send_message_work, message) == ESP_OK;
}

void websocket_tx_task(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_server == nullptr) {
            OutboundMessage* message = nullptr;
            while (xQueueReceive(s_ready_messages, &message, 0) == pdTRUE) {
                (void)xQueueSend(s_free_messages, &message, 0);
            }
            portENTER_CRITICAL(&s_status_lock);
            s_pending_status.pending = false;
            s_pending_status.length = 0;
            portEXIT_CRITICAL(&s_status_lock);
            continue;
        }
        while (true) {
            bool submitted = false;
            for (std::size_t index = 0; index < kNormalMessagesPerStatus; ++index) {
                OutboundMessage* message = nullptr;
                if (xQueueReceive(s_ready_messages, &message, 0) != pdTRUE) break;
                if (!queue_work(message)) {
                    (void)xQueueSendToFront(s_ready_messages, &message, 0);
                    vTaskDelay(1);
                    xTaskNotifyGive(s_tx_task);
                    break;
                }
                submitted = true;
            }

            OutboundMessage* status_message = nullptr;
            if (xQueueReceive(s_free_messages, &status_message, 0) == pdTRUE) {
                if (take_pending_status(*status_message)) {
                    if (!queue_work(status_message)) {
                        (void)xQueueSend(s_free_messages, &status_message, 0);
                    } else {
                        submitted = true;
                    }
                } else {
                    (void)xQueueSend(s_free_messages, &status_message, 0);
                }
            }
            if (!submitted || uxQueueMessagesWaiting(s_ready_messages) == 0) break;
        }
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

bool normalize_command(char* const text, std::size_t& length) {
    if (text == nullptr || length == 0) return false;
    std::size_t first = 0;
    while (first < length && std::isspace(static_cast<unsigned char>(text[first])) != 0) ++first;
    while (length > first && std::isspace(static_cast<unsigned char>(text[length - 1])) != 0) --length;
    if (first == length) return false;
    if (first != 0) std::memmove(text, text + first, length - first);
    length -= first;
    text[length] = '\0';
    return true;
}

void send_submit_error(const int fd, const network::SubmitResult result) {
    const char* response = "error:network-input-unavailable\r\n";
    if (result == network::SubmitResult::Busy) response = "error:network-input-busy\r\n";
    if (result == network::SubmitResult::Invalid) response = "error:invalid-command\r\n";
    (void)queue_message(fd, HTTPD_WS_TYPE_BINARY, reinterpret_cast<const std::uint8_t*>(response),
                        std::strlen(response));
}

void submit_to_protocol(const int fd, const char* const text, const std::size_t length,
                        const bool command_text) {
    const network::SubmitResult result = network::network_command_adapter_submit(text, length, command_text);
    if (result != network::SubmitResult::Accepted) send_submit_error(fd, result);
}

void send_client_bootstrap(const int fd) {
    char status[256] = {};
    std::size_t status_length = 0;
    if (protocol::command_format_realtime_status(status, sizeof(status), status_length)) {
        (void)queue_message(fd, HTTPD_WS_TYPE_BINARY, reinterpret_cast<const std::uint8_t*>(status),
                            status_length);
    }
    char build_info[512] = {};
    std::size_t build_info_length = 0;
    if (protocol::command_format_build_info(build_info, sizeof(build_info), build_info_length)) {
        (void)queue_message(fd, HTTPD_WS_TYPE_BINARY, reinterpret_cast<const std::uint8_t*>(build_info),
                            build_info_length);
    }
    char current_id[32] = {};
    const int current_length = std::snprintf(current_id, sizeof(current_id), "CURRENT_ID:%d", fd);
    if (current_length > 0 && static_cast<std::size_t>(current_length) < sizeof(current_id)) {
        (void)queue_message(fd, HTTPD_WS_TYPE_TEXT, reinterpret_cast<const std::uint8_t*>(current_id),
                            static_cast<std::size_t>(current_length));
    }
    char active_id[32] = {};
    const int active_length = std::snprintf(active_id, sizeof(active_id), "ACTIVE_ID:%d", s_active_client_fd);
    if (active_length > 0 && static_cast<std::size_t>(active_length) < sizeof(active_id)) {
        (void)queue_message(fd, HTTPD_WS_TYPE_TEXT, reinterpret_cast<const std::uint8_t*>(active_id),
                            static_cast<std::size_t>(active_length));
    }
}

void process_inbound_command(const int fd, char* const command, std::size_t length) {
    if (!normalize_command(command, length)) {
        static constexpr char kError[] = "error:invalid-command\r\n";
        (void)queue_message(fd, HTTPD_WS_TYPE_BINARY, reinterpret_cast<const std::uint8_t*>(kError),
                            sizeof(kError) - 1U);
        return;
    }
    if (length == 1 && command[0] == '?') {
        submit_to_protocol(fd, command, length, false);
        return;
    }
    if (equals_ignore_case(command, length, "$I")) {
        submit_to_protocol(fd, command, length, false);
        return;
    }
    if (equals_ignore_case(command, length, "req")) {
        static constexpr char kStatus[] = "?";
        static constexpr char kBuildInfo[] = "$I";
        submit_to_protocol(fd, kStatus, sizeof(kStatus) - 1U, false);
        submit_to_protocol(fd, kBuildInfo, sizeof(kBuildInfo) - 1U, false);
        return;
    }
    if (equals_ignore_case(command, length, "$WiFi/ListAPs")) {
        static constexpr char kResponse[] = "{\"AP_LIST\":[]}ok\r\n";
        (void)queue_message(fd, HTTPD_WS_TYPE_BINARY, reinterpret_cast<const std::uint8_t*>(kResponse),
                            sizeof(kResponse) - 1U);
        return;
    }
    if (length == std::strlen("{\"type\":\"ping\"}") &&
        std::memcmp(command, "{\"type\":\"ping\"}", length) == 0) {
        static constexpr char kPong[] = "{\"type\":\"pong\"}";
        (void)queue_message(fd, HTTPD_WS_TYPE_TEXT, reinterpret_cast<const std::uint8_t*>(kPong),
                            sizeof(kPong) - 1U);
        return;
    }
    submit_to_protocol(fd, command, length, true);
}

esp_err_t websocket_handler(httpd_req_t* const request) {
    if (request == nullptr) return ESP_ERR_INVALID_ARG;
    const int client_fd = httpd_req_to_sockfd(request);
    if (request->method == HTTP_GET) {
        if (!add_client(client_fd)) {
            ESP_LOGW(kLogTag, "connection rejected fd=%d clients=%u", client_fd,
                     static_cast<unsigned>(kMaximumClients));
            (void)httpd_sess_trigger_close(s_server, client_fd);
            return ESP_OK;
        }
        send_client_bootstrap(client_fd);
        ESP_LOGI(kLogTag, "connected fd=%d", client_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {};
    esp_err_t result = httpd_ws_recv_frame(request, &frame, 0);
    if (result != ESP_OK) {
        remove_client(client_fd);
        return result;
    }
    if (frame.len == 0) return ESP_OK;
    if (frame.len > kMaximumInboundFrameBytes) {
        ESP_LOGW(kLogTag, "inbound frame too large fd=%d len=%u", client_fd,
                 static_cast<unsigned>(frame.len));
        (void)httpd_sess_trigger_close(s_server, client_fd);
        remove_client(client_fd);
        return ESP_ERR_INVALID_SIZE;
    }
    std::array<std::uint8_t, kMaximumInboundFrameBytes + 1U> payload = {};
    frame.payload = payload.data();
    result = httpd_ws_recv_frame(request, &frame, kMaximumInboundFrameBytes);
    if (result != ESP_OK) {
        remove_client(client_fd);
        return result;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT && frame.type != HTTPD_WS_TYPE_BINARY) return ESP_OK;
    if (std::memchr(payload.data(), '\0', frame.len) != nullptr) {
        static constexpr char kError[] = "error:invalid-command\r\n";
        (void)queue_message(client_fd, HTTPD_WS_TYPE_BINARY,
                            reinterpret_cast<const std::uint8_t*>(kError), sizeof(kError) - 1U);
        return ESP_OK;
    }
    payload[frame.len] = '\0';
    process_inbound_command(client_fd, reinterpret_cast<char*>(payload.data()), frame.len);
    return ESP_OK;
}

const httpd_uri_t kWebsocketUri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = websocket_handler,
    .user_ctx = nullptr,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = nullptr,
};

}  // namespace

esp_err_t websocket_service_start() {
    if (s_server != nullptr) return ESP_OK;
    if (!wifi::wifi_is_started()) return ESP_ERR_INVALID_STATE;
    if (s_tx_task == nullptr) {
        s_free_messages = xQueueCreate(kOutboundQueueLength, sizeof(OutboundMessage*));
        s_ready_messages = xQueueCreate(kOutboundQueueLength, sizeof(OutboundMessage*));
        if (s_free_messages == nullptr || s_ready_messages == nullptr) return ESP_ERR_NO_MEM;
        for (std::size_t index = 0; index < kOutboundQueueLength; ++index) {
            OutboundMessage* message = &s_messages[index];
            (void)xQueueSend(s_free_messages, &message, 0);
        }
        if (xTaskCreatePinnedToCore(websocket_tx_task, "websocket_tx", 4096, nullptr, 3, &s_tx_task,
                                    runtime::kNonRealtimeCore) != pdPASS) {
            vQueueDelete(s_ready_messages);
            vQueueDelete(s_free_messages);
            s_ready_messages = nullptr;
            s_free_messages = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kServerPort;
    config.ctrl_port = kControlPort;
    config.max_open_sockets = kMaximumClients + 1;
    config.stack_size = 6144;
    config.task_priority = 4;
    httpd_handle_t server = nullptr;
    esp_err_t result = httpd_start(&server, &config);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "start failed: %s", esp_err_to_name(result));
        return result;
    }
    result = httpd_register_uri_handler(server, &kWebsocketUri);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "route registration failed: %s", esp_err_to_name(result));
        (void)httpd_stop(server);
        return result;
    }
    s_client_fds.fill(-1);
    s_active_client_fd = -1;
    s_server = server;
    ESP_LOGI(kLogTag, "started port=%u clients=%u", static_cast<unsigned>(kServerPort),
             static_cast<unsigned>(kMaximumClients));
    return ESP_OK;
}

esp_err_t websocket_service_stop() {
    if (s_server == nullptr) return ESP_OK;
    const esp_err_t result = httpd_stop(s_server);
    if (result == ESP_OK) {
        s_server = nullptr;
        s_client_fds.fill(-1);
        s_active_client_fd = -1;
        ESP_LOGI(kLogTag, "stopped");
    }
    return result;
}

bool websocket_service_is_started() {
    return s_server != nullptr;
}

esp_err_t websocket_service_publish_text(const char* const text, const std::size_t length) {
    return queue_message(kBroadcastTarget, HTTPD_WS_TYPE_TEXT,
                         reinterpret_cast<const std::uint8_t*>(text), length)
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

esp_err_t websocket_service_publish_binary(const std::uint8_t* const data, const std::size_t length) {
    return queue_message(kBroadcastTarget, HTTPD_WS_TYPE_BINARY, data, length) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t websocket_service_publish_binary_to_client(const int client_fd, const std::uint8_t* const data,
                                                      const std::size_t length) {
    if (client_fd < 0) return ESP_ERR_INVALID_ARG;
    return queue_message(client_fd, HTTPD_WS_TYPE_BINARY, data, length) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool websocket_service_publish_realtime_status(const std::uint8_t* const data, const std::size_t length) {
    if (data == nullptr || length == 0 || length > kStatusMessageCapacity || s_tx_task == nullptr) return false;
    portENTER_CRITICAL(&s_status_lock);
    std::memcpy(s_pending_status.data, data, length);
    s_pending_status.length = length;
    s_pending_status.pending = true;
    portEXIT_CRITICAL(&s_status_lock);
    xTaskNotifyGive(s_tx_task);
    return true;
}

}  // namespace ray2::websocket
