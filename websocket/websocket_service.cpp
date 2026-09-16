#include "websocket_service.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "esp_http_server.h"
#include "esp_log.h"

#include "wifi_service.hpp"

namespace ray2::websocket {
namespace {

constexpr char kLogTag[] = "websocket";
constexpr std::uint16_t kServerPort = 8849;
constexpr std::uint16_t kControlPort = 32769;
constexpr std::size_t kMaximumClients = 3;
constexpr std::size_t kMaximumInboundFrameBytes = 128;
constexpr std::size_t kMaximumOutboundTextBytes = 2048;

struct PendingTextMessage {
    std::size_t length = 0;
    char text[kMaximumOutboundTextBytes] = {};
};

httpd_handle_t s_server = nullptr;
std::array<int, kMaximumClients> s_client_fds = {-1, -1, -1};

void remove_client(const int fd) {
    for (int& client_fd : s_client_fds) {
        if (client_fd == fd) {
            client_fd = -1;
            return;
        }
    }
}

void prune_disconnected_clients() {
    if (s_server == nullptr) return;
    for (int& client_fd : s_client_fds) {
        if (client_fd >= 0 &&
            httpd_ws_get_fd_info(s_server, client_fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            client_fd = -1;
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
            return true;
        }
    }
    return false;
}

void broadcast_text_work(void* const user_data) {
    auto* const message = static_cast<PendingTextMessage*>(user_data);
    if (message == nullptr) return;

    if (s_server != nullptr) {
        prune_disconnected_clients();
        httpd_ws_frame_t frame = {};
        frame.type = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<std::uint8_t*>(message->text);
        frame.len = message->length;
        for (const int client_fd : s_client_fds) {
            if (client_fd < 0) continue;
            if (httpd_ws_send_frame_async(s_server, client_fd, &frame) != ESP_OK) {
                ESP_LOGW(kLogTag, "send failed fd=%d", client_fd);
                remove_client(client_fd);
            }
        }
    }

    std::free(message);
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
        ESP_LOGI(kLogTag, "connected fd=%d", client_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {};
    esp_err_t result = httpd_ws_recv_frame(request, &frame, 0);
    if (result != ESP_OK) {
        remove_client(client_fd);
        return result;
    }
    if (frame.len > kMaximumInboundFrameBytes) {
        ESP_LOGW(kLogTag, "inbound frame too large fd=%d len=%u", client_fd,
                 static_cast<unsigned>(frame.len));
        remove_client(client_fd);
        (void)httpd_sess_trigger_close(s_server, client_fd);
        return ESP_ERR_INVALID_SIZE;
    }
    if (frame.len == 0) return ESP_OK;

    std::array<std::uint8_t, kMaximumInboundFrameBytes> discarded = {};
    frame.payload = discarded.data();
    result = httpd_ws_recv_frame(request, &frame, discarded.size());
    if (result != ESP_OK) remove_client(client_fd);
    return result;
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

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kServerPort;
    config.ctrl_port = kControlPort;
    config.max_open_sockets = kMaximumClients + 1;
    config.stack_size = 4096;
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
        ESP_LOGI(kLogTag, "stopped");
    }
    return result;
}

bool websocket_service_is_started() {
    return s_server != nullptr;
}

esp_err_t websocket_service_publish_text(const char* const text, const std::size_t length) {
    if (text == nullptr || length == 0 || length >= kMaximumOutboundTextBytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_server == nullptr) return ESP_ERR_INVALID_STATE;

    auto* const message = static_cast<PendingTextMessage*>(std::malloc(sizeof(PendingTextMessage)));
    if (message == nullptr) return ESP_ERR_NO_MEM;
    message->length = length;
    std::memcpy(message->text, text, length);
    message->text[length] = '\0';

    const esp_err_t result = httpd_queue_work(s_server, broadcast_text_work, message);
    if (result != ESP_OK) {
        std::free(message);
        ESP_LOGW(kLogTag, "broadcast dropped: %s", esp_err_to_name(result));
    }
    return result;
}

}  // namespace ray2::websocket
