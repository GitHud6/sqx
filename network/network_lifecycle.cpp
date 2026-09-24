#include "network_lifecycle.hpp"

#include <atomic>
#include <cstdint>

#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "httpd_service.hpp"
#include "runtime_affinity.hpp"
#include "websocket_service.hpp"
#include "wifi_service.hpp"

namespace laserctl::network {
namespace {

constexpr char kLogTag[] = "network_lifecycle";
// This task owns HTTPD and WebSocket stop/start transitions. Their synchronous
// lifecycle calls require the original 4 KiB stack budget.
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::uint32_t kNotifyStaDisconnected = 1U << 0U;
constexpr std::uint32_t kNotifyStaGotIp = 1U << 1U;
constexpr std::uint32_t kNotifyWifiRestart = 1U << 2U;
constexpr TickType_t kWebSocketStopTimeout = pdMS_TO_TICKS(2000);
constexpr TickType_t kWebSocketStopPoll = pdMS_TO_TICKS(10);

TaskHandle_t s_task = nullptr;
std::atomic<bool> s_started = false;

void log_stack_watermark(const char* const event) {
    ESP_LOGI(kLogTag, "stack event=%s remaining=%u", event,
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
}

void notify(const std::uint32_t bits) {
    const TaskHandle_t task = s_task;
    if (task != nullptr) {
        (void)xTaskNotify(task, bits, eSetBits);
    }
}

void on_wifi_link_event(const wifi::LinkEvent event, void*) {
    switch (event) {
        case wifi::LinkEvent::StaDisconnected: notify(kNotifyStaDisconnected); break;
        case wifi::LinkEvent::StaGotIp: notify(kNotifyStaGotIp); break;
    }
}

bool wait_for_websocket_stop() {
    const TickType_t deadline = xTaskGetTickCount() + kWebSocketStopTimeout;
    while (websocket::websocket_service_is_started()) {
        if (static_cast<std::int32_t>(xTaskGetTickCount() - deadline) >= 0) {
            ESP_LOGE(kLogTag, "WebSocket stop timed out");
            return false;
        }
        vTaskDelay(kWebSocketStopPoll);
    }
    return true;
}

bool stop_transports() {
    const esp_err_t websocket_result = websocket::websocket_service_stop();
    if (websocket_result != ESP_OK) {
        ESP_LOGE(kLogTag, "WebSocket stop failed: %s", esp_err_to_name(websocket_result));
        return false;
    }
    if (!wait_for_websocket_stop()) return false;

    const esp_err_t httpd_result = httpd::httpd_service_stop();
    if (httpd_result != ESP_OK) {
        ESP_LOGE(kLogTag, "HTTP stop failed: %s", esp_err_to_name(httpd_result));
        return false;
    }
    return true;
}

bool start_transports() {
    if (!httpd::httpd_service_is_started()) {
        const esp_err_t httpd_result = httpd::httpd_service_start();
        if (httpd_result != ESP_OK) {
            ESP_LOGE(kLogTag, "HTTP start failed: %s", esp_err_to_name(httpd_result));
            return false;
        }
    }
    if (!websocket::websocket_service_is_started()) {
        const esp_err_t websocket_result = websocket::websocket_service_start();
        if (websocket_result != ESP_OK) {
            ESP_LOGE(kLogTag, "WebSocket start failed: %s", esp_err_to_name(websocket_result));
            return false;
        }
    }
    return true;
}

void start_if_link_available() {
    wifi::RuntimeInfo info = {};
    if (!wifi::wifi_get_runtime_info(info)) {
        ESP_LOGW(kLogTag, "Wi-Fi runtime state unavailable");
        return;
    }
    if (!info.ap_enabled && !info.sta_connected) return;
    if (start_transports()) {
        ESP_LOGI(kLogTag, "transports ready ap=%u sta=%u ip=%s", info.ap_enabled ? 1U : 0U,
                 info.sta_connected ? 1U : 0U,
                 info.sta_connected ? info.sta_ip : info.ap_ip);
        log_stack_watermark("transports-ready");
    }
}

void handle_sta_disconnect() {
    // A DHCP/association loss invalidates every old STA TCP session. Rebuild
    // both listeners even in AP+STA mode so no transport retains stale lwIP
    // socket state while the station interface acquires its next address.
    if (!stop_transports()) return;

    // SoftAP has its own static interface and can immediately serve recovery
    // traffic while STA reconnects. Pure STA waits for STA_GOT_IP below.
    wifi::RuntimeInfo info = {};
    if (wifi::wifi_get_runtime_info(info) && info.ap_enabled) {
        start_if_link_available();
    }

    const esp_err_t result = wifi::wifi_request_sta_connect();
    if (result != ESP_OK && result != ESP_ERR_WIFI_STATE) {
        ESP_LOGW(kLogTag, "STA reconnect request failed: %s", esp_err_to_name(result));
    }
    log_stack_watermark("sta-disconnect");
}

void handle_wifi_restart() {
    if (!stop_transports()) return;
    const esp_err_t result = wifi::wifi_restart();
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "Wi-Fi restart failed: %s", esp_err_to_name(result));
        return;
    }
    start_if_link_available();
    log_stack_watermark("wifi-restart");
}

void network_lifecycle_task(void*) {
    while (true) {
        std::uint32_t events = 0;
        (void)xTaskNotifyWait(0, 0xffffffffU, &events, portMAX_DELAY);

        if ((events & kNotifyWifiRestart) != 0U) {
            handle_wifi_restart();
            continue;
        }
        if ((events & kNotifyStaDisconnected) != 0U) {
            handle_sta_disconnect();
        }
        if ((events & kNotifyStaGotIp) != 0U) {
            start_if_link_available();
        }
    }
}

}  // namespace

esp_err_t network_lifecycle_start() {
    if (s_task != nullptr) return ESP_ERR_INVALID_STATE;
    if (!wifi::wifi_is_started()) return ESP_ERR_INVALID_STATE;

    if (xTaskCreatePinnedToCore(network_lifecycle_task, "network_lifecycle", kTaskStackBytes,
                                nullptr, 4, &s_task, runtime::kNonRealtimeCore) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    wifi::wifi_set_link_event_callback(on_wifi_link_event, nullptr);
    s_started.store(true, std::memory_order_release);
    // Initial radio startup may have completed before the callback was
    // installed. Route the initial reconciliation through this task as well.
    notify(kNotifyStaGotIp);
    ESP_LOGI(kLogTag, "started");
    return ESP_OK;
}

bool network_lifecycle_is_started() {
    return s_started.load(std::memory_order_acquire);
}

esp_err_t network_lifecycle_request_wifi_restart() {
    if (s_task == nullptr) return ESP_ERR_INVALID_STATE;
    notify(kNotifyWifiRestart);
    return ESP_OK;
}

}  // namespace laserctl::network
