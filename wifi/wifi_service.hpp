#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace laserctl::wifi {

constexpr std::size_t kWifiSsidCapacity = 33;
constexpr std::size_t kWifiIpTextCapacity = 16;
constexpr std::size_t kWifiMacTextCapacity = 18;

enum class State {
    Stopped,
    Starting,
    ApReady,
    StaConnecting,
    StaConnected,
    Error,
};

enum class LinkEvent : std::uint8_t {
    StaDisconnected,
    StaGotIp,
};

// Wi-Fi event handlers only publish link changes. The receiver must return
// promptly and must not start or stop socket services in event-loop context.
using LinkEventCallback = void (*)(LinkEvent event, void* user_data);

struct RuntimeInfo {
    bool started = false;
    bool ap_enabled = false;
    char ap_ssid[kWifiSsidCapacity] = {};
    char ap_ip[kWifiIpTextCapacity] = {};
    char ap_mac[kWifiMacTextCapacity] = {};
    bool sta_enabled = false;
    bool sta_connection_enabled = false;
    bool sta_connected = false;
    char sta_ssid[kWifiSsidCapacity] = {};
    char sta_ip[kWifiIpTextCapacity] = {};
    char sta_mac[kWifiMacTextCapacity] = {};
};

// Starts the saved AP, STA, or AP+STA radio configuration. The service only
// owns radio transport; it has no G-code or socket handling.
esp_err_t wifi_start();
esp_err_t wifi_stop();
esp_err_t wifi_restart();
// Requests a new association after a station disconnect. The caller runs in
// normal task context and owns retry policy.
esp_err_t wifi_request_sta_connect();
void wifi_set_link_event_callback(LinkEventCallback callback, void* user_data);
bool wifi_is_started();
State wifi_state();
const char* wifi_state_text(State state);

// Copies the active SoftAP identity for status and future discovery services.
bool wifi_get_ap_ssid(char* buffer, std::size_t capacity);
bool wifi_get_ap_ip(char* buffer, std::size_t capacity);
bool wifi_get_runtime_info(RuntimeInfo& info);

}  // namespace laserctl::wifi
