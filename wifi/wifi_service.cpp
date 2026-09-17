#include "wifi_service.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"

#include "wifi_settings.hpp"

namespace ray2::wifi {
namespace {

constexpr char kLogTag[] = "wifi";
constexpr char kApPassword[] = "12345678";
constexpr char kApIp[] = "192.168.0.1";
constexpr char kApNetmask[] = "255.255.255.0";
constexpr char kApSsidPrefix[] = "LongerLaser";
constexpr std::size_t kSsidCapacity = kWifiSsidCapacity;
constexpr std::size_t kIpCapacity = kWifiIpTextCapacity;

std::atomic<State> s_state = State::Stopped;
std::atomic<bool> s_started = false;
std::atomic<bool> s_sta_has_ip = false;
esp_netif_t* s_ap_netif = nullptr;
esp_netif_t* s_sta_netif = nullptr;
esp_event_handler_instance_t s_wifi_handler = nullptr;
esp_event_handler_instance_t s_ip_handler = nullptr;
char s_ap_ssid[kSsidCapacity] = {};

bool mode_requests_sta(const RadioMode mode) {
    return mode == RadioMode::Sta || mode == RadioMode::ApSta;
}

bool mode_requests_ap(const RadioMode mode) {
    return mode == RadioMode::Ap || mode == RadioMode::ApSta;
}

wifi_mode_t esp_mode_from_radio_mode(const RadioMode mode) {
    switch (mode) {
        case RadioMode::Ap: return WIFI_MODE_AP;
        case RadioMode::Sta: return WIFI_MODE_STA;
        case RadioMode::ApSta: return WIFI_MODE_APSTA;
    }
    return WIFI_MODE_NULL;
}

bool esp_mode_has_ap(const wifi_mode_t mode) {
    return mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA;
}

bool esp_mode_has_sta(const wifi_mode_t mode) {
    return mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA;
}

void copy_wifi_text(const std::uint8_t* source, const std::size_t source_capacity, char* target,
                    const std::size_t target_capacity) {
    if (source == nullptr || target == nullptr || target_capacity == 0) return;
    std::size_t length = 0;
    while (length < source_capacity && source[length] != 0 && length + 1 < target_capacity) {
        target[length] = static_cast<char>(source[length]);
        ++length;
    }
    target[length] = '\0';
}

void format_mac(const wifi_interface_t interface, char* buffer, const std::size_t capacity) {
    uint8_t mac[6] = {};
    if (buffer == nullptr || capacity < kWifiMacTextCapacity ||
        esp_wifi_get_mac(interface, mac) != ESP_OK) {
        return;
    }
    (void)std::snprintf(buffer, capacity, "%02X-%02X-%02X-%02X-%02X-%02X",
                        static_cast<unsigned>(mac[0]), static_cast<unsigned>(mac[1]),
                        static_cast<unsigned>(mac[2]), static_cast<unsigned>(mac[3]),
                        static_cast<unsigned>(mac[4]), static_cast<unsigned>(mac[5]));
}

void format_ip(esp_netif_t* netif, char* buffer, const std::size_t capacity) {
    if (netif == nullptr || buffer == nullptr || capacity < kIpCapacity) return;
    esp_netif_ip_info_t ip_info = {};
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) return;
    const std::uint32_t address = ntohl(ip_info.ip.addr);
    (void)std::snprintf(buffer, capacity, "%u.%u.%u.%u",
                        static_cast<unsigned>((address >> 24U) & 0xffU),
                        static_cast<unsigned>((address >> 16U) & 0xffU),
                        static_cast<unsigned>((address >> 8U) & 0xffU),
                        static_cast<unsigned>(address & 0xffU));
}

void set_ap_static_ip() {
    if (s_ap_netif == nullptr) return;

    esp_err_t result = esp_netif_dhcps_stop(s_ap_netif);
    if (result != ESP_OK && result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(kLogTag, "stop AP DHCP failed: %s", esp_err_to_name(result));
        return;
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = inet_addr(kApIp);
    ip_info.gw.addr = inet_addr(kApIp);
    ip_info.netmask.addr = inet_addr(kApNetmask);
    result = esp_netif_set_ip_info(s_ap_netif, &ip_info);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "set AP IP failed: %s", esp_err_to_name(result));
        (void)esp_netif_dhcps_start(s_ap_netif);
        return;
    }

    result = esp_netif_dhcps_start(s_ap_netif);
    if (result != ESP_OK && result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGW(kLogTag, "start AP DHCP failed: %s", esp_err_to_name(result));
        return;
    }
}

void build_ap_ssid() {
    uint8_t mac[6] = {};
    if (esp_wifi_get_mac(WIFI_IF_AP, mac) == ESP_OK) {
        std::snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s_%02X%02X", kApSsidPrefix,
                      static_cast<unsigned>(mac[4]), static_cast<unsigned>(mac[5]));
    } else {
        std::snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s", "LongerLaser_0000");
    }
}

esp_err_t configure_ap() {
    build_ap_ssid();
    wifi_config_t ap_config = {};
    std::memcpy(ap_config.ap.ssid, s_ap_ssid, std::strlen(s_ap_ssid));
    ap_config.ap.ssid_len = static_cast<uint8_t>(std::strlen(s_ap_ssid));
    std::memcpy(ap_config.ap.password, kApPassword, sizeof(kApPassword));
    ap_config.ap.channel = 11;
    ap_config.ap.max_connection = 3;
    ap_config.ap.beacon_interval = 100;
    ap_config.ap.dtim_period = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    return esp_wifi_set_config(WIFI_IF_AP, &ap_config);
}

esp_err_t configure_sta(const WifiSettings& settings) {
    wifi_config_t sta_config = {};
    std::memcpy(sta_config.sta.ssid, settings.ssid, std::strlen(settings.ssid));
    std::memcpy(sta_config.sta.password, settings.password, std::strlen(settings.password));
    sta_config.sta.scan_method = WIFI_FAST_SCAN;
    sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_config.sta.failure_retry_cnt = 3;
    return esp_wifi_set_config(WIFI_IF_STA, &sta_config);
}

void wifi_event_handler(void*, esp_event_base_t event_base, int32_t event_id, void*) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_sta_has_ip.store(true, std::memory_order_release);
        s_state.store(State::StaConnected, std::memory_order_release);
        ESP_LOGI(kLogTag, "STA connected");
        return;
    }
    if (event_base != WIFI_EVENT) return;

    switch (event_id) {
        case WIFI_EVENT_AP_START:
            s_state.store(State::ApReady, std::memory_order_release);
            ESP_LOGI(kLogTag, "SoftAP ready ssid=%s ip=%s", s_ap_ssid, kApIp);
            break;
        case WIFI_EVENT_AP_STOP:
            s_state.store(State::Stopped, std::memory_order_release);
            ESP_LOGW(kLogTag, "SoftAP stopped");
            break;
        case WIFI_EVENT_STA_START:
            s_state.store(State::StaConnecting, std::memory_order_release);
            ESP_LOGI(kLogTag, "STA connecting");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_sta_has_ip.store(false, std::memory_order_release);
            s_state.store(State::StaConnecting, std::memory_order_release);
            ESP_LOGW(kLogTag, "STA disconnected");
            break;
        default:
            break;
    }
}

}  // namespace

esp_err_t wifi_start() {
    if (s_started.load(std::memory_order_acquire)) return ESP_OK;
    s_state.store(State::Starting, std::memory_order_release);
    s_sta_has_ip.store(false, std::memory_order_release);

    WifiSettings saved_settings = {};
    esp_err_t result = wifi_settings_load(saved_settings);
    if (result != ESP_OK) {
        s_state.store(State::Error, std::memory_order_release);
        return result;
    }
    ESP_LOGI(kLogTag, "starting saved mode=%s sta_ssid=%s",
             wifi_radio_mode_text(saved_settings.radio_mode),
             saved_settings.ssid[0] == '\0' ? "<unset>" : "<set>");

    result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        s_state.store(State::Error, std::memory_order_release);
        return result;
    }
    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        s_state.store(State::Error, std::memory_order_release);
        return result;
    }

    if (mode_requests_ap(saved_settings.radio_mode) && s_ap_netif == nullptr) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == nullptr) {
            s_state.store(State::Error, std::memory_order_release);
            return ESP_ERR_NO_MEM;
        }
    }
    if (mode_requests_sta(saved_settings.radio_mode) && s_sta_netif == nullptr) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (s_sta_netif == nullptr) {
            s_state.store(State::Error, std::memory_order_release);
            return ESP_ERR_NO_MEM;
        }
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&init_config);
    if (result != ESP_OK && result != ESP_ERR_WIFI_INIT_STATE) {
        s_state.store(State::Error, std::memory_order_release);
        return result;
    }
    if (s_wifi_handler == nullptr) {
        result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                     &wifi_event_handler, nullptr,
                                                     &s_wifi_handler);
        if (result != ESP_OK) {
            s_state.store(State::Error, std::memory_order_release);
            return result;
        }
    }
    if (s_ip_handler == nullptr) {
        result = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                     &wifi_event_handler, nullptr, &s_ip_handler);
        if (result != ESP_OK) {
            s_state.store(State::Error, std::memory_order_release);
            return result;
        }
    }

    result = esp_wifi_set_mode(esp_mode_from_radio_mode(saved_settings.radio_mode));
    if (result == ESP_OK && mode_requests_ap(saved_settings.radio_mode)) {
        set_ap_static_ip();
        result = configure_ap();
    }
    if (result == ESP_OK && mode_requests_sta(saved_settings.radio_mode) &&
        saved_settings.ssid[0] != '\0') {
        result = configure_sta(saved_settings);
    }
    if (result == ESP_OK) result = esp_wifi_start();
    const bool driver_started = result == ESP_OK;
    if (result == ESP_OK) result = esp_wifi_set_ps(WIFI_PS_NONE);
    if (result != ESP_OK) {
        if (driver_started) (void)esp_wifi_stop();
        s_state.store(State::Error, std::memory_order_release);
        return result;
    }

    s_started.store(true, std::memory_order_release);

    if (mode_requests_sta(saved_settings.radio_mode)) {
        if (saved_settings.ssid[0] == '\0') {
            ESP_LOGW(kLogTag, "%s started without STA SSID", wifi_radio_mode_text(saved_settings.radio_mode));
        } else {
            result = esp_wifi_connect();
            if (result != ESP_OK) {
                ESP_LOGW(kLogTag, "%s STA connect request failed: %s",
                         wifi_radio_mode_text(saved_settings.radio_mode), esp_err_to_name(result));
            }
        }
    }

    ESP_LOGI(kLogTag, "started mode=%s ap_ssid=%s", wifi_radio_mode_text(saved_settings.radio_mode),
             mode_requests_ap(saved_settings.radio_mode) ? s_ap_ssid : "<disabled>");
    return ESP_OK;
}

esp_err_t wifi_stop() {
    if (!s_started.load(std::memory_order_acquire)) return ESP_OK;
    const esp_err_t result = esp_wifi_stop();
    if (result == ESP_OK || result == ESP_ERR_WIFI_NOT_INIT || result == ESP_ERR_WIFI_NOT_STARTED) {
        s_started.store(false, std::memory_order_release);
        s_sta_has_ip.store(false, std::memory_order_release);
        s_state.store(State::Stopped, std::memory_order_release);
    }
    return result;
}

esp_err_t wifi_restart() {
    const esp_err_t stop_result = wifi_stop();
    if (stop_result != ESP_OK) return stop_result;
    return wifi_start();
}

bool wifi_is_started() {
    return s_started.load(std::memory_order_acquire);
}

State wifi_state() {
    return s_state.load(std::memory_order_acquire);
}

const char* wifi_state_text(const State state) {
    switch (state) {
        case State::Stopped: return "stopped";
        case State::Starting: return "starting";
        case State::ApReady: return "ap-ready";
        case State::StaConnecting: return "sta-connecting";
        case State::StaConnected: return "sta-connected";
        case State::Error: return "error";
    }
    return "unknown";
}

bool wifi_get_ap_ssid(char* buffer, const std::size_t capacity) {
    if (buffer == nullptr || capacity == 0 || s_ap_ssid[0] == '\0') return false;
    const int written = std::snprintf(buffer, capacity, "%s", s_ap_ssid);
    return written >= 0 && static_cast<std::size_t>(written) < capacity;
}

bool wifi_get_ap_ip(char* buffer, const std::size_t capacity) {
    if (buffer == nullptr || capacity < kIpCapacity) return false;
    const int written = std::snprintf(buffer, capacity, "%s", kApIp);
    return written >= 0 && static_cast<std::size_t>(written) < capacity;
}

bool wifi_get_runtime_info(RuntimeInfo& info) {
    info = {};
    info.started = s_started.load(std::memory_order_acquire);
    if (!info.started) return true;

    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) != ESP_OK) return false;

    info.ap_enabled = esp_mode_has_ap(mode);
    info.sta_enabled = esp_mode_has_sta(mode);
    WifiSettings settings = {};
    if (wifi_settings_load(settings) == ESP_OK) {
        info.sta_connection_enabled = mode_requests_sta(settings.radio_mode);
    }
    if (info.ap_enabled) {
        (void)std::snprintf(info.ap_ssid, sizeof(info.ap_ssid), "%s", s_ap_ssid);
        format_ip(s_ap_netif, info.ap_ip, sizeof(info.ap_ip));
        format_mac(WIFI_IF_AP, info.ap_mac, sizeof(info.ap_mac));
    }
    if (info.sta_enabled) {
        wifi_config_t config = {};
        if (esp_wifi_get_config(WIFI_IF_STA, &config) == ESP_OK) {
            copy_wifi_text(config.sta.ssid, sizeof(config.sta.ssid), info.sta_ssid,
                           sizeof(info.sta_ssid));
        }
        const bool has_ip = s_sta_has_ip.load(std::memory_order_acquire);
        if (has_ip) format_ip(s_sta_netif, info.sta_ip, sizeof(info.sta_ip));
        format_mac(WIFI_IF_STA, info.sta_mac, sizeof(info.sta_mac));
        info.sta_connected = has_ip && info.sta_ip[0] != '\0' &&
                             std::strcmp(info.sta_ip, "0.0.0.0") != 0;
    }
    return true;
}

}  // namespace ray2::wifi
