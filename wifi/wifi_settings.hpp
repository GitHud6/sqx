#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace ray2::wifi {

constexpr std::size_t kStaSsidCapacity = 33;
constexpr std::size_t kStaPasswordCapacity = 65;

// Persisted desired radio mode. This does not describe the transient AP/STA
// link state, which remains owned by wifi_service at runtime.
enum class RadioMode : std::uint8_t {
    Ap = 0,
    Sta = 1,
    ApSta = 2,
};

struct WifiSettings {
    char ssid[kStaSsidCapacity] = {};
    char password[kStaPasswordCapacity] = {};
    RadioMode radio_mode = RadioMode::Ap;
};

// Creates the `ray2_wifi` NVS namespace defaults on first use. The namespace
// contains `sta_ssid`, `sta_password`, and `radio_mode`.
esp_err_t wifi_settings_init();
esp_err_t wifi_settings_load(WifiSettings& settings);

esp_err_t wifi_settings_store_sta_ssid(const char* ssid);
esp_err_t wifi_settings_store_sta_password(const char* password);
esp_err_t wifi_settings_store_radio_mode(RadioMode mode);
esp_err_t wifi_settings_clear_sta_credentials();

const char* wifi_radio_mode_text(RadioMode mode);

}  // namespace ray2::wifi
