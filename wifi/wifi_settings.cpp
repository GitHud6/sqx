#include "wifi_settings.hpp"

#include <cstring>

#include "nvs.h"

namespace laserctl::wifi {
namespace {

constexpr char kNvsNamespace[] = "laserctl_wifi";
constexpr char kNvsKeyStaSsid[] = "sta_ssid";
constexpr char kNvsKeyStaPassword[] = "sta_password";
constexpr char kNvsKeyRadioMode[] = "radio_mode";
constexpr char kNvsKeyLegacyStatus[] = "wifi_status";

bool valid_text(const char* text, const std::size_t capacity, std::size_t& length) {
    if (text == nullptr) return false;
    length = 0;
    while (length < capacity && text[length] != '\0') {
        const unsigned char character = static_cast<unsigned char>(text[length]);
        if (character < 0x20U || character > 0x7eU) return false;
        ++length;
    }
    return length < capacity;
}

esp_err_t ensure_default_string(nvs_handle_t handle, const char* key, const std::size_t capacity,
                                bool& wrote_default) {
    char value[kStaPasswordCapacity] = {};
    std::size_t length = capacity;
    const esp_err_t result = nvs_get_str(handle, key, value, &length);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        const esp_err_t set_result = nvs_set_str(handle, key, "");
        wrote_default = set_result == ESP_OK;
        return set_result;
    }
    return result;
}

bool valid_radio_mode(const std::uint8_t value) {
    return value <= static_cast<std::uint8_t>(RadioMode::ApSta);
}

esp_err_t store_string(const char* key, const char* value, const std::size_t capacity,
                       const bool require_nonempty) {
    std::size_t length = 0;
    if (!valid_text(value, capacity, length) || (require_nonempty && length == 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t init_result = wifi_settings_init();
    if (init_result != ESP_OK) return init_result;

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_set_str(handle, key, value);
        if (result == ESP_OK) result = nvs_commit(handle);
        nvs_close(handle);
    }
    return result;
}

}  // namespace

esp_err_t wifi_settings_init() {
    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;

    bool needs_commit = false;
    std::uint8_t radio_mode = 0;
    result = nvs_get_u8(handle, kNvsKeyRadioMode, &radio_mode);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        result = nvs_set_u8(handle, kNvsKeyRadioMode, static_cast<std::uint8_t>(RadioMode::Sta));
        needs_commit = result == ESP_OK;
    }
    if (result == ESP_OK && valid_radio_mode(radio_mode)) {
        bool wrote_ssid_default = false;
        const esp_err_t ssid_result =
            ensure_default_string(handle, kNvsKeyStaSsid, kStaSsidCapacity, wrote_ssid_default);
        if (ssid_result == ESP_OK) {
            bool wrote_password_default = false;
            const esp_err_t password_result =
                ensure_default_string(handle, kNvsKeyStaPassword, kStaPasswordCapacity,
                                      wrote_password_default);
            if (password_result != ESP_OK) {
                result = password_result;
            } else {
                needs_commit = needs_commit || wrote_ssid_default || wrote_password_default;
            }
        } else {
            result = ssid_result;
        }
    } else if (result == ESP_OK) {
        result = ESP_ERR_INVALID_RESPONSE;
    }
    if (result == ESP_OK) {
        const esp_err_t erase_result = nvs_erase_key(handle, kNvsKeyLegacyStatus);
        if (erase_result == ESP_OK) {
            needs_commit = true;
        } else if (erase_result != ESP_ERR_NVS_NOT_FOUND) {
            result = erase_result;
        }
    }
    if (result == ESP_OK && needs_commit) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

esp_err_t wifi_settings_load(WifiSettings& settings) {
    settings = {};
    const esp_err_t init_result = wifi_settings_init();
    if (init_result != ESP_OK) return init_result;

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (result != ESP_OK) return result;

    std::size_t ssid_length = sizeof(settings.ssid);
    result = nvs_get_str(handle, kNvsKeyStaSsid, settings.ssid, &ssid_length);
    if (result == ESP_OK) {
        std::size_t password_length = sizeof(settings.password);
        result = nvs_get_str(handle, kNvsKeyStaPassword, settings.password, &password_length);
    }
    std::uint8_t radio_mode = 0;
    if (result == ESP_OK) result = nvs_get_u8(handle, kNvsKeyRadioMode, &radio_mode);
    nvs_close(handle);
    if (result != ESP_OK) return result;
    if (!valid_radio_mode(radio_mode)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    settings.radio_mode = static_cast<RadioMode>(radio_mode);
    return ESP_OK;
}

esp_err_t wifi_settings_store_sta_ssid(const char* ssid) {
    return store_string(kNvsKeyStaSsid, ssid, kStaSsidCapacity, true);
}

esp_err_t wifi_settings_store_sta_password(const char* password) {
    return store_string(kNvsKeyStaPassword, password, kStaPasswordCapacity, false);
}

esp_err_t wifi_settings_store_radio_mode(const RadioMode mode) {
    if (!valid_radio_mode(static_cast<std::uint8_t>(mode))) return ESP_ERR_INVALID_ARG;

    const esp_err_t init_result = wifi_settings_init();
    if (init_result != ESP_OK) return init_result;

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_set_u8(handle, kNvsKeyRadioMode, static_cast<std::uint8_t>(mode));
        if (result == ESP_OK) result = nvs_commit(handle);
        nvs_close(handle);
    }
    return result;
}

esp_err_t wifi_settings_clear_sta_credentials() {
    const esp_err_t init_result = wifi_settings_init();
    if (init_result != ESP_OK) return init_result;

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;

    result = nvs_set_str(handle, kNvsKeyStaSsid, "");
    if (result == ESP_OK) result = nvs_set_str(handle, kNvsKeyStaPassword, "");
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

const char* wifi_radio_mode_text(const RadioMode mode) {
    switch (mode) {
        case RadioMode::Ap: return "AP";
        case RadioMode::Sta: return "STA";
        case RadioMode::ApSta: return "AP_STA";
    }
    return "unknown";
}

}  // namespace laserctl::wifi
