#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define NETGATE_WIFI_DEFAULT_SSID "changlang"
#define NETGATE_WIFI_DEFAULT_PASSWORD "changlang"
#define WIFI_RECONNECT_INITIAL_DELAY_MS 2000U
#define WIFI_RECONNECT_MAX_SHIFT 3U

static const char *TAG = "wifi_manager";
static bool s_initialized;
static bool s_connected;
static uint32_t s_reconnect_attempts;
static wifi_manager_connection_handler_t s_connection_handler;
static void *s_connection_context;
static esp_timer_handle_t s_reconnect_timer;
static wifi_manager_provisioning_handler_t s_provisioning_handler;
static void *s_provisioning_context;
static char s_candidate_ssid[33];
static char s_candidate_password[65];
static bool s_candidate_valid;
static bool s_candidate_switching;

static void notify_connection(bool connected);
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static const char *configured_ssid(void);
static const char *configured_password(void);
static const char *disconnect_reason_name(uint8_t reason);
static void reconnect_timer_callback(void *argument);
static void schedule_reconnect(void);
static esp_err_t load_persisted_credentials(char *ssid, size_t ssid_size,
                                            char *password, size_t password_size);
static esp_err_t persist_credentials(const char *ssid, const char *password);

/**
 * @brief Converts a selected ESP-IDF Wi-Fi disconnect reason into log text.
 *
 * @param reason ESP-IDF Wi-Fi disconnect reason code.
 * @return Static reason description.
 */
static const char *disconnect_reason_name(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_AUTH_FAIL:
        return "authentication failed";
    case WIFI_REASON_ASSOC_FAIL:
        return "association failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "WPA handshake timeout";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "WPA four-way handshake timeout";
    case WIFI_REASON_NO_AP_FOUND:
        return "access point not found";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        return "no access point with compatible security";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        return "access point rejected by auth mode threshold";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "access point rejected by RSSI threshold";
    case WIFI_REASON_CONNECTION_FAIL:
        return "connection failed";
    default:
        return "other";
    }
}

/**
 * @brief Requests one asynchronous Wi-Fi reconnection after a backoff delay.
 *
 * @param argument Unused timer callback argument.
 * @return None.
 */
static void reconnect_timer_callback(void *argument)
{
    (void)argument;
    esp_err_t result = esp_wifi_connect();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi reconnect request failed: %s", esp_err_to_name(result));
        schedule_reconnect();
    }
}

/**
 * @brief Schedules exponential-backoff Wi-Fi reconnection without blocking the event loop.
 *
 * @param None.
 * @return None.
 */
static void schedule_reconnect(void)
{
    if (s_reconnect_timer == NULL) {
        return;
    }
    uint32_t shift = s_reconnect_attempts > WIFI_RECONNECT_MAX_SHIFT ?
        WIFI_RECONNECT_MAX_SHIFT : s_reconnect_attempts;
    uint32_t delay_ms = WIFI_RECONNECT_INITIAL_DELAY_MS << shift;
    ++s_reconnect_attempts;
    if (esp_timer_is_active(s_reconnect_timer)) {
        esp_timer_stop(s_reconnect_timer);
    }
    esp_err_t result = esp_timer_start_once(s_reconnect_timer,
                                            (uint64_t)delay_ms * 1000U);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi reconnect attempt %lu scheduled in %lu ms",
                 (unsigned long)s_reconnect_attempts, (unsigned long)delay_ms);
    } else {
        ESP_LOGW(TAG, "Cannot schedule Wi-Fi reconnect: %s", esp_err_to_name(result));
    }
}

/**
 * @brief Returns the configured SSID or the project bootstrap network name.
 *
 * @param None.
 * @return Null-terminated SSID text.
 */
static const char *configured_ssid(void)
{
    static char ssid[33];
    if (load_persisted_credentials(ssid, sizeof(ssid), NULL, 0) == ESP_OK) {
        return ssid;
    }
    return CONFIG_NETGATE_WIFI_SSID[0] == '\0' ?
        NETGATE_WIFI_DEFAULT_SSID : CONFIG_NETGATE_WIFI_SSID;
}

/**
 * @brief Returns the configured password or the project bootstrap password.
 *
 * @param None.
 * @return Null-terminated Wi-Fi password text.
 */
static const char *configured_password(void)
{
    static char password[65];
    if (load_persisted_credentials(NULL, 0, password, sizeof(password)) == ESP_OK) {
        return password;
    }
    return CONFIG_NETGATE_WIFI_PASSWORD[0] == '\0' ?
        NETGATE_WIFI_DEFAULT_PASSWORD : CONFIG_NETGATE_WIFI_PASSWORD;
}

/**
 * @brief Loads the last successfully provisioned credentials from NVS.
 *
 * @param ssid Destination buffer for SSID, or NULL to skip it.
 * @param ssid_size SSID destination capacity.
 * @param password Destination buffer for password, or NULL to skip it.
 * @param password_size Password destination capacity.
 * @return ESP_OK when both requested values exist, otherwise an error code.
 */
static esp_err_t load_persisted_credentials(char *ssid, size_t ssid_size,
                                            char *password, size_t password_size)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open("wifi_cfg", NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }
    if (ssid != NULL) {
        size_t length = ssid_size;
        result = nvs_get_str(handle, "ssid", ssid, &length);
    }
    if (result == ESP_OK && password != NULL) {
        size_t length = password_size;
        result = nvs_get_str(handle, "password", password, &length);
    }
    nvs_close(handle);
    return result;
}

/**
 * @brief Persists validated Wi-Fi credentials as the active configuration.
 *
 * @param ssid SSID to persist.
 * @param password Password to persist.
 * @return ESP_OK when committed, otherwise an error code.
 */
static esp_err_t persist_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open("wifi_cfg", NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = nvs_set_str(handle, "ssid", ssid);
    if (result == ESP_OK) {
        result = nvs_set_str(handle, "password", password);
    }
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

/**
 * @brief Notifies the owner when Wi-Fi IP availability changes.
 *
 * @param connected True when an IP address is available.
 * @return None.
 */
static void notify_connection(bool connected)
{
    s_connected = connected;
    if (connected && s_candidate_valid) {
        esp_err_t persist_result = persist_credentials(s_candidate_ssid, s_candidate_password);
        if (persist_result == ESP_OK) {
            s_candidate_valid = false;
            memset(s_candidate_ssid, 0, sizeof(s_candidate_ssid));
            memset(s_candidate_password, 0, sizeof(s_candidate_password));
            ESP_LOGI(TAG, "Provisioned Wi-Fi credentials committed");
        } else {
            ESP_LOGW(TAG, "Cannot persist provisioned Wi-Fi credentials: %s",
                     esp_err_to_name(persist_result));
        }
    }
    if (s_connection_handler != NULL) {
        s_connection_handler(connected, s_connection_context);
    }
    if (s_provisioning_handler != NULL && (connected || !s_candidate_switching)) {
        s_provisioning_handler(connected, s_provisioning_context);
    }
}

/**
 * @brief Handles station start, disconnect, and IPv4 acquisition events.
 *
 * @param arg Unused event handler argument.
 * @param event_base ESP-IDF event base.
 * @param event_id Event identifier within the base.
 * @param event_data Event-specific payload.
 * @return None.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi station started");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *disconnected = event_data;
        uint8_t reason = disconnected == NULL ? 0U : disconnected->reason;
        int8_t rssi = disconnected == NULL ? 0 : disconnected->rssi;
        ESP_LOGW(TAG, "Wi-Fi disconnected: reason=%u (%s), rssi=%d",
                 reason, disconnect_reason_name(reason), rssi);
        notify_connection(false);
        if (s_candidate_switching) {
            s_candidate_switching = false;
            esp_err_t reconnect_result = esp_wifi_connect();
            if (reconnect_result != ESP_OK) {
                ESP_LOGW(TAG, "Provisioning reconnect request failed: %s",
                         esp_err_to_name(reconnect_result));
                schedule_reconnect();
            }
            return;
        }
        schedule_reconnect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected with an IP address");
        s_reconnect_attempts = 0;
        if (s_reconnect_timer != NULL && esp_timer_is_active(s_reconnect_timer)) {
            esp_timer_stop(s_reconnect_timer);
        }
        notify_connection(true);
    }
}

/**
 * @brief Initializes and starts the configured Wi-Fi station.
 *
 * @param handler Callback invoked when the station gains or loses an IP address.
 * @param context Opaque callback context.
 * @return ESP_OK when Wi-Fi starts, otherwise an error code.
 */
esp_err_t wifi_manager_init(wifi_manager_connection_handler_t handler, void *context)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const char *ssid = configured_ssid();
    const char *password = configured_password();
    if (strlen(ssid) >= sizeof(((wifi_config_t *)0)->sta.ssid) ||
        strlen(password) >= sizeof(((wifi_config_t *)0)->sta.password)) {
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_LOGI(TAG, "Starting Wi-Fi station for SSID: %s", ssid);

    esp_err_t result = esp_netif_init();
    if (result != ESP_OK) {
        return result;
    }
    result = esp_event_loop_create_default();
    if (result != ESP_OK) {
        return result;
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&init_config);
    if (result != ESP_OK) {
        return result;
    }
    const esp_timer_create_args_t reconnect_timer_config = {
        .callback = reconnect_timer_callback,
        .arg = NULL,
        .name = "wifi_reconnect",
    };
    result = esp_timer_create(&reconnect_timer_config, &s_reconnect_timer);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL);
    if (result != ESP_OK) {
        return result;
    }

    wifi_config_t station_config = {0};
    snprintf((char *)station_config.sta.ssid, sizeof(station_config.sta.ssid),
             "%s", ssid);
    snprintf((char *)station_config.sta.password, sizeof(station_config.sta.password),
             "%s", password);
    station_config.sta.pmf_cfg.capable = true;
    station_config.sta.pmf_cfg.required = false;

    result = esp_wifi_set_mode(WIFI_MODE_STA);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_wifi_set_config(WIFI_IF_STA, &station_config);
    if (result != ESP_OK) {
        return result;
    }

    s_connection_handler = handler;
    s_connection_context = context;
    result = esp_wifi_start();
    if (result == ESP_OK) {
        s_initialized = true;
    }
    return result;
}

/**
 * @brief Reports whether the station currently owns an IPv4 address.
 *
 * @return true when connected with an IP address, otherwise false.
 */
bool wifi_manager_is_connected(void)
{
    return s_connected;
}

/**
 * @brief Registers an observer for provisioning status updates.
 *
 * @param handler Callback invoked when Wi-Fi provisioning state changes.
 * @param context Opaque callback context.
 * @return None.
 */
void wifi_manager_set_provisioning_handler(wifi_manager_provisioning_handler_t handler,
                                           void *context)
{
    s_provisioning_handler = handler;
    s_provisioning_context = context;
}

/**
 * @brief Applies candidate Wi-Fi credentials and starts an asynchronous connection attempt.
 *
 * @param ssid Candidate access point name.
 * @param password Candidate WPA password.
 * @return ESP_OK when the attempt was requested, otherwise an error code.
 */
esp_err_t wifi_manager_apply_credentials(const char *ssid, const char *password)
{
    if (!s_initialized || ssid == NULL || password == NULL || ssid[0] == '\0' ||
        strlen(ssid) >= sizeof(s_candidate_ssid) || strlen(password) >= sizeof(s_candidate_password)) {
        return ESP_ERR_INVALID_ARG;
    }
    snprintf(s_candidate_ssid, sizeof(s_candidate_ssid), "%s", ssid);
    snprintf(s_candidate_password, sizeof(s_candidate_password), "%s", password);
    s_candidate_valid = true;

    wifi_config_t config = {0};
    snprintf((char *)config.sta.ssid, sizeof(config.sta.ssid), "%s", ssid);
    snprintf((char *)config.sta.password, sizeof(config.sta.password), "%s", password);
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (result != ESP_OK) {
        return result;
    }
    s_candidate_switching = s_connected;
    if (s_candidate_switching) {
        result = esp_wifi_disconnect();
        if (result != ESP_OK) {
            s_candidate_switching = false;
            return result;
        }
        return ESP_OK;
    }
    return esp_wifi_connect();
}
