#include "network_manager.h"

#include <stdbool.h>

#include "esp_log.h"
#include "mqtt_gateway.h"
#include "wifi_manager.h"

static const char *TAG = "network_manager";
static bool s_initialized;
static bool s_mqtt_ready;

static void network_on_wifi_connection(bool connected, void *context);

/**
 * @brief Starts or stops MQTT to follow Wi-Fi IP availability.
 *
 * @param connected True when Wi-Fi owns an IP address.
 * @param context Unused Wi-Fi callback context.
 * @return None.
 */
static void network_on_wifi_connection(bool connected, void *context)
{
    (void)context;
    if (!s_mqtt_ready) {
        return;
    }
    esp_err_t result = connected ? mqtt_gateway_start() : mqtt_gateway_stop();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "MQTT lifecycle update failed: %s", esp_err_to_name(result));
    }
}

/**
 * @brief Initializes optional MQTT support before the Wi-Fi station starts.
 *
 * @return ESP_OK when network control state is ready, otherwise an error code.
 */
esp_err_t network_manager_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = mqtt_gateway_init();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "MQTT disabled during Wi-Fi startup: %s",
                 esp_err_to_name(result));
    } else {
        s_mqtt_ready = true;
    }
    s_initialized = true;
    return ESP_OK;
}

/**
 * @brief Starts Wi-Fi independently from optional MQTT availability.
 *
 * @return ESP_OK when Wi-Fi startup is requested, otherwise an error code.
 */
esp_err_t network_manager_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return wifi_manager_init(network_on_wifi_connection, NULL);
}
