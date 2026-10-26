#ifndef NETGATE_MQTT_GATEWAY_H
#define NETGATE_MQTT_GATEWAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "gateway_event.h"

typedef esp_err_t (*mqtt_gateway_config_handler_fn)(const char *json, size_t length,
                                                    void *context);
typedef esp_err_t (*mqtt_gateway_ota_handler_fn)(const char *json, size_t length,
                                                 void *context);

/**
 * @brief Initializes the MQTT client without connecting it to the broker.
 *
 * @return ESP_OK when the client is ready, otherwise an error code.
 */
esp_err_t mqtt_gateway_init(void);

/**
 * @brief Starts the MQTT client after Wi-Fi receives an IP address.
 *
 * @return ESP_OK when the client starts, otherwise an error code.
 */
esp_err_t mqtt_gateway_start(void);

/**
 * @brief Stops the MQTT client after Wi-Fi loses its IP address.
 *
 * @return ESP_OK when the client stops, otherwise an error code.
 */
esp_err_t mqtt_gateway_stop(void);

/**
 * @brief Reports whether MQTT currently has a broker connection.
 *
 * @return true when connected to the broker, otherwise false.
 */
bool mqtt_gateway_is_connected(void);

/**
 * @brief Installs the asynchronous consumer for BLE configuration JSON messages.
 *
 * @param handler Configuration consumer, or NULL to disable handling.
 * @param context Opaque consumer context.
 * @return ESP_OK when stored, otherwise an error code.
 */
esp_err_t mqtt_gateway_set_config_handler(mqtt_gateway_config_handler_fn handler,
                                          void *context);

/**
 * @brief Installs the asynchronous consumer for OTA command JSON messages.
 *
 * @param handler OTA command consumer, or NULL to disable handling.
 * @param context Opaque consumer context.
 * @return ESP_OK when stored, otherwise an error code.
 */
esp_err_t mqtt_gateway_set_ota_handler(mqtt_gateway_ota_handler_fn handler,
                                       void *context);

/**
 * @brief Publishes one firmware update state and byte progress report.
 *
 * @param state Short state name such as downloading, failed, or rebooting.
 * @param error ESP error associated with the state, or ESP_OK.
 * @param received Number of image bytes received so far.
 * @param total Total image bytes, or zero when unknown.
 * @return ESP_OK when accepted by MQTT, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_ota_status(const char *state, esp_err_t error,
                                          size_t received, size_t total);

/**
 * @brief Publishes the result of one remote BLE configuration transaction.
 *
 * @param accepted True when the candidate became active.
 * @param error Transaction result code.
 * @param revision Committed configuration revision, or zero on failure.
 * @return ESP_OK when accepted by MQTT, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_config_status(bool accepted, esp_err_t error,
                                             uint32_t revision);

/**
 * @brief Publishes one normalized gateway event as a JSON MQTT message.
 *
 * @param event Event to publish.
 * @return ESP_OK when accepted by the MQTT client, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_event(const gateway_event_t *event);

#endif
