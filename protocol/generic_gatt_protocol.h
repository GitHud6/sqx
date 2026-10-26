#ifndef NETGATE_GENERIC_GATT_PROTOCOL_H
#define NETGATE_GENERIC_GATT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "ble_profile.h"
#include "esp_err.h"

/**
 * @brief Decodes generic GATT bytes and publishes a normalized gateway event.
 *
 * @param device_id JSON device identifier.
 * @param characteristic_id JSON characteristic identifier.
 * @param event_type Semantic event type declared in the device profile.
 * @param topic MQTT topic segment declared in the device profile.
 * @param decoder Declarative field decoder declared in the device profile.
 * @param data Raw characteristic bytes.
 * @param length Number of bytes in data.
 * @return ESP_OK when the event is accepted, otherwise an error code.
 */
esp_err_t generic_gatt_protocol_publish(const char *device_id,
                                         const char *characteristic_id,
                                         const char *event_type,
                                         const char *topic,
                                         const ble_decoder_profile_t *decoder,
                                         const uint8_t *data, size_t length);

#endif
