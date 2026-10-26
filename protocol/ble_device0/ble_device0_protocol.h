#ifndef NETGATE_BLE_DEVICE0_PROTOCOL_H
#define NETGATE_BLE_DEVICE0_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Accepts one raw Ble_device0 telemetry notification for frame decoding.
 *
 * @param device_id JSON device identifier.
 * @param characteristic_id JSON characteristic identifier.
 * @param data Notification payload copied by the BLE bus.
 * @param length Number of bytes in data.
 * @return ESP_OK when a normalized event is accepted, otherwise an error code.
 */
esp_err_t ble_device0_protocol_on_telemetry(const char *device_id,
                                            const char *characteristic_id,
                                            const uint8_t *data, size_t length);

#endif
