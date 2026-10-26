#ifndef NETGATE_GENERIC_GATT_DRIVER_H
#define NETGATE_GENERIC_GATT_DRIVER_H

#include "ble_driver.h"

extern const ble_driver_t generic_gatt_driver;

/**
 * @brief Starts a read on a named characteristic of the first ready generic device.
 *
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_read(const char *characteristic_id);

/**
 * @brief Starts a read on a named characteristic of one configured generic device.
 *
 * @param device_id Device identifier declared in devices.json.
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_read_device(const char *device_id, const char *characteristic_id);

/**
 * @brief Writes bytes to a named characteristic of the first ready generic device.
 *
 * @param characteristic_id JSON characteristic identifier.
 * @param mode GATT write mode.
 * @param data Bytes to send.
 * @param length Number of bytes to send.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_write(const char *characteristic_id,
                              ble_write_mode_t mode,
                              const uint8_t *data, size_t length);

/**
 * @brief Writes bytes to a named characteristic of one configured generic device.
 *
 * @param device_id Device identifier declared in devices.json.
 * @param characteristic_id JSON characteristic identifier.
 * @param mode GATT write mode.
 * @param data Bytes to send.
 * @param length Number of bytes to send.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_write_device(const char *device_id,
                                    const char *characteristic_id,
                                    ble_write_mode_t mode,
                                    const uint8_t *data, size_t length);

#endif
