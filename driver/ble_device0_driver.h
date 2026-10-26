#ifndef NETGATE_BLE_DEVICE0_DRIVER_H
#define NETGATE_BLE_DEVICE0_DRIVER_H

#include "ble_driver.h"

extern const ble_driver_t ble_device0_driver;

/**
 * @brief Writes a Ble_device0 command through the JSON characteristic named command.
 *
 * @param mode GATT write mode.
 * @param data Command bytes.
 * @param length Command length in bytes.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_device0_write(ble_write_mode_t mode, const uint8_t *data, size_t length);

#endif
