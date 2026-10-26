#ifndef NETGATE_DRIVER_BLE_DRIVER_REGISTRY_H
#define NETGATE_DRIVER_BLE_DRIVER_REGISTRY_H

#include "ble_driver.h"

/**
 * @brief Registers a protocol driver before BLE profile binding.
 *
 * @param driver Driver API and optional capabilities.
 * @return ESP_OK when registered, otherwise an error code.
 */
esp_err_t ble_driver_registry_register(const ble_driver_t *driver);

/**
 * @brief Finds a driver by the name declared in devices.json.
 *
 * @param name Driver name declared in the device profile.
 * @return Matching driver, or NULL when unregistered.
 */
const ble_driver_t *ble_driver_registry_find(const char *name);

#endif
