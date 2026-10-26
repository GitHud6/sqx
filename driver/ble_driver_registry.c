#include "ble_driver_registry.h"

#include <string.h>

#define BLE_DRIVER_REGISTRY_MAX_DRIVERS 8

static const ble_driver_t *s_drivers[BLE_DRIVER_REGISTRY_MAX_DRIVERS];
static size_t s_driver_count;

/**
 * @brief Registers a protocol driver before BLE profile binding.
 *
 * @param driver Driver API and optional capabilities.
 * @return ESP_OK when registered, otherwise an error code.
 */
esp_err_t ble_driver_registry_register(const ble_driver_t *driver)
{
    if (driver == NULL || driver->name == NULL || driver->ops == NULL ||
        driver->ops->on_connected == NULL || driver->ops->on_disconnected == NULL ||
        driver->characteristic_callbacks == NULL || driver->characteristic_callback_count == 0 ||
        s_driver_count >= BLE_DRIVER_REGISTRY_MAX_DRIVERS) {
        return ESP_ERR_INVALID_ARG;
    }
    s_drivers[s_driver_count++] = driver;
    return ESP_OK;
}

/**
 * @brief Finds one named characteristic callback supplied by a driver.
 *
 * @param driver Driver that owns callback definitions.
 * @param name Callback name declared in devices.json.
 * @return Matching callback, or NULL when the driver does not provide it.
 */
const ble_characteristic_callback_t *ble_driver_find_callback(const ble_driver_t *driver,
                                                              const char *name)
{
    if (driver == NULL || name == NULL) {
        return NULL;
    }
    for (size_t index = 0; index < driver->characteristic_callback_count; ++index) {
        if (strcmp(name, driver->characteristic_callbacks[index].name) == 0) {
            return &driver->characteristic_callbacks[index];
        }
    }
    return NULL;
}

/**
 * @brief Finds a driver by the name declared in devices.json.
 *
 * @param name Driver name declared in the device profile.
 * @return Matching driver, or NULL when unregistered.
 */
const ble_driver_t *ble_driver_registry_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t index = 0; index < s_driver_count; ++index) {
        if (strcmp(name, s_drivers[index]->name) == 0) {
            return s_drivers[index];
        }
    }
    return NULL;
}
