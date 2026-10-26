#ifndef NETGATE_DRIVER_BLE_DRIVER_H
#define NETGATE_DRIVER_BLE_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct ble_device ble_device_t;

typedef enum {
    BLE_WRITE_WITH_RESPONSE,
    BLE_WRITE_WITHOUT_RESPONSE,
} ble_write_mode_t;

typedef struct {
    esp_err_t (*on_connected)(ble_device_t *device);
    esp_err_t (*on_disconnected)(ble_device_t *device, int reason);
} ble_driver_ops_t;

typedef struct {
    const char *name;
    esp_err_t (*on_read)(ble_device_t *device,
                         const char *characteristic_id,
                         const uint8_t *data, size_t length);
    esp_err_t (*on_write_complete)(ble_device_t *device,
                                   const char *characteristic_id,
                                   int status);
    esp_err_t (*on_notification)(ble_device_t *device,
                                 const char *characteristic_id,
                                 const uint8_t *data, size_t length);
} ble_characteristic_callback_t;

typedef struct {
    const char *name;
    const ble_driver_ops_t *ops;
    const ble_characteristic_callback_t *characteristic_callbacks;
    size_t characteristic_callback_count;
    esp_err_t (*ota_begin)(ble_device_t *device, size_t image_size);
    esp_err_t (*ota_write)(ble_device_t *device, const uint8_t *data, size_t length);
    esp_err_t (*ota_finish)(ble_device_t *device);
} ble_driver_t;

/**
 * @brief Finds one named characteristic callback supplied by a driver.
 *
 * @param driver Driver that owns callback definitions.
 * @param name Callback name declared in devices.json.
 * @return Matching callback, or NULL when the driver does not provide it.
 */
const ble_characteristic_callback_t *ble_driver_find_callback(const ble_driver_t *driver,
                                                              const char *name);

#endif
