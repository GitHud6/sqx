#ifndef NETGATE_BLE_BUS_H
#define NETGATE_BLE_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "ble_driver.h"
#include "ble_profile.h"

#define BLE_MAX_DEVICES BLE_PROFILE_MAX_DEVICES
#define BLE_BUS_MAX_CONNECTIONS 2U

typedef struct {
    ble_device_t *device;
    ble_characteristic_profile_t *profile;
    uint8_t properties;
    uint16_t value_handle;
    uint16_t definition_handle;
    uint16_t descriptor_end_handle;
    uint16_t cccd_handle;
    bool notification_enabled;
} ble_characteristic_runtime_t;

typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
    ble_characteristic_runtime_t characteristics[BLE_PROFILE_MAX_CHARACTERISTICS];
} ble_service_runtime_t;

struct ble_device {
    uint8_t address[6];
    uint8_t address_type;
    int8_t rssi;
    uint16_t negotiated_mtu;
    char observed_advertising_name[BLE_PROFILE_MAX_NAME];
    uint16_t connection_handle;
    ble_profile_t *profile;
    const ble_driver_t *driver;
    void *protocol_context;
    size_t discovery_service_index;
    size_t discovery_characteristic_index;
    ble_service_runtime_t services[BLE_PROFILE_MAX_SERVICES];
    int last_error;
    bool registered;
    bool connecting;
    bool connected;
    bool ready;
};

typedef struct ble_device ble_device_slot_t;

/**
 * @brief Registers one JSON-backed device profile with the BLE bus.
 *
 * @param profile Loaded device profile.
 * @param driver Registered driver selected by profile driver name.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t ble_bus_register_profile(ble_profile_t *profile, const ble_driver_t *driver);

/**
 * @brief Initializes NimBLE before GATT services are registered.
 *
 * @return ESP_OK when NimBLE is ready for GATT service registration, otherwise an error code.
 */
esp_err_t ble_bus_init(void);

/**
 * @brief Starts the NimBLE host task after all local GATT services are registered.
 *
 * @return ESP_OK when host startup was requested, otherwise an error code.
 */
esp_err_t ble_bus_start(void);

/**
 * @brief Stops scanning and all active connections before profile replacement.
 *
 * @param timeout_ms Maximum time to wait for connection shutdown.
 * @return ESP_OK when quiescent, otherwise an error code.
 */
esp_err_t ble_bus_suspend(uint32_t timeout_ms);

/**
 * @brief Clears registered profile slots while the bus is suspended.
 *
 * @return ESP_OK when cleared, otherwise an error code.
 */
esp_err_t ble_bus_clear_profiles(void);

/**
 * @brief Resumes scanning after replacement profiles are registered.
 *
 * @return ESP_OK when scanning is requested, otherwise an error code.
 */
esp_err_t ble_bus_resume(void);

/**
 * @brief Finds the ready device instance associated with one driver.
 *
 * @param driver Driver that owns the device.
 * @return Ready device, or NULL when unavailable.
 */
ble_device_t *ble_bus_get_ready_device(const ble_driver_t *driver);

/**
 * @brief Finds a ready device by its configured identifier.
 *
 * @param device_id Device identifier declared in devices.json.
 * @return Ready device, or NULL when unavailable.
 */
ble_device_t *ble_bus_get_ready_device_by_id(const char *device_id);

/**
 * @brief Writes bytes to a named JSON characteristic.
 *
 * @param device Connected device.
 * @param characteristic_id JSON characteristic identifier.
 * @param mode GATT write mode.
 * @param data Bytes to send.
 * @param length Number of bytes to send.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_bus_write(ble_device_t *device, const char *characteristic_id,
                        ble_write_mode_t mode, const uint8_t *data, size_t length);

/**
 * @brief Starts a GATT read from a named JSON characteristic.
 *
 * @param device Connected device.
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_bus_read(ble_device_t *device, const char *characteristic_id);

/**
 * @brief Reports whether a named driver exposes its optional OTA operations.
 *
 * @param driver Driver to inspect.
 * @return true when all OTA operations are available, otherwise false.
 */
bool ble_bus_driver_supports_ota(const ble_driver_t *driver);

/**
 * @brief Starts driver-specific OTA through the common device API.
 *
 * @param device Ready device.
 * @param image_size Total OTA image size in bytes.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_begin(ble_device_t *device, size_t image_size);

/**
 * @brief Sends one OTA data block through the common device API.
 *
 * @param device Ready device.
 * @param data OTA data block.
 * @param length OTA block length in bytes.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_write(ble_device_t *device, const uint8_t *data, size_t length);

/**
 * @brief Finishes driver-specific OTA through the common device API.
 *
 * @param device Ready device.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_finish(ble_device_t *device);

#endif
