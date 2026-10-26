#include "ble_device0_driver.h"

#include <string.h>

#include "ble_bus.h"
#include "ble_device0_protocol.h"

static esp_err_t ble_device0_on_connected(ble_device_t *device);
static esp_err_t ble_device0_on_disconnected(ble_device_t *device, int reason);
static esp_err_t ble_device0_on_telemetry(ble_device_t *device,
                                   const char *characteristic_id,
                                   const uint8_t *data, size_t length);
static esp_err_t ble_device0_on_command(ble_device_t *device,
                                 const char *characteristic_id, int status);

static const ble_driver_ops_t s_ble_device0_ops = {
    .on_connected = ble_device0_on_connected,
    .on_disconnected = ble_device0_on_disconnected,
};

static const ble_characteristic_callback_t s_ble_device0_callbacks[] = {
    {.name = "Ble_device0.command", .on_write_complete = ble_device0_on_command},
    {.name = "Ble_device0.telemetry", .on_notification = ble_device0_on_telemetry},
};

const ble_driver_t ble_device0_driver = {
    .name = "Ble_device0",
    .ops = &s_ble_device0_ops,
    .characteristic_callbacks = s_ble_device0_callbacks,
    .characteristic_callback_count = sizeof(s_ble_device0_callbacks) / sizeof(s_ble_device0_callbacks[0]),
    .ota_begin = NULL,
    .ota_write = NULL,
    .ota_finish = NULL,
};

/**
 * @brief Writes a Ble_device0 command through the JSON characteristic named command.
 *
 * @param mode GATT write mode.
 * @param data Command bytes.
 * @param length Command length in bytes.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_device0_write(ble_write_mode_t mode, const uint8_t *data, size_t length)
{
    ble_device_t *device = ble_bus_get_ready_device(&ble_device0_driver);
    return device == NULL ? ESP_ERR_INVALID_STATE :
        ble_bus_write(device, "command", mode, data, length);
}

/**
 * @brief Validates the Ble_device0 device after all configured notifications are enabled.
 *
 * @param device Connected Ble_device0 instance.
 * @return ESP_OK when ready, otherwise an error code.
 */
static esp_err_t ble_device0_on_connected(ble_device_t *device)
{
    return device == NULL ? ESP_ERR_INVALID_ARG : ESP_OK;
}

/**
 * @brief Releases Ble_device0 protocol state after disconnection.
 *
 * @param device Disconnected Ble_device0 instance.
 * @param reason NimBLE disconnect reason.
 * @return ESP_OK after cleanup.
 */
static esp_err_t ble_device0_on_disconnected(ble_device_t *device, int reason)
{
    (void)device;
    (void)reason;
    return ESP_OK;
}

/**
 * @brief Routes Ble_device0 telemetry notifications to its protocol decoder.
 *
 * @param device Connected Ble_device0 instance.
 * @param characteristic_id JSON characteristic identifier.
 * @param data Notification bytes.
 * @param length Notification length in bytes.
 * @return ESP_OK when the notification is handled, otherwise an error code.
 */
static esp_err_t ble_device0_on_telemetry(ble_device_t *device,
                                   const char *characteristic_id,
                                   const uint8_t *data, size_t length)
{
    (void)device;
    if (characteristic_id == NULL || strcmp(characteristic_id, "telemetry") != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ble_device0_protocol_on_telemetry(device->profile->device_id,
                                             characteristic_id, data, length);
}

/**
 * @brief Accepts a Ble_device0 command characteristic callback registration.
 *
 * @param device Connected Ble_device0 instance.
 * @param characteristic_id JSON characteristic identifier.
 * @param status NimBLE write completion status.
 * @return ESP_OK when the write completed successfully, otherwise ESP_FAIL.
 */
static esp_err_t ble_device0_on_command(ble_device_t *device,
                                 const char *characteristic_id, int status)
{
    (void)device;
    (void)characteristic_id;
    return status == 0 ? ESP_OK : ESP_FAIL;
}
