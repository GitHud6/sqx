#include "generic_gatt_driver.h"

#include "ble_bus.h"
#include "esp_log.h"
#include "generic_gatt_protocol.h"

static const char *TAG = "generic_gatt_driver";

static esp_err_t generic_gatt_on_connected(ble_device_t *device);
static esp_err_t generic_gatt_on_disconnected(ble_device_t *device, int reason);
static esp_err_t generic_gatt_on_read(ble_device_t *device,
                                      const char *characteristic_id,
                                      const uint8_t *data, size_t length);
static esp_err_t generic_gatt_on_write_complete(ble_device_t *device,
                                                const char *characteristic_id,
                                                int status);
static esp_err_t generic_gatt_on_notification(ble_device_t *device,
                                              const char *characteristic_id,
                                              const uint8_t *data, size_t length);
static const ble_characteristic_profile_t *generic_gatt_find_characteristic(
    const ble_device_t *device, const char *characteristic_id);

static const ble_driver_ops_t s_generic_gatt_ops = {
    .on_connected = generic_gatt_on_connected,
    .on_disconnected = generic_gatt_on_disconnected,
};

static const ble_characteristic_callback_t s_generic_gatt_callbacks[] = {
    {
        .name = "generic_gatt.io",
        .on_read = generic_gatt_on_read,
        .on_write_complete = generic_gatt_on_write_complete,
        .on_notification = generic_gatt_on_notification,
    },
};

const ble_driver_t generic_gatt_driver = {
    .name = "generic_gatt",
    .ops = &s_generic_gatt_ops,
    .characteristic_callbacks = s_generic_gatt_callbacks,
    .characteristic_callback_count =
        sizeof(s_generic_gatt_callbacks) / sizeof(s_generic_gatt_callbacks[0]),
    .ota_begin = NULL,
    .ota_write = NULL,
    .ota_finish = NULL,
};

/**
 * @brief Starts a read on a named characteristic of the active generic device.
 *
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_read(const char *characteristic_id)
{
    ble_device_t *device = ble_bus_get_ready_device(&generic_gatt_driver);
    return device == NULL ? ESP_ERR_INVALID_STATE :
        ble_bus_read(device, characteristic_id);
}

/**
 * @brief Starts a read on a named characteristic of one configured generic device.
 *
 * @param device_id Device identifier declared in devices.json.
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_read_device(const char *device_id, const char *characteristic_id)
{
    ble_device_t *device = ble_bus_get_ready_device_by_id(device_id);
    return device == NULL || device->driver != &generic_gatt_driver ? ESP_ERR_INVALID_STATE :
        ble_bus_read(device, characteristic_id);
}

/**
 * @brief Writes bytes to a named characteristic of the active generic device.
 *
 * @param characteristic_id JSON characteristic identifier.
 * @param mode GATT write mode.
 * @param data Bytes to send.
 * @param length Number of bytes to send.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t generic_gatt_write(const char *characteristic_id,
                             ble_write_mode_t mode,
                             const uint8_t *data, size_t length)
{
    ble_device_t *device = ble_bus_get_ready_device(&generic_gatt_driver);
    return device == NULL ? ESP_ERR_INVALID_STATE :
        ble_bus_write(device, characteristic_id, mode, data, length);
}

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
                                    const uint8_t *data, size_t length)
{
    ble_device_t *device = ble_bus_get_ready_device_by_id(device_id);
    return device == NULL || device->driver != &generic_gatt_driver ? ESP_ERR_INVALID_STATE :
        ble_bus_write(device, characteristic_id, mode, data, length);
}

/**
 * @brief Accepts a generic device after its configured GATT resources are ready.
 *
 * @param device Connected generic GATT device.
 * @return ESP_OK when the device context is valid, otherwise an error code.
 */
static esp_err_t generic_gatt_on_connected(ble_device_t *device)
{
    if (device == NULL || device->profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Generic device ready: %s", device->profile->device_id);
    return ESP_OK;
}

/**
 * @brief Handles generic device disconnection without protocol-specific cleanup.
 *
 * @param device Disconnected generic GATT device.
 * @param reason NimBLE disconnect reason.
 * @return ESP_OK after the lifecycle notification is handled.
 */
static esp_err_t generic_gatt_on_disconnected(ble_device_t *device, int reason)
{
    if (device == NULL || device->profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Generic device disconnected: %s, reason=%d",
             device->profile->device_id, reason);
    return ESP_OK;
}

/**
 * @brief Converts a generic characteristic read result into a gateway event.
 *
 * @param device Device that owns the characteristic.
 * @param characteristic_id JSON characteristic identifier.
 * @param data Read bytes.
 * @param length Number of read bytes.
 * @return ESP_OK when the event is queued, otherwise an error code.
 */
static esp_err_t generic_gatt_on_read(ble_device_t *device,
                                      const char *characteristic_id,
                                      const uint8_t *data, size_t length)
{
    const ble_characteristic_profile_t *characteristic =
        generic_gatt_find_characteristic(device, characteristic_id);
    if (characteristic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = generic_gatt_protocol_publish(device->profile->device_id,
                                                      characteristic_id,
                                                      characteristic->event_type,
                                                      characteristic->topic,
                                                      &characteristic->decoder,
                                                      data, length);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Generic read decode failed: device=%s characteristic=%s error=%s",
                 device->profile->device_id, characteristic_id, esp_err_to_name(result));
    }
    return result;
}

/**
 * @brief Handles completion of an acknowledged generic characteristic write.
 *
 * @param device Device that owns the characteristic.
 * @param characteristic_id JSON characteristic identifier.
 * @param status NimBLE write completion status.
 * @return ESP_OK when the write succeeded, otherwise ESP_FAIL.
 */
static esp_err_t generic_gatt_on_write_complete(ble_device_t *device,
                                                const char *characteristic_id,
                                                int status)
{
    if (device == NULL || device->profile == NULL || characteristic_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (status != 0) {
        ESP_LOGW(TAG, "Generic write failed: device=%s characteristic=%s status=%d",
                 device->profile->device_id, characteristic_id, status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Converts a generic characteristic notification into a gateway event.
 *
 * @param device Device that owns the characteristic.
 * @param characteristic_id JSON characteristic identifier.
 * @param data Notification bytes.
 * @param length Number of notification bytes.
 * @return ESP_OK when the event is queued, otherwise an error code.
 */
static esp_err_t generic_gatt_on_notification(ble_device_t *device,
                                              const char *characteristic_id,
                                              const uint8_t *data, size_t length)
{
    const ble_characteristic_profile_t *characteristic =
        generic_gatt_find_characteristic(device, characteristic_id);
    if (characteristic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t result = generic_gatt_protocol_publish(device->profile->device_id,
                                                      characteristic_id,
                                                      characteristic->event_type,
                                                      characteristic->topic,
                                                      &characteristic->decoder,
                                                      data, length);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Generic notify decode failed: device=%s characteristic=%s error=%s",
                 device->profile->device_id, characteristic_id, esp_err_to_name(result));
    }
    return result;
}

/**
 * @brief Finds characteristic metadata required for generic event routing.
 *
 * @param device Connected generic GATT device.
 * @param characteristic_id JSON characteristic identifier.
 * @return Matching characteristic metadata, or NULL when unavailable.
 */
static const ble_characteristic_profile_t *generic_gatt_find_characteristic(
    const ble_device_t *device, const char *characteristic_id)
{
    return device == NULL ? NULL :
        ble_profile_find_characteristic(device->profile, characteristic_id);
}
