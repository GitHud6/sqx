#include "ble_device0_protocol.h"

#include <string.h>

#include "esp_log.h"
#include "gateway_event.h"

static const char *TAG = "ble_device0_protocol";

/**
 * @brief Logs raw Ble_device0 telemetry while its frame format is unspecified.
 *
 * @param device_id JSON device identifier.
 * @param characteristic_id JSON characteristic identifier.
 * @param data Raw notification bytes.
 * @param length Number of bytes in data.
 * @return ESP_OK when a normalized event is accepted, otherwise an error code.
 */
esp_err_t ble_device0_protocol_on_telemetry(const char *device_id,
                                            const char *characteristic_id,
                                            const uint8_t *data, size_t length)
{
    if (device_id == NULL || characteristic_id == NULL || data == NULL || length == 0 ||
        length > GATEWAY_EVENT_PAYLOAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, length, ESP_LOG_DEBUG);

    gateway_event_t event = {0};
    strncpy(event.device_id, device_id, sizeof(event.device_id) - 1);
    strncpy(event.characteristic_id, characteristic_id,
            sizeof(event.characteristic_id) - 1);
    strncpy(event.event_type, "telemetry", sizeof(event.event_type) - 1);
    strncpy(event.topic, "telemetry", sizeof(event.topic) - 1);
    strncpy(event.decoder_type, "raw", sizeof(event.decoder_type) - 1);
    memcpy(event.payload, data, length);
    event.payload_length = length;
    return gateway_event_publish(&event);
}
