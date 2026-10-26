#include "generic_gatt_protocol.h"

#include <stdint.h>
#include <string.h>

#include "gateway_event.h"

/**
 * @brief Converts a decoder type to its JSON-facing name.
 *
 * @param type Decoder type.
 * @return Static decoder type text, or NULL when unsupported.
 */
static const char *decoder_type_name(ble_decoder_type_t type)
{
    switch (type) {
    case BLE_DECODER_RAW:
        return "raw";
    case BLE_DECODER_UINT8:
        return "uint8";
    case BLE_DECODER_UINT16:
        return "uint16";
    case BLE_DECODER_INT16:
        return "int16";
    case BLE_DECODER_UINT32:
        return "uint32";
    default:
        return NULL;
    }
}

/**
 * @brief Extracts one configured numeric field from raw characteristic bytes.
 *
 * @param decoder Decoder declaration from devices.json.
 * @param data Raw characteristic bytes.
 * @param length Number of raw bytes.
 * @param value Output scaled numeric value.
 * @return ESP_OK when decoded, otherwise an error code.
 */
static esp_err_t decode_numeric_value(const ble_decoder_profile_t *decoder,
                                      const uint8_t *data, size_t length, double *value)
{
    if (decoder == NULL || data == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t expected_length = 0;
    switch (decoder->type) {
    case BLE_DECODER_UINT8:
        expected_length = 1;
        break;
    case BLE_DECODER_UINT16:
    case BLE_DECODER_INT16:
        expected_length = 2;
        break;
    case BLE_DECODER_UINT32:
        expected_length = 4;
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (decoder->length != expected_length || decoder->offset > length ||
        decoder->length > length - decoder->offset) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint8_t *field = &data[decoder->offset];
    uint32_t unsigned_value = 0;
    switch (decoder->type) {
    case BLE_DECODER_UINT8:
        unsigned_value = field[0];
        break;
    case BLE_DECODER_UINT16:
    case BLE_DECODER_INT16:
        if (decoder->byte_order == BLE_BYTE_ORDER_LITTLE_ENDIAN) {
            unsigned_value = (uint16_t)field[0] | ((uint16_t)field[1] << 8);
        } else if (decoder->byte_order == BLE_BYTE_ORDER_BIG_ENDIAN) {
            unsigned_value = ((uint16_t)field[0] << 8) | (uint16_t)field[1];
        } else {
            return ESP_ERR_INVALID_ARG;
        }
        break;
    case BLE_DECODER_UINT32:
        if (decoder->byte_order == BLE_BYTE_ORDER_LITTLE_ENDIAN) {
            unsigned_value = (uint32_t)field[0] | ((uint32_t)field[1] << 8) |
                             ((uint32_t)field[2] << 16) | ((uint32_t)field[3] << 24);
        } else if (decoder->byte_order == BLE_BYTE_ORDER_BIG_ENDIAN) {
            unsigned_value = ((uint32_t)field[0] << 24) | ((uint32_t)field[1] << 16) |
                             ((uint32_t)field[2] << 8) | (uint32_t)field[3];
        } else {
            return ESP_ERR_INVALID_ARG;
        }
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    double raw_value = (double)unsigned_value;
    if (decoder->type == BLE_DECODER_INT16) {
        int32_t signed_value = unsigned_value <= INT16_MAX ?
            (int32_t)unsigned_value : (int32_t)unsigned_value - 0x10000;
        raw_value = (double)signed_value;
    }
    *value = raw_value * decoder->scale;
    return ESP_OK;
}

/**
 * @brief Decodes generic GATT bytes and publishes a normalized gateway event.
 *
 * @param device_id JSON device identifier.
 * @param characteristic_id JSON characteristic identifier.
 * @param event_type Semantic event type declared in the device profile.
 * @param topic MQTT topic segment declared in the device profile.
 * @param decoder Declarative field decoder declared in the device profile.
 * @param data Raw characteristic bytes.
 * @param length Number of bytes in data.
 * @return ESP_OK when the event is accepted, otherwise an error code.
 */
esp_err_t generic_gatt_protocol_publish(const char *device_id,
                                         const char *characteristic_id,
                                         const char *event_type,
                                         const char *topic,
                                         const ble_decoder_profile_t *decoder,
                                         const uint8_t *data, size_t length)
{
    if (device_id == NULL || characteristic_id == NULL || event_type == NULL || topic == NULL ||
        decoder == NULL || data == NULL || length == 0 || length > GATEWAY_EVENT_PAYLOAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    gateway_event_t event = {0};
    strncpy(event.device_id, device_id, sizeof(event.device_id) - 1);
    strncpy(event.characteristic_id, characteristic_id,
            sizeof(event.characteristic_id) - 1);
    strncpy(event.event_type, event_type, sizeof(event.event_type) - 1);
    strncpy(event.topic, topic, sizeof(event.topic) - 1);
    memcpy(event.payload, data, length);
    event.payload_length = length;

    const char *type_name = decoder_type_name(decoder->type);
    if (type_name == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    strncpy(event.decoder_type, type_name, sizeof(event.decoder_type) - 1);
    if (decoder->type != BLE_DECODER_RAW) {
        esp_err_t result = decode_numeric_value(decoder, data, length, &event.decoded_value);
        if (result != ESP_OK) {
            return result;
        }
        event.has_decoded_value = true;
        strncpy(event.unit, decoder->unit, sizeof(event.unit) - 1);
    }
    return gateway_event_publish(&event);
}
