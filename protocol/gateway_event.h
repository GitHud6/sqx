#ifndef NETGATE_PROTOCOL_GATEWAY_EVENT_H
#define NETGATE_PROTOCOL_GATEWAY_EVENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define GATEWAY_EVENT_DEVICE_ID_MAX 32
#define GATEWAY_EVENT_CHARACTERISTIC_ID_MAX 32
#define GATEWAY_EVENT_TYPE_MAX 32
#define GATEWAY_EVENT_TOPIC_MAX 64
#define GATEWAY_EVENT_DECODER_TYPE_MAX 16
#define GATEWAY_EVENT_UNIT_MAX 16
#define GATEWAY_EVENT_PAYLOAD_MAX 256
#define GATEWAY_EVENT_SCHEMA_VERSION 2

typedef struct {
    char device_id[GATEWAY_EVENT_DEVICE_ID_MAX];
    char characteristic_id[GATEWAY_EVENT_CHARACTERISTIC_ID_MAX];
    char event_type[GATEWAY_EVENT_TYPE_MAX];
    char topic[GATEWAY_EVENT_TOPIC_MAX];
    char decoder_type[GATEWAY_EVENT_DECODER_TYPE_MAX];
    char unit[GATEWAY_EVENT_UNIT_MAX];
    uint8_t payload[GATEWAY_EVENT_PAYLOAD_MAX];
    size_t payload_length;
    double decoded_value;
    bool has_decoded_value;
} gateway_event_t;

typedef esp_err_t (*gateway_event_sink_fn)(const gateway_event_t *event, void *context);

/**
 * @brief Installs the consumer for normalized protocol events.
 *
 * @param sink Event consumer, or NULL to disable delivery.
 * @param context Opaque consumer context.
 * @return ESP_OK when the sink is stored, otherwise an error code.
 */
esp_err_t gateway_event_set_sink(gateway_event_sink_fn sink, void *context);

/**
 * @brief Publishes one normalized event to the installed consumer.
 *
 * @param event Event to deliver.
 * @return ESP_OK when delivered, ESP_ERR_INVALID_STATE when no sink exists,
 *         or another error returned by the sink.
 */
esp_err_t gateway_event_publish(const gateway_event_t *event);

#endif
