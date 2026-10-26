#include "mqtt_gateway.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mbedtls/base64.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

#define MQTT_GATEWAY_TOPIC_MAX 128

static const char *TAG = "mqtt_gateway";
static esp_mqtt_client_handle_t s_client;
static bool s_started;
static bool s_connected;
static mqtt_gateway_config_handler_fn s_config_handler;
static void *s_config_context;
static mqtt_gateway_ota_handler_fn s_ota_handler;
static void *s_ota_context;
static char *s_config_buffer;
static size_t s_config_received;
static size_t s_config_expected;
static int64_t s_connected_at_us;
static int64_t s_disconnected_at_us;
static char s_ota_buffer[CONFIG_NETGATE_MQTT_OTA_MAX_BYTES + 1U];
static size_t s_ota_received;
static size_t s_ota_expected;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static const char *mqtt_error_type_name(esp_mqtt_error_type_t error_type);
static esp_err_t encode_event_json(const gateway_event_t *event, char **json);
static void handle_config_data(const esp_mqtt_event_t *event);
static void handle_ota_data(const esp_mqtt_event_t *event);

/**
 * @brief Releases an incomplete MQTT configuration message.
 *
 * @return None.
 */
static void reset_config_message(void)
{
    free(s_config_buffer);
    s_config_buffer = NULL;
    s_config_received = 0;
    s_config_expected = 0;
}

/**
 * @brief Tests whether one MQTT event topic exactly matches configured text.
 *
 * @param event MQTT data event.
 * @param topic Expected null-terminated topic.
 * @return true when topics match exactly, otherwise false.
 */
static bool event_topic_matches(const esp_mqtt_event_t *event, const char *topic)
{
    return event != NULL && topic != NULL && event->topic != NULL &&
           event->topic_len == (int)strlen(topic) &&
           memcmp(event->topic, topic, (size_t)event->topic_len) == 0;
}

/**
 * @brief Reassembles and dispatches a possibly fragmented configuration payload.
 *
 * @param event MQTT data event.
 * @return None.
 */
static void handle_config_data(const esp_mqtt_event_t *event)
{
    if (event == NULL || event->current_data_offset < 0 || event->data_len < 0 ||
        event->total_data_len <= 0) {
        reset_config_message();
        return;
    }
    if (event->current_data_offset == 0) {
        reset_config_message();
        if (!event_topic_matches(event, CONFIG_NETGATE_MQTT_CONFIG_TOPIC) ||
            event->total_data_len > CONFIG_NETGATE_MQTT_CONFIG_MAX_BYTES) {
            return;
        }
        s_config_buffer = malloc((size_t)event->total_data_len + 1U);
        if (s_config_buffer == NULL) {
            return;
        }
        s_config_expected = (size_t)event->total_data_len;
    }
    if (s_config_buffer == NULL || event->data == NULL ||
        s_config_received > s_config_expected ||
        (size_t)event->current_data_offset != s_config_received ||
        (size_t)event->data_len > s_config_expected - s_config_received) {
        reset_config_message();
        return;
    }
    memcpy(&s_config_buffer[s_config_received], event->data, (size_t)event->data_len);
    s_config_received += (size_t)event->data_len;
    if (s_config_received == s_config_expected) {
        s_config_buffer[s_config_received] = '\0';
        esp_err_t result = s_config_handler == NULL ? ESP_ERR_INVALID_STATE :
            s_config_handler(s_config_buffer, s_config_received, s_config_context);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "BLE config message rejected before processing: %s",
                     esp_err_to_name(result));
            mqtt_gateway_publish_config_status(false, result, 0);
        }
        reset_config_message();
    }
}

/**
 * @brief Reassembles and dispatches a bounded MQTT OTA command.
 *
 * @param event MQTT data event.
 * @return None.
 */
static void handle_ota_data(const esp_mqtt_event_t *event)
{
    if (event == NULL || event->current_data_offset < 0 || event->data_len < 0 ||
        event->total_data_len <= 0) {
        s_ota_received = 0;
        s_ota_expected = 0;
        return;
    }
    if (event->current_data_offset == 0) {
        s_ota_received = 0;
        s_ota_expected = 0;
        if (!event_topic_matches(event, CONFIG_NETGATE_MQTT_OTA_TOPIC) ||
            (size_t)event->total_data_len > CONFIG_NETGATE_MQTT_OTA_MAX_BYTES) {
            return;
        }
        s_ota_expected = (size_t)event->total_data_len;
    }
    if (s_ota_expected == 0 || event->data == NULL ||
        (size_t)event->current_data_offset != s_ota_received ||
        s_ota_received > s_ota_expected ||
        (size_t)event->data_len > s_ota_expected - s_ota_received) {
        s_ota_received = 0;
        s_ota_expected = 0;
        return;
    }
    memcpy(&s_ota_buffer[s_ota_received], event->data, (size_t)event->data_len);
    s_ota_received += (size_t)event->data_len;
    if (s_ota_received == s_ota_expected) {
        s_ota_buffer[s_ota_received] = '\0';
        esp_err_t result = s_ota_handler == NULL ? ESP_ERR_INVALID_STATE :
            s_ota_handler(s_ota_buffer, s_ota_received, s_ota_context);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "OTA command rejected: %s", esp_err_to_name(result));
            mqtt_gateway_publish_ota_status("rejected", result, 0, 0);
        }
        s_ota_received = 0;
        s_ota_expected = 0;
    }
}

/**
 * @brief Returns a readable name for an ESP-MQTT error category.
 *
 * @param error_type ESP-MQTT error category.
 * @return Static string describing the category.
 */
static const char *mqtt_error_type_name(esp_mqtt_error_type_t error_type)
{
    switch (error_type) {
    case MQTT_ERROR_TYPE_NONE:
        return "none";
    case MQTT_ERROR_TYPE_TCP_TRANSPORT:
        return "tcp_transport";
    case MQTT_ERROR_TYPE_CONNECTION_REFUSED:
        return "connection_refused";
    case MQTT_ERROR_TYPE_SUBSCRIBE_FAILED:
        return "subscribe_failed";
    default:
        return "unknown";
    }
}

/**
 * @brief Encodes a normalized event as a JSON message with a Base64 payload.
 *
 * @param event Event to encode.
 * @param json Output pointer receiving heap-allocated JSON on success.
 * @return ESP_OK when JSON is encoded, otherwise an error code.
 */
static esp_err_t encode_event_json(const gateway_event_t *event, char **json)
{
    if (event == NULL || json == NULL || event->device_id[0] == '\0' ||
        event->characteristic_id[0] == '\0' || event->event_type[0] == '\0' ||
        event->topic[0] == '\0' || event->decoder_type[0] == '\0' ||
        event->payload_length > GATEWAY_EVENT_PAYLOAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t encoded_size = ((event->payload_length + 2U) / 3U) * 4U + 1U;
    char *encoded_payload = calloc(encoded_size, sizeof(*encoded_payload));
    if (encoded_payload == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t encoded_length = 0;
    if (mbedtls_base64_encode((unsigned char *)encoded_payload, encoded_size,
                              &encoded_length, event->payload,
                              event->payload_length) != 0) {
        free(encoded_payload);
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL ||
        cJSON_AddNumberToObject(root, "schema_version", GATEWAY_EVENT_SCHEMA_VERSION) == NULL ||
        cJSON_AddStringToObject(root, "device_id", event->device_id) == NULL ||
        cJSON_AddStringToObject(root, "characteristic", event->characteristic_id) == NULL ||
        cJSON_AddStringToObject(root, "event_type", event->event_type) == NULL ||
        cJSON_AddStringToObject(root, "topic", event->topic) == NULL ||
        cJSON_AddStringToObject(root, "payload_encoding", "base64") == NULL ||
        cJSON_AddStringToObject(root, "payload", encoded_payload) == NULL ||
        cJSON_AddStringToObject(root, "decoder", event->decoder_type) == NULL ||
        cJSON_AddNumberToObject(root, "payload_bytes", event->payload_length) == NULL) {
        cJSON_Delete(root);
        free(encoded_payload);
        return ESP_ERR_NO_MEM;
    }
    if (event->has_decoded_value) {
        if (cJSON_AddNumberToObject(root, "decoded", event->decoded_value) == NULL ||
            (event->unit[0] != '\0' &&
             cJSON_AddStringToObject(root, "unit", event->unit) == NULL)) {
            cJSON_Delete(root);
            free(encoded_payload);
            return ESP_ERR_NO_MEM;
        }
    }
    free(encoded_payload);

    *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return *json == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

/**
 * @brief Tracks MQTT connection state and subscribes to the reserved status topic.
 *
 * @param handler_args Unused event handler argument.
 * @param base ESP-IDF event base.
 * @param event_id MQTT event identifier.
 * @param event_data Event-specific MQTT payload.
 * @return None.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    if (event_id == MQTT_EVENT_CONNECTED) {
        int64_t now_us = esp_timer_get_time();
        s_connected = true;
        if (s_disconnected_at_us > 0) {
            ESP_LOGI(TAG, "MQTT reconnected after outage: duration=%" PRId64 " ms",
                     (now_us - s_disconnected_at_us) / 1000);
            s_disconnected_at_us = 0;
        }
        s_connected_at_us = now_us;
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(s_client, CONFIG_NETGATE_MQTT_STATUS_TOPIC, 0);
        esp_mqtt_client_subscribe(s_client, CONFIG_NETGATE_MQTT_CONFIG_TOPIC, 1);
        esp_mqtt_client_subscribe(s_client, CONFIG_NETGATE_MQTT_OTA_TOPIC, 1);
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        int64_t now_us = esp_timer_get_time();
        s_connected = false;
        if (s_connected_at_us > 0) {
            ESP_LOGW(TAG, "MQTT disconnected after connected for: duration=%" PRId64 " ms",
                     (now_us - s_connected_at_us) / 1000);
            s_connected_at_us = 0;
        } else {
            ESP_LOGW(TAG, "MQTT disconnected before a successful connection");
        }
        s_disconnected_at_us = now_us;
    } else if (event_id == MQTT_EVENT_ERROR) {
        const esp_mqtt_event_t *event = (const esp_mqtt_event_t *)event_data;
        const esp_mqtt_error_codes_t *error = event == NULL ? NULL : event->error_handle;
        if (error == NULL) {
            ESP_LOGE(TAG, "MQTT error event has no error details");
        } else {
            ESP_LOGE(TAG,
                     "MQTT error: type=%s(%d), socket_errno=%d, "
                     "tls_esp_err=0x%x (%s), "
                     "tls_stack_err=%d, cert_flags=0x%x, broker_connect_rc=%d",
                     mqtt_error_type_name(error->error_type), (int)error->error_type,
                     error->esp_transport_sock_errno,
                     (unsigned int)error->esp_tls_last_esp_err,
                     esp_err_to_name(error->esp_tls_last_esp_err),
                     error->esp_tls_stack_err, error->esp_tls_cert_verify_flags,
                     (int)error->connect_return_code);
#ifdef CONFIG_MQTT_PROTOCOL_5
            ESP_LOGE(TAG, "MQTT broker disconnect reason=%d",
                     (int)error->disconnect_return_code);
#endif
        }
    } else if (event_id == MQTT_EVENT_DATA) {
        const esp_mqtt_event_t *event = (const esp_mqtt_event_t *)event_data;
        if (event_topic_matches(event, CONFIG_NETGATE_MQTT_OTA_TOPIC) ||
            s_ota_expected > 0) {
            handle_ota_data(event);
        } else {
            handle_config_data(event);
        }
    }
}

/**
 * @brief Initializes the MQTT client without connecting it to the broker.
 *
 * @return ESP_OK when the client is ready, otherwise an error code.
 */
esp_err_t mqtt_gateway_init(void)
{
    if (s_client != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (CONFIG_NETGATE_MQTT_BROKER_URI[0] == '\0' ||
        CONFIG_NETGATE_MQTT_TOPIC_PREFIX[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_mqtt_client_config_t config = {
        .broker.address.uri = CONFIG_NETGATE_MQTT_BROKER_URI,
        .session.keepalive = 60,
    };
    s_client = esp_mqtt_client_init(&config);
    if (s_client == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                                       mqtt_event_handler, NULL);
    if (result != ESP_OK) {
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    return result;
}

/**
 * @brief Starts the MQTT client after Wi-Fi receives an IP address.
 *
 * @return ESP_OK when the client starts, otherwise an error code.
 */
esp_err_t mqtt_gateway_start(void)
{
    if (s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_started) {
        return ESP_OK;
    }

    esp_err_t result = esp_mqtt_client_start(s_client);
    if (result == ESP_OK) {
        s_started = true;
    }
    return result;
}

/**
 * @brief Stops the MQTT client after Wi-Fi loses its IP address.
 *
 * @return ESP_OK when the client stops, otherwise an error code.
 */
esp_err_t mqtt_gateway_stop(void)
{
    if (s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_started) {
        return ESP_OK;
    }

    esp_err_t result = esp_mqtt_client_stop(s_client);
    if (result == ESP_OK) {
        s_started = false;
        s_connected = false;
    }
    return result;
}

/**
 * @brief Reports whether MQTT currently has a broker connection.
 *
 * @return true when connected to the broker, otherwise false.
 */
bool mqtt_gateway_is_connected(void)
{
    return s_connected;
}

/**
 * @brief Installs the consumer for reassembled BLE configuration JSON.
 *
 * @param handler Configuration consumer, or NULL to disable handling.
 * @param context Opaque consumer context.
 * @return ESP_OK after storage.
 */
esp_err_t mqtt_gateway_set_config_handler(mqtt_gateway_config_handler_fn handler,
                                          void *context)
{
    s_config_handler = handler;
    s_config_context = context;
    return ESP_OK;
}

/**
 * @brief Installs the consumer for reassembled MQTT OTA commands.
 *
 * @param handler OTA command consumer, or NULL to disable handling.
 * @param context Opaque consumer context.
 * @return ESP_OK after storing the handler.
 */
esp_err_t mqtt_gateway_set_ota_handler(mqtt_gateway_ota_handler_fn handler,
                                       void *context)
{
    s_ota_handler = handler;
    s_ota_context = context;
    return ESP_OK;
}

/**
 * @brief Publishes an OTA state and byte progress report as JSON.
 *
 * @param state Short OTA state text.
 * @param error ESP error associated with the state.
 * @param received Number of image bytes received so far.
 * @param total Total image bytes, or zero when unknown.
 * @return ESP_OK when accepted by MQTT, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_ota_status(const char *state, esp_err_t error,
                                          size_t received, size_t total)
{
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state == NULL || state[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    cJSON *root = cJSON_CreateObject();
    if (root == NULL || cJSON_AddStringToObject(root, "state", state) == NULL ||
        cJSON_AddStringToObject(root, "error", esp_err_to_name(error)) == NULL ||
        cJSON_AddNumberToObject(root, "received_bytes", (double)received) == NULL ||
        cJSON_AddNumberToObject(root, "total_bytes", (double)total) == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    int message_id = esp_mqtt_client_publish(s_client,
                                              CONFIG_NETGATE_MQTT_OTA_STATUS_TOPIC,
                                              json, 0, 1, 0);
    cJSON_free(json);
    return message_id < 0 ? ESP_FAIL : ESP_OK;
}

/**
 * @brief Publishes one configuration transaction result as JSON.
 *
 * @param accepted True when the candidate became active.
 * @param error Transaction result code.
 * @param revision Committed revision, or zero on failure.
 * @return ESP_OK when accepted by MQTT, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_config_status(bool accepted, esp_err_t error,
                                             uint32_t revision)
{
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    cJSON *root = cJSON_CreateObject();
    if (root == NULL || cJSON_AddBoolToObject(root, "accepted", accepted) == NULL ||
        cJSON_AddStringToObject(root, "error", esp_err_to_name(error)) == NULL ||
        cJSON_AddNumberToObject(root, "revision", revision) == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }
    char topic[MQTT_GATEWAY_TOPIC_MAX];
    int written = snprintf(topic, sizeof(topic), "%s/result",
                           CONFIG_NETGATE_MQTT_CONFIG_TOPIC);
    if (written < 0 || (size_t)written >= sizeof(topic)) {
        cJSON_free(json);
        return ESP_ERR_INVALID_SIZE;
    }
    int message_id = esp_mqtt_client_publish(s_client, topic, json, 0, 1, 0);
    cJSON_free(json);
    return message_id < 0 ? ESP_FAIL : ESP_OK;
}

/**
 * @brief Publishes one normalized gateway event as a JSON MQTT message.
 *
 * @param event Event to publish.
 * @return ESP_OK when accepted by the MQTT client, otherwise an error code.
 */
esp_err_t mqtt_gateway_publish_event(const gateway_event_t *event)
{
    if (event == NULL || event->device_id[0] == '\0' ||
        event->characteristic_id[0] == '\0' || event->event_type[0] == '\0' ||
        event->topic[0] == '\0' || event->decoder_type[0] == '\0' ||
        event->payload_length > GATEWAY_EVENT_PAYLOAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }

    char topic[MQTT_GATEWAY_TOPIC_MAX];
    int written = snprintf(topic, sizeof(topic), "%s/%s/%s",
                           CONFIG_NETGATE_MQTT_TOPIC_PREFIX,
                           event->device_id, event->topic);
    if (written < 0 || (size_t)written >= sizeof(topic)) {
        return ESP_ERR_INVALID_SIZE;
    }

    char *json = NULL;
    esp_err_t result = encode_event_json(event, &json);
    if (result != ESP_OK) {
        return result;
    }

    int message_id = esp_mqtt_client_publish(s_client, topic, json, 0, 0, 0);
    cJSON_free(json);
    return message_id < 0 ? ESP_FAIL : ESP_OK;
}
