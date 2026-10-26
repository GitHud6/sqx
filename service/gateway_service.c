#include "gateway_service.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gateway_event.h"
#include "mqtt_gateway.h"

#define GATEWAY_SERVICE_QUEUE_LENGTH 16
#define GATEWAY_SERVICE_TASK_STACK_SIZE 4096
#define GATEWAY_SERVICE_TASK_PRIORITY 5

static const char *TAG = "gateway_service";
static QueueHandle_t s_event_queue;

static esp_err_t gateway_service_enqueue(const gateway_event_t *event, void *context);
static void gateway_service_task(void *argument);

/**
 * @brief Copies a protocol event into the bounded gateway service queue.
 *
 * @param event Event produced by a protocol decoder.
 * @param context Unused event sink context.
 * @return ESP_OK when queued, otherwise an error code.
 */
static esp_err_t gateway_service_enqueue(const gateway_event_t *event, void *context)
{
    (void)context;
    if (event == NULL || event->payload_length > GATEWAY_EVENT_PAYLOAD_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(s_event_queue, event, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

/**
 * @brief Publishes queued events through MQTT outside BLE callback context.
 *
 * @param argument Unused FreeRTOS task argument.
 * @return None.
 */
static void gateway_service_task(void *argument)
{
    (void)argument;
    gateway_event_t event;
    while (true) {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            esp_err_t result = mqtt_gateway_publish_event(&event);
            if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "MQTT event delivery failed: %s", esp_err_to_name(result));
            }
        }
    }
}

/**
 * @brief Starts the event queue and installs it as the protocol event sink.
 *
 * @return ESP_OK when the service task starts, otherwise an error code.
 */
esp_err_t gateway_service_init(void)
{
    if (s_event_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_event_queue = xQueueCreate(GATEWAY_SERVICE_QUEUE_LENGTH, sizeof(gateway_event_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(gateway_service_task, "gateway_service",
                    GATEWAY_SERVICE_TASK_STACK_SIZE, NULL,
                    GATEWAY_SERVICE_TASK_PRIORITY, NULL) != pdPASS) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return gateway_event_set_sink(gateway_service_enqueue, NULL);
}
