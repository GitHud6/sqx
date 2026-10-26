#include "gateway_event.h"

static gateway_event_sink_fn s_sink;
static void *s_context;

/**
 * @brief Installs the consumer for normalized protocol events.
 *
 * @param sink Event consumer, or NULL to disable delivery.
 * @param context Opaque consumer context.
 * @return ESP_OK when the sink is stored, otherwise an error code.
 */
esp_err_t gateway_event_set_sink(gateway_event_sink_fn sink, void *context)
{
    s_sink = sink;
    s_context = context;
    return ESP_OK;
}

/**
 * @brief Publishes one normalized event to the installed consumer.
 *
 * @param event Event to deliver.
 * @return ESP_OK when delivered, ESP_ERR_INVALID_STATE when no sink exists,
 *         or another error returned by the sink.
 */
esp_err_t gateway_event_publish(const gateway_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_sink == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return s_sink(event, s_context);
}
