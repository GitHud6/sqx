#ifndef NETGATE_GATEWAY_SERVICE_H
#define NETGATE_GATEWAY_SERVICE_H

#include "esp_err.h"

/**
 * @brief Starts the event queue and installs it as the protocol event sink.
 *
 * @return ESP_OK when the service task starts, otherwise an error code.
 */
esp_err_t gateway_service_init(void);

#endif
