#ifndef NETGATE_NETWORK_MANAGER_H
#define NETGATE_NETWORK_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initializes optional MQTT support before network startup.
 *
 * @return ESP_OK when network control state is ready, otherwise an error code.
 */
esp_err_t network_manager_init(void);

/**
 * @brief Starts the Wi-Fi station and binds IP availability to MQTT lifecycle.
 *
 * @return ESP_OK when Wi-Fi startup is requested, otherwise an error code.
 */
esp_err_t network_manager_start(void);

#endif
