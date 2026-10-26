#ifndef NETGATE_BLE_CONFIG_MANAGER_H
#define NETGATE_BLE_CONFIG_MANAGER_H

#include "esp_err.h"

/**
 * @brief Loads the persisted active BLE configuration or the embedded fallback.
 *
 * @return ESP_OK when one valid profile set is active, otherwise an error code.
 */
esp_err_t ble_config_manager_load_initial(void);

/**
 * @brief Starts the remote BLE configuration worker and MQTT input binding.
 *
 * @return ESP_OK when initialized, otherwise an error code.
 */
esp_err_t ble_config_manager_init(void);

#endif
