#ifndef NETGATE_OTA_MANAGER_H
#define NETGATE_OTA_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initializes the OTA worker and registers its MQTT command handler.
 *
 * @return ESP_OK when the worker is ready, otherwise an error code.
 */
esp_err_t ota_manager_init(void);

/**
 * @brief Confirms a pending OTA image after critical services initialize.
 *
 * @return ESP_OK when no confirmation is needed or the image is accepted.
 */
esp_err_t ota_manager_confirm_running_image(void);

/**
 * @brief Rejects a pending image and reboots into the previous OTA slot.
 *
 * @return ESP_OK when no pending image exists, otherwise an error code.
 */
esp_err_t ota_manager_reject_running_image(void);

#endif
