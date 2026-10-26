#ifndef NETGATE_BLE_PROVISIONING_SERVICE_H
#define NETGATE_BLE_PROVISIONING_SERVICE_H

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Registers the BLE GATT service used to provision Wi-Fi credentials.
 *
 * @return ESP_OK when the service is registered, otherwise an error code.
 */
esp_err_t ble_provisioning_service_init(void);

/**
 * @brief Starts connectable provisioning advertisements.
 *
 * @return ESP_OK when advertising starts, otherwise an error code.
 */
esp_err_t ble_provisioning_service_start(void);

/**
 * @brief Publishes the current provisioning state through the Status characteristic.
 *
 * @param connected True when Wi-Fi owns an IPv4 address.
 * @return None.
 */
void ble_provisioning_service_set_wifi_state(bool connected);

#endif
