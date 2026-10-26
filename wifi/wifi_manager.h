#ifndef NETGATE_WIFI_MANAGER_H
#define NETGATE_WIFI_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

typedef void (*wifi_manager_connection_handler_t)(bool connected, void *context);

typedef void (*wifi_manager_provisioning_handler_t)(bool connected, void *context);

/**
 * @brief Initializes and starts the configured Wi-Fi station.
 *
 * @param handler Callback invoked when the station gains or loses an IP address.
 * @param context Opaque callback context.
 * @return ESP_OK when Wi-Fi starts, otherwise an error code.
 */
esp_err_t wifi_manager_init(wifi_manager_connection_handler_t handler, void *context);

/**
 * @brief Reports whether the station currently owns an IPv4 address.
 *
 * @return true when connected with an IP address, otherwise false.
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Registers an observer for provisioning status updates.
 *
 * @param handler Callback invoked when Wi-Fi provisioning state changes.
 * @param context Opaque callback context.
 * @return None.
 */
void wifi_manager_set_provisioning_handler(wifi_manager_provisioning_handler_t handler,
                                           void *context);

/**
 * @brief Applies candidate Wi-Fi credentials and starts an asynchronous connection attempt.
 *
 * @param ssid Candidate access point name.
 * @param password Candidate WPA password.
 * @return ESP_OK when the attempt was requested, otherwise an error code.
 */
esp_err_t wifi_manager_apply_credentials(const char *ssid, const char *password);

#endif
