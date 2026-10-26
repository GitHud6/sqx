#include "esp_log.h"
#include "nvs_flash.h"

#include "ble_bus.h"
#include "ble_config_manager.h"
#include "ble_profile.h"
#include "ble_driver_registry.h"
#include "ble_device0_driver.h"
#include "generic_gatt_driver.h"
#include "gateway_service.h"
#include "network_manager.h"
#include "ble_provisioning_service.h"
#include "ota_manager.h"

static const char *TAG = "netgate";

/**
 * @brief Initializes gateway services, network transport, BLE drivers, and the BLE bus.
 *
 * @param None.
 * @return None.
 */
void app_main(void)
{
    const char *failure_stage = NULL;
    esp_err_t result = nvs_flash_init();

    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        result = nvs_flash_erase();
        if (result == ESP_OK) {
            result = nvs_flash_init();
        }
    }

    if (result != ESP_OK) {
        failure_stage = "NVS initialization";
        goto startup_failed;
    }

    result = ota_manager_init();
    if (result != ESP_OK) {
        failure_stage = "OTA manager initialization";
        goto startup_failed;
    }

    result = gateway_service_init();
    if (result != ESP_OK) {
        failure_stage = "Gateway service initialization";
        goto startup_failed;
    }

    result = ble_driver_registry_register(&ble_device0_driver);
    if (result != ESP_OK) {
        failure_stage = "Ble_device0 driver registration";
        goto startup_failed;
    }

    result = ble_driver_registry_register(&generic_gatt_driver);
    if (result != ESP_OK) {
        failure_stage = "Generic GATT driver registration";
        goto startup_failed;
    }

    result = ble_config_manager_load_initial();
    if (result != ESP_OK) {
        failure_stage = "BLE device configuration load";
        goto startup_failed;
    }

    for (size_t index = 0; index < ble_profile_count(); ++index) {
        ble_profile_t *profile = ble_profile_at(index);
        const ble_driver_t *driver = profile == NULL ? NULL :
            ble_driver_registry_find(profile->driver_name);
        result = ble_bus_register_profile(profile, driver);
        if (result != ESP_OK) {
            failure_stage = "BLE profile registration";
            goto startup_failed;
        }
    }

    result = ble_bus_init();
    if (result != ESP_OK) {
        failure_stage = "BLE bus initialization";
        goto startup_failed;
    }

    result = ble_provisioning_service_init();
    if (result != ESP_OK) {
        failure_stage = "BLE provisioning service initialization";
        goto startup_failed;
    }

    result = ble_bus_start();
    if (result != ESP_OK) {
        failure_stage = "BLE bus startup";
        goto startup_failed;
    }

    result = ble_config_manager_init();
    if (result != ESP_OK) {
        failure_stage = "BLE configuration manager";
        goto startup_failed;
    }

    result = network_manager_init();
    if (result != ESP_OK) {
        failure_stage = "Network initialization";
        goto startup_failed;
    }

    result = network_manager_start();
    if (result != ESP_OK) {
        failure_stage = "Wi-Fi startup";
        goto startup_failed;
    }

    result = ota_manager_confirm_running_image();
    if (result != ESP_OK) {
        failure_stage = "OTA image confirmation";
        goto startup_failed;
    }
    return;

startup_failed:
    ESP_LOGE(TAG, "Startup failed at %s: %s", failure_stage, esp_err_to_name(result));
    ota_manager_reject_running_image();
}
