#include "ble_config_manager.h"

#include <stdlib.h>
#include <string.h>

#include "ble_bus.h"
#include "ble_config_store.h"
#include "ble_driver_registry.h"
#include "ble_profile.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_gateway.h"

#define BLE_CONFIG_QUEUE_LENGTH 2U
#define BLE_CONFIG_TASK_STACK_SIZE 6144U
#define BLE_CONFIG_TASK_PRIORITY 5U
#define BLE_CONFIG_SUSPEND_TIMEOUT_MS 5000U

typedef struct {
    char *json;
    size_t length;
} ble_config_request_t;

static const char *TAG = "ble_config_manager";
static QueueHandle_t s_request_queue;

static esp_err_t enqueue_candidate(const char *json, size_t length, void *context);
static void config_task(void *argument);
static esp_err_t validate_staged_profiles(void);
static esp_err_t validate_active_profiles(void);
static esp_err_t register_active_profiles(void);
static esp_err_t persist_embedded_baseline(void);
static esp_err_t apply_candidate(const char *json, size_t length,
                                 ble_config_metadata_t *metadata);
static esp_err_t restore_runtime_profiles(bool profile_committed);

/**
 * @brief Validates that one profile is backed by a registered driver and callbacks.
 *
 * @param profile Profile to validate.
 * @return ESP_OK when all bindings exist, otherwise an error code.
 */
static esp_err_t validate_profile(const ble_profile_t *profile)
{
    if (profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const ble_driver_t *driver = ble_driver_registry_find(profile->driver_name);
    if (driver == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    for (size_t service_index = 0; service_index < profile->service_count; ++service_index) {
        const ble_service_profile_t *service = &profile->services[service_index];
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            if (ble_driver_find_callback(driver,
                                         service->characteristics[index].callback_set) == NULL) {
                return ESP_ERR_NOT_FOUND;
            }
        }
    }
    return ESP_OK;
}

/**
 * @brief Validates all isolated staged profiles against the driver registry.
 *
 * @return ESP_OK when valid, otherwise an error code.
 */
static esp_err_t validate_staged_profiles(void)
{
    if (ble_profile_staged_count() == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    for (size_t index = 0; index < ble_profile_staged_count(); ++index) {
        esp_err_t result = validate_profile(ble_profile_staged_at(index));
        if (result != ESP_OK) {
            return result;
        }
    }
    return ESP_OK;
}

/**
 * @brief Validates all active profiles against the driver registry.
 *
 * @return ESP_OK when valid, otherwise an error code.
 */
static esp_err_t validate_active_profiles(void)
{
    if (ble_profile_count() == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    for (size_t index = 0; index < ble_profile_count(); ++index) {
        esp_err_t result = validate_profile(ble_profile_at(index));
        if (result != ESP_OK) {
            return result;
        }
    }
    return ESP_OK;
}

/**
 * @brief Registers every active profile into empty fixed BLE slots.
 *
 * @return ESP_OK when registered, otherwise an error code.
 */
static esp_err_t register_active_profiles(void)
{
    for (size_t index = 0; index < ble_profile_count(); ++index) {
        ble_profile_t *profile = ble_profile_at(index);
        const ble_driver_t *driver = profile == NULL ? NULL :
            ble_driver_registry_find(profile->driver_name);
        esp_err_t result = ble_bus_register_profile(profile, driver);
        if (result != ESP_OK) {
            return result;
        }
    }
    return ESP_OK;
}

/**
 * @brief Loads and validates one stored JSON document into active profile storage.
 *
 * @param json Stored JSON bytes.
 * @param length JSON length in bytes.
 * @return ESP_OK when activated, otherwise an error code.
 */
static esp_err_t activate_initial_json(const char *json, size_t length)
{
    esp_err_t result = ble_profile_stage_json(json, length);
    if (result == ESP_OK) {
        result = validate_staged_profiles();
    }
    if (result == ESP_OK) {
        result = ble_profile_commit_staged();
        if (result == ESP_OK) {
            ble_profile_discard_staged();
        }
    } else {
        ble_profile_discard_staged();
    }
    return result;
}

/**
 * @brief Persists the validated embedded configuration as the first rollback baseline.
 *
 * @return ESP_OK when the baseline is committed, otherwise an error code.
 */
static esp_err_t persist_embedded_baseline(void)
{
    const char *json = NULL;
    size_t length = 0;
    esp_err_t result = ble_profile_get_embedded_json(&json, &length);
    if (result == ESP_OK) {
        result = ble_config_store_stage(json, length);
    }
    if (result == ESP_OK) {
        ble_config_metadata_t metadata = {0};
        result = ble_config_store_commit(json, length, &metadata);
    }
    if (result != ESP_OK) {
        ble_config_store_discard_candidate();
    }
    return result;
}

/**
 * @brief Loads NVS active, NVS backup, or embedded fallback profiles in that order.
 *
 * @return ESP_OK when one valid profile set is active, otherwise an error code.
 */
esp_err_t ble_config_manager_load_initial(void)
{
    char *json = NULL;
    size_t length = 0;
    ble_config_metadata_t metadata = {0};
    esp_err_t result = ble_config_store_load_active(&json, &length, &metadata);
    if (result == ESP_OK) {
        uint32_t checksum = esp_crc32_le(0, (const uint8_t *)json, (uint32_t)length);
        if (checksum == metadata.checksum) {
            result = activate_initial_json(json, length);
        } else {
            result = ESP_ERR_INVALID_CRC;
        }
        free(json);
        json = NULL;
        if (result == ESP_OK) {
            esp_err_t discard_result = ble_config_store_discard_candidate();
            if (discard_result != ESP_OK) {
                ESP_LOGW(TAG, "Cannot discard stale BLE candidate: %s",
                         esp_err_to_name(discard_result));
            }
            ESP_LOGI(TAG, "Loaded BLE config revision=%lu", (unsigned long)metadata.revision);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Active BLE config invalid: %s", esp_err_to_name(result));
    }

    result = ble_config_store_load_backup(&json, &length);
    if (result == ESP_OK) {
        result = activate_initial_json(json, length);
        free(json);
        json = NULL;
        if (result == ESP_OK) {
            esp_err_t restore_result = ble_config_store_restore_backup();
            if (restore_result != ESP_OK) {
                ESP_LOGE(TAG, "Cannot promote recovered BLE backup: %s",
                         esp_err_to_name(restore_result));
                return restore_result;
            }
            ESP_LOGW(TAG, "Recovered BLE config from NVS backup");
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Backup BLE config invalid: %s", esp_err_to_name(result));
    }

    result = ble_profile_load();
    if (result == ESP_OK) {
        result = validate_active_profiles();
    }
    if (result == ESP_OK) {
        result = persist_embedded_baseline();
    }
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Using embedded BLE config fallback with NVS baseline");
    }
    return result;
}

/**
 * @brief Copies an MQTT candidate into the bounded configuration work queue.
 *
 * @param json Candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @param context Unused handler context.
 * @return ESP_OK when queued, otherwise an error code.
 */
static esp_err_t enqueue_candidate(const char *json, size_t length, void *context)
{
    (void)context;
    if (s_request_queue == NULL || json == NULL || length == 0 ||
        length > BLE_CONFIG_JSON_MAX_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    char *copy = malloc(length + 1U);
    if (copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, json, length);
    copy[length] = '\0';
    ble_config_request_t request = {.json = copy, .length = length};
    if (xQueueSend(s_request_queue, &request, 0) != pdTRUE) {
        free(copy);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/**
 * @brief Restores RAM profiles and BLE slots after an activation failure.
 *
 * @param profile_committed True when active profiles were already replaced.
 * @return ESP_OK when the previous runtime configuration is restored, otherwise an error code.
 */
static esp_err_t restore_runtime_profiles(bool profile_committed)
{
    esp_err_t result = ble_bus_clear_profiles();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Cannot clear failed profile slots: %s", esp_err_to_name(result));
        return result;
    }
    if (profile_committed) {
        result = ble_profile_rollback();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Profile rollback failed: %s", esp_err_to_name(result));
            return result;
        }
    }
    result = register_active_profiles();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Rollback profile registration failed: %s", esp_err_to_name(result));
        return result;
    }
    result = ble_bus_resume();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "BLE resume after rollback failed: %s", esp_err_to_name(result));
    }
    return result;
}

/**
 * @brief Applies one validated candidate transaction with runtime and NVS rollback.
 *
 * @param json Candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @param metadata Output committed metadata.
 * @return ESP_OK when activated and persisted, otherwise an error code.
 */
static esp_err_t apply_candidate(const char *json, size_t length,
                                 ble_config_metadata_t *metadata)
{
    esp_err_t result = ble_profile_stage_json(json, length);
    if (result == ESP_OK) {
        result = validate_staged_profiles();
    }
    if (result != ESP_OK) {
        ble_profile_discard_staged();
        return result;
    }
    result = ble_config_store_stage(json, length);
    if (result != ESP_OK) {
        ble_profile_discard_staged();
        return result;
    }
    result = ble_bus_suspend(BLE_CONFIG_SUSPEND_TIMEOUT_MS);
    if (result != ESP_OK) {
        ble_profile_discard_staged();
        ble_config_store_discard_candidate();
        ble_bus_resume();
        return result;
    }
    result = ble_bus_clear_profiles();
    if (result != ESP_OK) {
        ble_profile_discard_staged();
        ble_config_store_discard_candidate();
        ble_bus_resume();
        return result;
    }

    bool profile_committed = false;
    result = ble_profile_commit_staged();
    if (result == ESP_OK) {
        profile_committed = true;
        result = register_active_profiles();
    }
    if (result == ESP_OK) {
        result = ble_config_store_commit(json, length, metadata);
    }
    if (result != ESP_OK) {
        ble_config_store_discard_candidate();
        esp_err_t restore_result = restore_runtime_profiles(profile_committed);
        return restore_result == ESP_OK ? result : restore_result;
    }
    result = ble_bus_resume();
    if (result != ESP_OK) {
        esp_err_t nvs_restore_result = ble_config_store_restore_backup();
        if (nvs_restore_result != ESP_OK) {
            ESP_LOGE(TAG, "NVS rollback failed: %s",
                     esp_err_to_name(nvs_restore_result));
        }
        esp_err_t runtime_restore_result = restore_runtime_profiles(true);
        if (runtime_restore_result != ESP_OK) {
            return runtime_restore_result;
        }
        if (nvs_restore_result != ESP_OK) {
            return nvs_restore_result;
        }
    } else {
        ble_profile_discard_staged();
    }
    return result;
}

/**
 * @brief Processes queued configuration transactions outside MQTT callback context.
 *
 * @param argument Unused task argument.
 * @return None.
 */
static void config_task(void *argument)
{
    (void)argument;
    ble_config_request_t request;
    while (true) {
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) == pdTRUE) {
            ble_config_metadata_t metadata = {0};
            esp_err_t result = apply_candidate(request.json, request.length, &metadata);
            ESP_LOGI(TAG, "Remote BLE config result=%s revision=%lu",
                     esp_err_to_name(result), (unsigned long)metadata.revision);
            mqtt_gateway_publish_config_status(result == ESP_OK, result,
                                               result == ESP_OK ? metadata.revision : 0);
            free(request.json);
        }
    }
}

/**
 * @brief Starts the configuration queue, worker, and MQTT handler binding.
 *
 * @return ESP_OK when initialized, otherwise an error code.
 */
esp_err_t ble_config_manager_init(void)
{
    if (s_request_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_request_queue = xQueueCreate(BLE_CONFIG_QUEUE_LENGTH, sizeof(ble_config_request_t));
    if (s_request_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(config_task, "ble_config", BLE_CONFIG_TASK_STACK_SIZE,
                    NULL, BLE_CONFIG_TASK_PRIORITY, NULL) != pdPASS) {
        vQueueDelete(s_request_queue);
        s_request_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return mqtt_gateway_set_config_handler(enqueue_candidate, NULL);
}
