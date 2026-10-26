#include "ble_config_store.h"

#include <stdlib.h>
#include <time.h>

#include "esp_crc.h"
#include "nvs.h"

#define BLE_CONFIG_NAMESPACE "netgate_cfg"
#define BLE_CONFIG_ACTIVE_KEY "active"
#define BLE_CONFIG_BACKUP_KEY "backup"
#define BLE_CONFIG_CANDIDATE_KEY "candidate"
#define BLE_CONFIG_REVISION_KEY "revision"
#define BLE_CONFIG_CHECKSUM_KEY "checksum"
#define BLE_CONFIG_UPDATED_AT_KEY "updated_at"

/**
 * @brief Loads one bounded JSON blob from an open NVS namespace.
 *
 * @param handle Open NVS handle.
 * @param key Blob key.
 * @param json Output heap-allocated, null-terminated JSON.
 * @param length Output JSON byte length.
 * @return ESP_OK when loaded, otherwise an error code.
 */
static esp_err_t load_blob(nvs_handle_t handle, const char *key,
                           char **json, size_t *length)
{
    if (key == NULL || json == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *json = NULL;
    *length = 0;
    size_t stored_length = 0;
    esp_err_t result = nvs_get_blob(handle, key, NULL, &stored_length);
    if (result != ESP_OK) {
        return result;
    }
    if (stored_length == 0 || stored_length > BLE_CONFIG_JSON_MAX_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }
    char *buffer = malloc(stored_length + 1U);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    result = nvs_get_blob(handle, key, buffer, &stored_length);
    if (result != ESP_OK) {
        free(buffer);
        return result;
    }
    buffer[stored_length] = '\0';
    *json = buffer;
    *length = stored_length;
    return ESP_OK;
}

/**
 * @brief Loads the active JSON and optional metadata from NVS.
 *
 * @param json Output heap-allocated JSON.
 * @param length Output JSON byte length.
 * @param metadata Optional output metadata.
 * @return ESP_OK when loaded, otherwise an error code.
 */
esp_err_t ble_config_store_load_active(char **json, size_t *length,
                                       ble_config_metadata_t *metadata)
{
    if (json == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = load_blob(handle, BLE_CONFIG_ACTIVE_KEY, json, length);
    if (result == ESP_OK && metadata != NULL) {
        result = nvs_get_u32(handle, BLE_CONFIG_REVISION_KEY, &metadata->revision);
        if (result == ESP_OK) {
            result = nvs_get_u32(handle, BLE_CONFIG_CHECKSUM_KEY,
                                 &metadata->checksum);
        }
        if (result == ESP_OK) {
            result = nvs_get_i64(handle, BLE_CONFIG_UPDATED_AT_KEY,
                                 &metadata->updated_at);
        }
        if (result != ESP_OK) {
            free(*json);
            *json = NULL;
            *length = 0;
        }
    }
    nvs_close(handle);
    return result;
}

/**
 * @brief Loads the rollback JSON document from NVS.
 *
 * @param json Output heap-allocated JSON.
 * @param length Output JSON byte length.
 * @return ESP_OK when loaded, otherwise an error code.
 */
esp_err_t ble_config_store_load_backup(char **json, size_t *length)
{
    if (json == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = load_blob(handle, BLE_CONFIG_BACKUP_KEY, json, length);
    nvs_close(handle);
    return result;
}

/**
 * @brief Persists a candidate JSON document in NVS.
 *
 * @param json Candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @return ESP_OK when committed, otherwise an error code.
 */
esp_err_t ble_config_store_stage(const char *json, size_t length)
{
    if (json == NULL || length == 0 || length > BLE_CONFIG_JSON_MAX_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = nvs_set_blob(handle, BLE_CONFIG_CANDIDATE_KEY, json, length);
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

/**
 * @brief Promotes a candidate while preserving the previous active JSON.
 *
 * @param json Accepted candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @param metadata Output committed metadata.
 * @return ESP_OK when committed, otherwise an error code.
 */
esp_err_t ble_config_store_commit(const char *json, size_t length,
                                  ble_config_metadata_t *metadata)
{
    if (json == NULL || length == 0 || length > BLE_CONFIG_JSON_MAX_BYTES || metadata == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }

    char *active = NULL;
    size_t active_length = 0;
    esp_err_t active_result = load_blob(handle, BLE_CONFIG_ACTIVE_KEY,
                                        &active, &active_length);
    if (active_result == ESP_OK) {
        result = nvs_set_blob(handle, BLE_CONFIG_BACKUP_KEY, active, active_length);
    } else if (active_result != ESP_ERR_NVS_NOT_FOUND &&
               active_result != ESP_ERR_INVALID_SIZE) {
        result = active_result;
    }
    free(active);

    uint32_t revision = 0;
    esp_err_t revision_result = nvs_get_u32(handle, BLE_CONFIG_REVISION_KEY, &revision);
    if (revision_result != ESP_OK && revision_result != ESP_ERR_NVS_NOT_FOUND) {
        result = revision_result;
    }
    if (result == ESP_OK) {
        ++revision;
        metadata->revision = revision;
        metadata->checksum = esp_crc32_le(0, (const uint8_t *)json, (uint32_t)length);
        metadata->updated_at = (int64_t)time(NULL);
        result = nvs_set_blob(handle, BLE_CONFIG_ACTIVE_KEY, json, length);
    }
    if (result == ESP_OK) {
        result = nvs_set_u32(handle, BLE_CONFIG_REVISION_KEY, metadata->revision);
    }
    if (result == ESP_OK) {
        result = nvs_set_u32(handle, BLE_CONFIG_CHECKSUM_KEY, metadata->checksum);
    }
    if (result == ESP_OK) {
        result = nvs_set_i64(handle, BLE_CONFIG_UPDATED_AT_KEY, metadata->updated_at);
    }
    if (result == ESP_OK) {
        esp_err_t erase_result = nvs_erase_key(handle, BLE_CONFIG_CANDIDATE_KEY);
        if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
            result = erase_result;
        }
    }
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

/**
 * @brief Restores the backup JSON as active without overwriting the backup.
 *
 * @return ESP_OK when restored, otherwise an error code.
 */
esp_err_t ble_config_store_restore_backup(void)
{
    char *backup = NULL;
    size_t length = 0;
    esp_err_t result = ble_config_store_load_backup(&backup, &length);
    if (result != ESP_OK) {
        return result;
    }
    nvs_handle_t handle;
    result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        free(backup);
        return result;
    }
    result = nvs_set_blob(handle, BLE_CONFIG_ACTIVE_KEY, backup, length);
    uint32_t revision = 0;
    esp_err_t revision_result = nvs_get_u32(handle, BLE_CONFIG_REVISION_KEY, &revision);
    if (result == ESP_OK && revision_result != ESP_OK &&
        revision_result != ESP_ERR_NVS_NOT_FOUND) {
        result = revision_result;
    }
    if (result == ESP_OK) {
        result = nvs_set_u32(handle, BLE_CONFIG_REVISION_KEY, revision + 1U);
    }
    if (result == ESP_OK) {
        uint32_t checksum = esp_crc32_le(0, (const uint8_t *)backup, (uint32_t)length);
        result = nvs_set_u32(handle, BLE_CONFIG_CHECKSUM_KEY, checksum);
    }
    if (result == ESP_OK) {
        result = nvs_set_i64(handle, BLE_CONFIG_UPDATED_AT_KEY, (int64_t)time(NULL));
    }
    if (result == ESP_OK) {
        esp_err_t erase_result = nvs_erase_key(handle, BLE_CONFIG_CANDIDATE_KEY);
        if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
            result = erase_result;
        }
    }
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    free(backup);
    return result;
}

/**
 * @brief Removes an uncommitted candidate key from NVS.
 *
 * @return ESP_OK when absent or removed, otherwise an error code.
 */
esp_err_t ble_config_store_discard_candidate(void)
{
    nvs_handle_t handle;
    esp_err_t result = nvs_open(BLE_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = nvs_erase_key(handle, BLE_CONFIG_CANDIDATE_KEY);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        result = ESP_OK;
    } else if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}
