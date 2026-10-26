#ifndef NETGATE_BLE_CONFIG_STORE_H
#define NETGATE_BLE_CONFIG_STORE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define BLE_CONFIG_JSON_MAX_BYTES 4096U

typedef struct {
    uint32_t revision;
    uint32_t checksum;
    int64_t updated_at;
} ble_config_metadata_t;

/**
 * @brief Loads the active BLE JSON document from NVS.
 *
 * @param json Output pointer receiving heap-allocated, null-terminated JSON.
 * @param length Output JSON length in bytes, excluding the terminator.
 * @param metadata Optional output metadata.
 * @return ESP_OK when loaded, otherwise an error code.
 */
esp_err_t ble_config_store_load_active(char **json, size_t *length,
                                       ble_config_metadata_t *metadata);

/**
 * @brief Loads the rollback BLE JSON document from NVS.
 *
 * @param json Output pointer receiving heap-allocated, null-terminated JSON.
 * @param length Output JSON length in bytes, excluding the terminator.
 * @return ESP_OK when loaded, otherwise an error code.
 */
esp_err_t ble_config_store_load_backup(char **json, size_t *length);

/**
 * @brief Persists a validated candidate document before runtime activation.
 *
 * @param json Candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @return ESP_OK when committed to NVS, otherwise an error code.
 */
esp_err_t ble_config_store_stage(const char *json, size_t length);

/**
 * @brief Promotes a runtime-accepted candidate and preserves the previous active document.
 *
 * @param json Accepted candidate JSON bytes.
 * @param length Candidate length in bytes.
 * @param metadata Output committed metadata.
 * @return ESP_OK when committed atomically, otherwise an error code.
 */
esp_err_t ble_config_store_commit(const char *json, size_t length,
                                  ble_config_metadata_t *metadata);

/**
 * @brief Restores the stored backup document as the active configuration.
 *
 * @return ESP_OK when restored, otherwise an error code.
 */
esp_err_t ble_config_store_restore_backup(void);

/**
 * @brief Removes an uncommitted candidate from NVS.
 *
 * @return ESP_OK when absent or removed, otherwise an error code.
 */
esp_err_t ble_config_store_discard_candidate(void);

#endif
