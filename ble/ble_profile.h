#ifndef NETGATE_BLE_PROFILE_H
#define NETGATE_BLE_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "host/ble_uuid.h"

#define BLE_PROFILE_MAX_NAME 32
#define BLE_PROFILE_MAX_TOPIC 64
#define BLE_PROFILE_MAX_UNIT 16
#define BLE_PROFILE_MAX_UUID_TEXT 40
#define BLE_PROFILE_MAX_DEVICES 8
#define BLE_PROFILE_MAX_SERVICES 4
#define BLE_PROFILE_MAX_CHARACTERISTICS 16

typedef enum {
    BLE_DECODER_RAW,
    BLE_DECODER_UINT8,
    BLE_DECODER_UINT16,
    BLE_DECODER_INT16,
    BLE_DECODER_UINT32,
} ble_decoder_type_t;

typedef enum {
    BLE_BYTE_ORDER_NONE,
    BLE_BYTE_ORDER_LITTLE_ENDIAN,
    BLE_BYTE_ORDER_BIG_ENDIAN,
} ble_byte_order_t;

typedef struct {
    ble_decoder_type_t type;
    ble_byte_order_t byte_order;
    uint16_t offset;
    uint16_t length;
    double scale;
    char unit[BLE_PROFILE_MAX_UNIT];
} ble_decoder_profile_t;

typedef struct {
    char id[BLE_PROFILE_MAX_NAME];
    char uuid_text[BLE_PROFILE_MAX_UUID_TEXT];
    ble_uuid128_t uuid;
    uint8_t required_properties;
    uint8_t properties;
    char callback_set[BLE_PROFILE_MAX_NAME];
    char event_type[BLE_PROFILE_MAX_NAME];
    char topic[BLE_PROFILE_MAX_TOPIC];
    ble_decoder_profile_t decoder;
} ble_characteristic_profile_t;

typedef struct {
    char uuid_text[BLE_PROFILE_MAX_UUID_TEXT];
    ble_uuid128_t uuid;
    size_t characteristic_count;
    ble_characteristic_profile_t *characteristics;
} ble_service_profile_t;

typedef struct ble_profile {
    char device_id[BLE_PROFILE_MAX_NAME];
    char advertising_name[BLE_PROFILE_MAX_NAME];
    char advertised_service_uuid_text[BLE_PROFILE_MAX_UUID_TEXT];
    ble_uuid128_t advertised_service_uuid;
    char driver_name[BLE_PROFILE_MAX_NAME];
    uint8_t match_address[6];
    uint8_t manufacturer_data_prefix[16];
    uint8_t manufacturer_data_prefix_length;
    bool has_match_address;
    size_t service_count;
    ble_service_profile_t *services;
} ble_profile_t;

/**
 * @brief Loads and validates device profiles from the embedded JSON document.
 *
 * @return ESP_OK when all profiles are loaded, otherwise an error code.
 */
esp_err_t ble_profile_load(void);

/**
 * @brief Returns the embedded fallback JSON document without copying it.
 *
 * @param json Output pointer receiving the embedded JSON bytes.
 * @param length Output embedded JSON length in bytes.
 * @return ESP_OK when the embedded document is available, otherwise an error code.
 */
esp_err_t ble_profile_get_embedded_json(const char **json, size_t *length);

/**
 * @brief Parses a candidate JSON document without replacing active profiles.
 *
 * @param json Candidate devices.json content.
 * @param length Candidate length in bytes.
 * @return ESP_OK when syntax and schema validation succeeds, otherwise an error code.
 */
esp_err_t ble_profile_stage_json(const char *json, size_t length);

/**
 * @brief Returns the number of profiles in the staged candidate.
 *
 * @return Staged profile count.
 */
size_t ble_profile_staged_count(void);

/**
 * @brief Returns one staged candidate profile by index.
 *
 * @param index Zero-based staged profile index.
 * @return Staged profile pointer, or NULL when the index is out of range.
 */
const ble_profile_t *ble_profile_staged_at(size_t index);

/**
 * @brief Replaces active profiles with the staged candidate and preserves the old bank for rollback.
 *
 * @return ESP_OK when a staged candidate is committed, otherwise an error code.
 */
esp_err_t ble_profile_commit_staged(void);

/**
 * @brief Restores the active profile set saved by the last staged commit.
 *
 * @return ESP_OK when restored, otherwise an error code.
 */
esp_err_t ble_profile_rollback(void);

/**
 * @brief Releases the inactive profile bank and clears rollback availability.
 *
 * @return None.
 */
void ble_profile_discard_staged(void);

/**
 * @brief Finds one loaded profile by its device identifier.
 *
 * @param device_id Device identifier declared in devices.json.
 * @return Matching profile, or NULL when no profile exists.
 */
ble_profile_t *ble_profile_find(const char *device_id);

/**
 * @brief Returns the number of profiles loaded from devices.json.
 *
 * @return Loaded profile count.
 */
size_t ble_profile_count(void);

/**
 * @brief Returns one loaded profile by index.
 *
 * @param index Zero-based profile index.
 * @return Profile pointer, or NULL when the index is out of range.
 */
ble_profile_t *ble_profile_at(size_t index);

/**
 * @brief Finds one configured characteristic by its JSON identifier.
 *
 * @param profile Loaded device profile.
 * @param characteristic_id JSON characteristic identifier.
 * @return Matching characteristic, or NULL when it is not declared.
 */
const ble_characteristic_profile_t *ble_profile_find_characteristic(
    const ble_profile_t *profile, const char *characteristic_id);

#endif
