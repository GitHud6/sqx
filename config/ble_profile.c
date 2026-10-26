#include "ble_profile.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"

extern const uint8_t _binary_devices_json_start[];
extern const uint8_t _binary_devices_json_end[];

typedef struct {
    ble_profile_t *profiles;
    size_t count;
} ble_profile_bank_t;

static ble_profile_bank_t s_profile_banks[2];
static ble_profile_bank_t *s_active_bank = &s_profile_banks[0];
static ble_profile_bank_t *s_staged_bank = &s_profile_banks[1];
static bool s_rollback_available;

static esp_err_t copy_string(char *destination, size_t capacity, const cJSON *item);
static int hex_value(char character);
static esp_err_t parse_match(ble_profile_t *profile, const cJSON *item);
static esp_err_t parse_uuid(ble_uuid128_t *destination, char *text, size_t capacity,
                            const cJSON *item);
static esp_err_t parse_properties(const cJSON *items, uint8_t *properties);
static esp_err_t parse_u16(const cJSON *item, uint16_t *value);
static esp_err_t parse_decoder(ble_decoder_profile_t *decoder, const cJSON *item);
static bool is_mqtt_topic_segment(const char *text);
static esp_err_t parse_profiles(const char *json, size_t length,
                                ble_profile_bank_t *bank);
static esp_err_t validate_profile_uniqueness(const ble_profile_t *profile);
static void free_profile_bank(ble_profile_bank_t *bank);

/**
 * @brief Converts one hexadecimal character to its numeric value.
 *
 * @param character Input hexadecimal character.
 * @return Value from 0 to 15, or -1 when invalid.
 */
static int hex_value(char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    if (character >= 'a' && character <= 'f') {
        return character - 'a' + 10;
    }
    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }
    return -1;
}

/**
 * @brief Parses optional address and manufacturer-data match constraints.
 *
 * @param profile Profile receiving parsed constraints.
 * @param item Optional JSON match object.
 * @return ESP_OK when valid, otherwise an error code.
 */
static esp_err_t parse_match(ble_profile_t *profile, const cJSON *item)
{
    if (profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (item == NULL) {
        return ESP_OK;
    }
    if (!cJSON_IsObject(item)) {
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *address = cJSON_GetObjectItem(item, "address");
    if (address != NULL) {
        if (!cJSON_IsString(address) || address->valuestring == NULL ||
            strlen(address->valuestring) != 17) {
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t index = 0; index < 6; ++index) {
            size_t text_index = index * 3;
            int high = hex_value(address->valuestring[text_index]);
            int low = hex_value(address->valuestring[text_index + 1]);
            if (high < 0 || low < 0 ||
                (index < 5 && address->valuestring[text_index + 2] != ':')) {
                return ESP_ERR_INVALID_ARG;
            }
            profile->match_address[5 - index] = (uint8_t)((high << 4) | low);
        }
        profile->has_match_address = true;
    }

    const cJSON *prefix = cJSON_GetObjectItem(item, "manufacturer_data_prefix");
    if (prefix != NULL) {
        if (!cJSON_IsString(prefix) || prefix->valuestring == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        size_t text_length = strlen(prefix->valuestring);
        if (text_length == 0 || text_length > sizeof(profile->manufacturer_data_prefix) * 2 ||
            (text_length & 1U) != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t index = 0; index < text_length / 2; ++index) {
            int high = hex_value(prefix->valuestring[index * 2]);
            int low = hex_value(prefix->valuestring[index * 2 + 1]);
            if (high < 0 || low < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            profile->manufacturer_data_prefix[index] = (uint8_t)((high << 4) | low);
        }
        profile->manufacturer_data_prefix_length = (uint8_t)(text_length / 2);
    }
    return ESP_OK;
}

/**
 * @brief Releases all dynamically sized services and characteristics in one bank.
 *
 * @param bank Profile bank to release.
 * @return None.
 */
static void free_profile_bank(ble_profile_bank_t *bank)
{
    if (bank == NULL) {
        return;
    }
    for (size_t profile_index = 0; profile_index < bank->count; ++profile_index) {
        ble_profile_t *profile = &bank->profiles[profile_index];
        for (size_t service_index = 0;
             service_index < profile->service_count; ++service_index) {
            free(profile->services[service_index].characteristics);
        }
        free(profile->services);
    }
    free(bank->profiles);
    bank->profiles = NULL;
    bank->count = 0;
}

/**
 * @brief Copies one JSON string to a bounded destination buffer.
 *
 * @param destination Destination buffer.
 * @param capacity Destination capacity in bytes.
 * @param item JSON string value.
 * @return ESP_OK on success, otherwise an error code.
 */
static esp_err_t copy_string(char *destination, size_t capacity, const cJSON *item)
{
    if (destination == NULL || capacity == 0 || !cJSON_IsString(item) ||
        item->valuestring == NULL || strlen(item->valuestring) >= capacity) {
        return ESP_ERR_INVALID_ARG;
    }
    strcpy(destination, item->valuestring);
    return ESP_OK;
}

/**
 * @brief Parses a JSON UUID string into NimBLE UUID storage.
 *
 * @param destination Destination NimBLE UUID storage.
 * @param text Textual UUID storage.
 * @param capacity Textual UUID storage capacity.
 * @param item JSON UUID string.
 * @return ESP_OK on success, otherwise an error code.
 */
static esp_err_t parse_uuid(ble_uuid128_t *destination, char *text, size_t capacity,
                            const cJSON *item)
{
    if (destination == NULL || text == NULL ||
        copy_string(text, capacity, item) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    ble_uuid_any_t parsed = {0};
    if (ble_uuid_from_str(&parsed, text) != 0 || parsed.u.type != BLE_UUID_TYPE_128) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(destination, &parsed.u128, sizeof(*destination));
    return ESP_OK;
}

/**
 * @brief Converts a JSON property array to NimBLE characteristic property bits.
 *
 * @param items JSON property string array.
 * @param properties Output characteristic property bits.
 * @return ESP_OK on success, otherwise an error code.
 */
static esp_err_t parse_properties(const cJSON *items, uint8_t *properties)
{
    if (properties == NULL || !cJSON_IsArray(items)) {
        return ESP_ERR_INVALID_ARG;
    }
    *properties = 0;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, items) {
        if (!cJSON_IsString(item) || item->valuestring == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
        if (strcmp(item->valuestring, "read") == 0) {
            *properties |= BLE_GATT_CHR_PROP_READ;
        } else if (strcmp(item->valuestring, "write") == 0) {
            *properties |= BLE_GATT_CHR_PROP_WRITE;
        } else if (strcmp(item->valuestring, "write_without_response") == 0) {
            *properties |= BLE_GATT_CHR_PROP_WRITE_NO_RSP;
        } else if (strcmp(item->valuestring, "notify") == 0) {
            *properties |= BLE_GATT_CHR_PROP_NOTIFY;
        } else {
            return ESP_ERR_NOT_SUPPORTED;
        }
    }
    return *properties == 0 ? ESP_ERR_INVALID_ARG : ESP_OK;
}

/**
 * @brief Parses a non-negative JSON number representable as uint16_t.
 *
 * @param item JSON number.
 * @param value Output integer value.
 * @return ESP_OK when parsed, otherwise an error code.
 */
static esp_err_t parse_u16(const cJSON *item, uint16_t *value)
{
    if (value == NULL || !cJSON_IsNumber(item) || item->valuedouble < 0 ||
        item->valuedouble > UINT16_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t parsed = (uint16_t)item->valuedouble;
    if ((double)parsed != item->valuedouble) {
        return ESP_ERR_INVALID_ARG;
    }
    *value = parsed;
    return ESP_OK;
}

/**
 * @brief Parses one JSON decoder declaration into bounded runtime metadata.
 *
 * @param decoder Output decoder metadata.
 * @param item Optional JSON decoder object.
 * @return ESP_OK when parsed, otherwise an error code.
 */
static esp_err_t parse_decoder(ble_decoder_profile_t *decoder, const cJSON *item)
{
    if (decoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(decoder, 0, sizeof(*decoder));
    decoder->type = BLE_DECODER_RAW;
    decoder->byte_order = BLE_BYTE_ORDER_NONE;
    decoder->scale = 1.0;

    if (item == NULL) {
        return ESP_OK;
    }
    if (!cJSON_IsObject(item)) {
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *type = cJSON_GetObjectItem(item, "type");
    if (!cJSON_IsString(type) || type->valuestring == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(type->valuestring, "raw") == 0) {
        return ESP_OK;
    }
    if (strcmp(type->valuestring, "uint8") == 0) {
        decoder->type = BLE_DECODER_UINT8;
        decoder->length = 1;
    } else if (strcmp(type->valuestring, "uint16") == 0) {
        decoder->type = BLE_DECODER_UINT16;
        decoder->length = 2;
    } else if (strcmp(type->valuestring, "int16") == 0) {
        decoder->type = BLE_DECODER_INT16;
        decoder->length = 2;
    } else if (strcmp(type->valuestring, "uint32") == 0) {
        decoder->type = BLE_DECODER_UINT32;
        decoder->length = 4;
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (parse_u16(cJSON_GetObjectItem(item, "offset"), &decoder->offset) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    const cJSON *length = cJSON_GetObjectItem(item, "length");
    if (length != NULL) {
        uint16_t configured_length = 0;
        if (parse_u16(length, &configured_length) != ESP_OK ||
            configured_length != decoder->length) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    const cJSON *byte_order = cJSON_GetObjectItem(item, "byte_order");
    if (byte_order == NULL ||
        (cJSON_IsString(byte_order) && strcmp(byte_order->valuestring, "little") == 0)) {
        decoder->byte_order = BLE_BYTE_ORDER_LITTLE_ENDIAN;
    } else if (cJSON_IsString(byte_order) && strcmp(byte_order->valuestring, "big") == 0) {
        decoder->byte_order = BLE_BYTE_ORDER_BIG_ENDIAN;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    const cJSON *scale = cJSON_GetObjectItem(item, "scale");
    if (scale != NULL) {
        if (!cJSON_IsNumber(scale) || !isfinite(scale->valuedouble)) {
            return ESP_ERR_INVALID_ARG;
        }
        decoder->scale = scale->valuedouble;
    }
    const cJSON *unit = cJSON_GetObjectItem(item, "unit");
    return unit == NULL ? ESP_OK : copy_string(decoder->unit, sizeof(decoder->unit), unit);
}

/**
 * @brief Validates one single-level MQTT topic routing segment.
 *
 * @param text Topic segment text.
 * @return true when the segment is non-empty and contains no MQTT separators or wildcards.
 */
static bool is_mqtt_topic_segment(const char *text)
{
    if (text == NULL || text[0] == '\0') {
        return false;
    }
    for (const char *character = text; *character != '\0'; ++character) {
        if (*character == '/' || *character == '+' || *character == '#') {
            return false;
        }
    }
    return true;
}

/**
 * @brief Rejects identifiers and UUID declarations that are ambiguous at runtime.
 *
 * @param profile Parsed device profile.
 * @return ESP_OK when unique, otherwise an error code.
 */
static esp_err_t validate_profile_uniqueness(const ble_profile_t *profile)
{
    if (profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t service_index = 0; service_index < profile->service_count; ++service_index) {
        const ble_service_profile_t *service = &profile->services[service_index];
        for (size_t other_service = service_index + 1;
             other_service < profile->service_count; ++other_service) {
            if (ble_uuid_cmp(&service->uuid.u,
                             &profile->services[other_service].uuid.u) == 0) {
                return ESP_ERR_INVALID_ARG;
            }
        }
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            const ble_characteristic_profile_t *characteristic =
                &service->characteristics[index];
            for (size_t compare_service = service_index;
                 compare_service < profile->service_count; ++compare_service) {
                const ble_service_profile_t *other = &profile->services[compare_service];
                size_t compare_index = compare_service == service_index ? index + 1 : 0;
                for (; compare_index < other->characteristic_count; ++compare_index) {
                    if (strcmp(characteristic->id,
                               other->characteristics[compare_index].id) == 0 ||
                        (compare_service == service_index &&
                         ble_uuid_cmp(&characteristic->uuid.u,
                                      &other->characteristics[compare_index].uuid.u) == 0)) {
                        return ESP_ERR_INVALID_ARG;
                    }
                }
            }
        }
    }
    return ESP_OK;
}

/**
 * @brief Parses and validates one profile JSON document into caller-provided storage.
 *
 * @param json JSON document bytes.
 * @param length JSON length in bytes.
 * @param bank Output bank receiving dynamically sized profiles.
 * @return ESP_OK when all profiles are parsed, otherwise an error code.
 */
static esp_err_t parse_profiles(const char *json, size_t length,
                                ble_profile_bank_t *bank)
{
    if (json == NULL || length == 0 || bank == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    free_profile_bank(bank);
    cJSON *document = cJSON_ParseWithLength(json, length);
    if (document == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const cJSON *schema_version = cJSON_GetObjectItem(document, "schema_version");
    const cJSON *devices = cJSON_GetObjectItem(document, "devices");
    int device_count = cJSON_GetArraySize(devices);
    if (!cJSON_IsNumber(schema_version) || schema_version->valuedouble != 3.0 ||
        !cJSON_IsArray(devices) || device_count <= 0 ||
        device_count > BLE_PROFILE_MAX_DEVICES) {
        cJSON_Delete(document);
        return ESP_ERR_INVALID_ARG;
    }

    bank->profiles = calloc((size_t)device_count, sizeof(*bank->profiles));
    if (bank->profiles == NULL) {
        cJSON_Delete(document);
        return ESP_ERR_NO_MEM;
    }
    bank->count = (size_t)device_count;

    size_t profile_index = 0;
    const cJSON *device = NULL;
    cJSON_ArrayForEach(device, devices) {
        ble_profile_t *profile = &bank->profiles[profile_index];
        if (copy_string(profile->device_id, sizeof(profile->device_id),
                        cJSON_GetObjectItem(device, "device_id")) != ESP_OK ||
            parse_uuid(&profile->advertised_service_uuid,
                       profile->advertised_service_uuid_text,
                       sizeof(profile->advertised_service_uuid_text),
                       cJSON_GetObjectItem(device, "advertised_service_uuid")) != ESP_OK ||
            copy_string(profile->driver_name, sizeof(profile->driver_name),
                        cJSON_GetObjectItem(device, "driver")) != ESP_OK ||
            parse_match(profile, cJSON_GetObjectItem(device, "match")) != ESP_OK) {
            cJSON_Delete(document);
            free_profile_bank(bank);
            return ESP_ERR_INVALID_ARG;
        }
        const cJSON *advertising_name = cJSON_GetObjectItem(device, "advertising_name");
        if (advertising_name != NULL &&
            copy_string(profile->advertising_name, sizeof(profile->advertising_name),
                        advertising_name) != ESP_OK) {
            cJSON_Delete(document);
            free_profile_bank(bank);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t index = 0; index < profile_index; ++index) {
            if (strcmp(profile->device_id, bank->profiles[index].device_id) == 0) {
                cJSON_Delete(document);
                free_profile_bank(bank);
                return ESP_ERR_INVALID_ARG;
            }
        }

        const cJSON *services = cJSON_GetObjectItem(device, "services");
        int service_count = cJSON_GetArraySize(services);
        if (!cJSON_IsArray(services) || service_count <= 0 ||
            service_count > BLE_PROFILE_MAX_SERVICES) {
            cJSON_Delete(document);
            free_profile_bank(bank);
            return ESP_ERR_INVALID_ARG;
        }
        profile->services = calloc((size_t)service_count, sizeof(*profile->services));
        if (profile->services == NULL) {
            cJSON_Delete(document);
            free_profile_bank(bank);
            return ESP_ERR_NO_MEM;
        }
        profile->service_count = (size_t)service_count;

        size_t service_index = 0;
        const cJSON *service = NULL;
        cJSON_ArrayForEach(service, services) {
            ble_service_profile_t *service_profile = &profile->services[service_index];
            if (parse_uuid(&service_profile->uuid, service_profile->uuid_text,
                           sizeof(service_profile->uuid_text),
                           cJSON_GetObjectItem(service, "uuid")) != ESP_OK) {
                cJSON_Delete(document);
                free_profile_bank(bank);
                return ESP_ERR_INVALID_ARG;
            }
            const cJSON *characteristics = cJSON_GetObjectItem(service, "characteristics");
            int characteristic_count = cJSON_GetArraySize(characteristics);
            if (!cJSON_IsArray(characteristics) || characteristic_count <= 0 ||
                characteristic_count > BLE_PROFILE_MAX_CHARACTERISTICS) {
                cJSON_Delete(document);
                free_profile_bank(bank);
                return ESP_ERR_INVALID_ARG;
            }
            service_profile->characteristics =
                calloc((size_t)characteristic_count,
                       sizeof(*service_profile->characteristics));
            if (service_profile->characteristics == NULL) {
                cJSON_Delete(document);
                free_profile_bank(bank);
                return ESP_ERR_NO_MEM;
            }
            service_profile->characteristic_count = (size_t)characteristic_count;

            size_t characteristic_index = 0;
            const cJSON *characteristic = NULL;
            cJSON_ArrayForEach(characteristic, characteristics) {
                ble_characteristic_profile_t *entry =
                    &service_profile->characteristics[characteristic_index];
                if (copy_string(entry->id, sizeof(entry->id),
                                cJSON_GetObjectItem(characteristic, "id")) != ESP_OK ||
                    parse_uuid(&entry->uuid, entry->uuid_text, sizeof(entry->uuid_text),
                               cJSON_GetObjectItem(characteristic, "uuid")) != ESP_OK ||
                    parse_properties(cJSON_GetObjectItem(characteristic, "properties"),
                                     &entry->properties) != ESP_OK ||
                    copy_string(entry->callback_set, sizeof(entry->callback_set),
                                cJSON_GetObjectItem(characteristic, "callback_set")) != ESP_OK ||
                    copy_string(entry->event_type, sizeof(entry->event_type),
                                cJSON_GetObjectItem(characteristic, "event_type")) != ESP_OK ||
                    parse_decoder(&entry->decoder,
                                  cJSON_GetObjectItem(characteristic, "decoder")) != ESP_OK) {
                    cJSON_Delete(document);
                    free_profile_bank(bank);
                    return ESP_ERR_INVALID_ARG;
                }
                const cJSON *topic = cJSON_GetObjectItem(characteristic, "topic");
                if (topic == NULL) {
                    strcpy(entry->topic, entry->event_type);
                } else if (copy_string(entry->topic, sizeof(entry->topic), topic) != ESP_OK) {
                    cJSON_Delete(document);
                    free_profile_bank(bank);
                    return ESP_ERR_INVALID_ARG;
                }
                if (!is_mqtt_topic_segment(entry->topic)) {
                    cJSON_Delete(document);
                    free_profile_bank(bank);
                    return ESP_ERR_INVALID_ARG;
                }
                entry->required_properties = entry->properties;
                ++characteristic_index;
            }
            ++service_index;
        }
        if (validate_profile_uniqueness(profile) != ESP_OK) {
            cJSON_Delete(document);
            free_profile_bank(bank);
            return ESP_ERR_INVALID_ARG;
        }
        ++profile_index;
    }
    cJSON_Delete(document);
    return ESP_OK;
}

/**
 * @brief Loads and validates profiles from the embedded fallback JSON document.
 *
 * @return ESP_OK when all profiles are loaded, otherwise an error code.
 */
esp_err_t ble_profile_load(void)
{
    const char *json = NULL;
    size_t length = 0;
    esp_err_t result = ble_profile_get_embedded_json(&json, &length);
    return result == ESP_OK ?
        parse_profiles(json, length, s_active_bank) : result;
}

/**
 * @brief Returns the linker-provided embedded fallback JSON document.
 *
 * @param json Output pointer receiving the embedded JSON bytes.
 * @param length Output embedded JSON length in bytes.
 * @return ESP_OK when the embedded document is available, otherwise an error code.
 */
esp_err_t ble_profile_get_embedded_json(const char **json, size_t *length)
{
    if (json == NULL || length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t embedded_length =
        (size_t)(_binary_devices_json_end - _binary_devices_json_start);
    if (embedded_length == 0) {
        return ESP_ERR_INVALID_SIZE;
    }
    *json = (const char *)_binary_devices_json_start;
    *length = embedded_length;
    return ESP_OK;
}

/**
 * @brief Parses candidate JSON into isolated staged profile storage.
 *
 * @param json Candidate devices.json content.
 * @param length Candidate length in bytes.
 * @return ESP_OK when syntax and schema validation succeeds, otherwise an error code.
 */
esp_err_t ble_profile_stage_json(const char *json, size_t length)
{
    s_rollback_available = false;
    return parse_profiles(json, length, s_staged_bank);
}

/**
 * @brief Returns the staged candidate profile count.
 *
 * @return Staged profile count.
 */
size_t ble_profile_staged_count(void)
{
    return s_staged_bank->count;
}

/**
 * @brief Returns one staged candidate profile.
 *
 * @param index Zero-based staged profile index.
 * @return Staged profile pointer, or NULL when out of range.
 */
const ble_profile_t *ble_profile_staged_at(size_t index)
{
    return index < s_staged_bank->count ? &s_staged_bank->profiles[index] : NULL;
}

/**
 * @brief Commits staged profiles while preserving the old bank for rollback.
 *
 * @return ESP_OK when committed, otherwise an error code.
 */
esp_err_t ble_profile_commit_staged(void)
{
    if (s_staged_bank->count == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    s_rollback_available = s_active_bank->count > 0;
    ble_profile_bank_t *previous_bank = s_active_bank;
    s_active_bank = s_staged_bank;
    s_staged_bank = previous_bank;
    return ESP_OK;
}

/**
 * @brief Restores the profile set preserved by the last candidate commit.
 *
 * @return ESP_OK when restored, otherwise an error code.
 */
esp_err_t ble_profile_rollback(void)
{
    if (!s_rollback_available) {
        return ESP_ERR_INVALID_STATE;
    }
    ble_profile_bank_t *candidate_bank = s_active_bank;
    s_active_bank = s_staged_bank;
    s_staged_bank = candidate_bank;
    s_rollback_available = false;
    ble_profile_discard_staged();
    return ESP_OK;
}

/**
 * @brief Releases the inactive profile bank and clears rollback availability.
 *
 * @return None.
 */
void ble_profile_discard_staged(void)
{
    free_profile_bank(s_staged_bank);
    s_rollback_available = false;
}

/**
 * @brief Finds one loaded profile by its device identifier.
 *
 * @param device_id Device identifier declared in devices.json.
 * @return Matching profile, or NULL when no profile exists.
 */
ble_profile_t *ble_profile_find(const char *device_id)
{
    if (device_id == NULL) {
        return NULL;
    }
    for (size_t index = 0; index < s_active_bank->count; ++index) {
        if (strcmp(device_id, s_active_bank->profiles[index].device_id) == 0) {
            return &s_active_bank->profiles[index];
        }
    }
    return NULL;
}

/**
 * @brief Returns the number of profiles loaded from devices.json.
 *
 * @return Loaded profile count.
 */
size_t ble_profile_count(void)
{
    return s_active_bank->count;
}

/**
 * @brief Returns one loaded profile by index.
 *
 * @param index Zero-based profile index.
 * @return Profile pointer, or NULL when the index is out of range.
 */
ble_profile_t *ble_profile_at(size_t index)
{
    return index < s_active_bank->count ? &s_active_bank->profiles[index] : NULL;
}

/**
 * @brief Finds one configured characteristic by its JSON identifier.
 *
 * @param profile Loaded device profile.
 * @param characteristic_id JSON characteristic identifier.
 * @return Matching characteristic, or NULL when it is not declared.
 */
const ble_characteristic_profile_t *ble_profile_find_characteristic(
    const ble_profile_t *profile, const char *characteristic_id)
{
    if (profile == NULL || characteristic_id == NULL) {
        return NULL;
    }
    for (size_t service_index = 0; service_index < profile->service_count; ++service_index) {
        const ble_service_profile_t *service = &profile->services[service_index];
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            if (strcmp(service->characteristics[index].id, characteristic_id) == 0) {
                return &service->characteristics[index];
            }
        }
    }
    return NULL;
}
