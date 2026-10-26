#include "ble_bus.h"

#include <string.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_store.h"
#include "nimble/hci_common.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "ble_provisioning_service.h"

#define BLE_BUS_MAX_NOTIFICATION BLE_ATT_ATTR_MAX_LEN
#define BLE_SCAN_NAME_WAIT_US 300000ULL

static const char *TAG = "ble_bus";
static ble_device_slot_t s_devices[BLE_MAX_DEVICES];
static size_t s_binding_count;
static ble_device_t *volatile s_connecting_device;
static volatile bool s_suspended;
static bool s_initialized;
static bool s_started;
static esp_timer_handle_t s_scan_candidate_timer;
static ble_device_t *s_scan_candidate;
static ble_addr_t s_scan_candidate_address;
static int8_t s_scan_candidate_rssi;
static char s_scan_candidate_name[BLE_PROFILE_MAX_NAME];

void ble_store_config_init(void);
static int gap_event(struct ble_gap_event *event, void *arg);
static esp_err_t scan_start(void);
static int mtu_exchange_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t mtu, void *arg);
static void start_service_discovery(ble_device_t *device, uint16_t conn_handle);
static bool advertisement_has_service(const struct ble_hs_adv_fields *fields,
                                     const ble_uuid128_t *uuid);
static bool profile_matches_advertisement(const ble_profile_t *profile,
                                          const struct ble_hs_adv_fields *fields,
                                          const ble_addr_t *address);
static void connect_scan_candidate(void);
static void scan_candidate_timeout(void *arg);
static int service_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *service, void *arg);
static int characteristic_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *characteristic, void *arg);
static int descriptor_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *descriptor,
                         void *arg);
static int subscription_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg);
static int read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg);
static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg);
static void on_reset(int reason);
static void on_sync(void);
static void host_task(void *arg);
static void clear_runtime_handles(ble_device_t *device);
static void set_descriptor_ranges(ble_device_t *device, ble_service_profile_t *service);
static ble_device_t *find_device_by_connection(uint16_t conn_handle);
static void finish_device_discovery(ble_device_t *device, uint16_t conn_handle);
static size_t connected_device_count(void);
static bool connection_capacity_available(void);
static bool bus_is_quiescent(void);
static void resume_discovery_if_needed(void);

/**
 * @brief Counts established remote BLE peripheral connections.
 *
 * @return Number of currently established device connections.
 */
static size_t connected_device_count(void)
{
    size_t count = 0;
    for (size_t index = 0; index < s_binding_count; ++index) {
        if (s_devices[index].connected) {
            ++count;
        }
    }
    return count;
}

/**
 * @brief Tests whether another remote peripheral can be connected.
 *
 * @return true when the configured central connection limit has free capacity.
 */
static bool connection_capacity_available(void)
{
    return connected_device_count() < BLE_BUS_MAX_CONNECTIONS;
}

/**
 * @brief Tests whether no connection attempt or established device remains.
 *
 * @return true when the BLE bus is safe to reconfigure.
 */
static bool bus_is_quiescent(void)
{
    return s_connecting_device == NULL && connected_device_count() == 0;
}

/**
 * @brief Restarts discovery when a free central connection slot is available.
 *
 * @return None.
 */
static void resume_discovery_if_needed(void)
{
    if (!s_suspended && s_connecting_device == NULL && connection_capacity_available()) {
        scan_start();
    }
}

/**
 * @brief Starts configured GATT service discovery after MTU negotiation completes.
 *
 * @param device Connected BLE device instance.
 * @param conn_handle NimBLE connection handle.
 * @return None.
 */
static void start_service_discovery(ble_device_t *device, uint16_t conn_handle)
{
    if (device == NULL || device->profile == NULL || device->profile->service_count == 0) {
        return;
    }
    if (ble_gattc_disc_svc_by_uuid(conn_handle,
                                   &device->profile->services[0].uuid.u,
                                   service_cb, device) != 0) {
        device->last_error = BLE_HS_EAPP;
        ESP_LOGE(TAG, "Cannot start GATT service discovery for %s",
                 device->profile->device_id);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

/**
 * @brief Receives completion of a central-initiated ATT MTU exchange.
 *
 * @param conn_handle NimBLE connection handle.
 * @param error MTU exchange status.
 * @param mtu Negotiated ATT MTU reported by NimBLE.
 * @param arg Connected BLE device instance.
 * @return 0 after starting GATT service discovery.
 */
static int mtu_exchange_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t mtu, void *arg)
{
    ble_device_t *device = arg;
    if (device == NULL || !device->connected || device->connection_handle != conn_handle) {
        return 0;
    }
    device->negotiated_mtu = ble_att_mtu(conn_handle);
    if (error == NULL || error->status != 0) {
        ESP_LOGW(TAG, "MTU exchange failed for %s, status=%d; using MTU=%u",
                 device->profile->device_id, error == NULL ? BLE_HS_EAPP : error->status,
                 (unsigned int)device->negotiated_mtu);
    } else {
        ESP_LOGI(TAG, "MTU negotiated for %s: %u (payload=%u)",
                 device->profile->device_id, (unsigned int)mtu,
                 (unsigned int)(mtu > 3 ? mtu - 3 : 0));
    }
    start_service_discovery(device, conn_handle);
    return 0;
}

/**
 * @brief Registers one JSON-backed profile and its protocol driver.
 *
 * @param profile Loaded profile storage.
 * @param driver Protocol driver selected by the profile.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t ble_bus_register_profile(ble_profile_t *profile, const ble_driver_t *driver)
{
    if (profile == NULL || driver == NULL || driver->ops == NULL ||
        s_binding_count >= BLE_MAX_DEVICES) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t service_index = 0; service_index < profile->service_count; ++service_index) {
        ble_service_profile_t *service = &profile->services[service_index];
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            if (ble_driver_find_callback(driver,
                                         service->characteristics[index].callback_set) == NULL) {
                return ESP_ERR_NOT_FOUND;
            }
        }
    }
    memset(&s_devices[s_binding_count], 0, sizeof(s_devices[s_binding_count]));
    s_devices[s_binding_count].profile = profile;
    s_devices[s_binding_count].driver = driver;
    s_devices[s_binding_count].registered = true;
    clear_runtime_handles(&s_devices[s_binding_count]);
    ++s_binding_count;
    return ESP_OK;
}

/**
 * @brief Initializes NimBLE before local GATT services are registered.
 *
 * @return ESP_OK when NimBLE is ready for GATT service registration, otherwise an error code.
 */
esp_err_t ble_bus_init(void)
{
    if (s_binding_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = nimble_port_init();
    if (result != ESP_OK) {
        return result;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_store_config_init();
    esp_err_t timer_result = esp_timer_create(&(esp_timer_create_args_t) {
        .callback = scan_candidate_timeout,
        .name = "ble_scan_candidate",
    }, &s_scan_candidate_timer);
    if (timer_result != ESP_OK) {
        return timer_result;
    }
    s_initialized = true;
    return ESP_OK;
}

/**
 * @brief Starts the NimBLE host task after all local GATT services are registered.
 *
 * @return ESP_OK when host startup was requested, otherwise an error code.
 */
esp_err_t ble_bus_start(void)
{
    if (!s_initialized || s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_started = true;
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

/**
 * @brief Quiesces BLE discovery and every active connection.
 *
 * @param timeout_ms Maximum wait time for connection shutdown.
 * @return ESP_OK when quiescent, otherwise an error code.
 */
esp_err_t ble_bus_suspend(uint32_t timeout_ms)
{
    if (!s_initialized || timeout_ms == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    s_suspended = true;
    if (s_scan_candidate_timer != NULL &&
        esp_timer_is_active(s_scan_candidate_timer)) {
        esp_timer_stop(s_scan_candidate_timer);
    }
    s_scan_candidate = NULL;
    if (ble_gap_disc_active()) {
        int result = ble_gap_disc_cancel();
        if (result != 0 && result != BLE_HS_EALREADY) {
            s_suspended = false;
            return ESP_FAIL;
        }
    }

    if (s_connecting_device != NULL) {
        int result = ble_gap_conn_cancel();
        if (result != 0 && result != BLE_HS_EALREADY) {
            s_suspended = false;
            return ESP_FAIL;
        }
    }
    for (size_t index = 0; index < s_binding_count; ++index) {
        if (!s_devices[index].connected) {
            continue;
        }
        int result = ble_gap_terminate(s_devices[index].connection_handle,
                                       BLE_ERR_REM_USER_CONN_TERM);
        if (result != 0 && result != BLE_HS_EALREADY) {
            s_suspended = false;
            return ESP_FAIL;
        }
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    while (!bus_is_quiescent()) {
        if (xTaskGetTickCount() - start >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

/**
 * @brief Clears all fixed profile slots while BLE is quiescent.
 *
 * @return ESP_OK when cleared, otherwise an error code.
 */
esp_err_t ble_bus_clear_profiles(void)
{
    if (!s_suspended || !bus_is_quiescent()) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(s_devices, 0, sizeof(s_devices));
    s_binding_count = 0;
    return ESP_OK;
}

/**
 * @brief Leaves reconfiguration mode and resumes discovery when capacity is available.
 *
 * @return ESP_OK when scanning is requested, otherwise an error code.
 */
esp_err_t ble_bus_resume(void)
{
    if (!s_initialized || !s_suspended || s_binding_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    s_suspended = false;
    esp_err_t result = scan_start();
    if (result != ESP_OK) {
        s_suspended = true;
    }
    return result;
}

/**
 * @brief Returns the ready device owned by a driver.
 *
 * @param driver Driver to match.
 * @return Ready device, or NULL when unavailable.
 */
ble_device_t *ble_bus_get_ready_device(const ble_driver_t *driver)
{
    if (driver == NULL) {
        return NULL;
    }
    for (size_t index = 0; index < s_binding_count; ++index) {
        if (s_devices[index].ready && s_devices[index].driver == driver) {
            return &s_devices[index];
        }
    }
    return NULL;
}

/**
 * @brief Finds a ready device by configured identifier.
 *
 * @param device_id Device identifier declared in devices.json.
 * @return Ready device, or NULL when unavailable.
 */
ble_device_t *ble_bus_get_ready_device_by_id(const char *device_id)
{
    if (device_id == NULL) {
        return NULL;
    }
    for (size_t index = 0; index < s_binding_count; ++index) {
        if (s_devices[index].ready && s_devices[index].profile != NULL &&
            strcmp(s_devices[index].profile->device_id, device_id) == 0) {
            return &s_devices[index];
        }
    }
    return NULL;
}

/**
 * @brief Finds a characteristic by JSON identifier.
 *
 * @param device Connected device.
 * @param id Characteristic identifier.
 * @param runtime Output pointer for the matching slot runtime state, or NULL.
 * @return Matching characteristic, or NULL when absent.
 */
static ble_characteristic_profile_t *find_characteristic(
    ble_device_t *device, const char *id, ble_characteristic_runtime_t **runtime)
{
    if (runtime != NULL) {
        *runtime = NULL;
    }
    if (device == NULL || device->profile == NULL || id == NULL) {
        return NULL;
    }
    for (size_t service_index = 0; service_index < device->profile->service_count; ++service_index) {
        ble_service_profile_t *service = &device->profile->services[service_index];
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            if (strcmp(id, service->characteristics[index].id) == 0) {
                if (runtime != NULL) {
                    *runtime = &device->services[service_index].characteristics[index];
                }
                return &service->characteristics[index];
            }
        }
    }
    return NULL;
}

/**
 * @brief Writes bytes to a configured characteristic.
 *
 * @param device Connected device.
 * @param characteristic_id JSON characteristic identifier.
 * @param mode GATT write mode.
 * @param data Bytes to send.
 * @param length Number of bytes to send.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_bus_write(ble_device_t *device, const char *characteristic_id,
                        ble_write_mode_t mode, const uint8_t *data, size_t length)
{
    ble_characteristic_runtime_t *runtime = NULL;
    ble_characteristic_profile_t *characteristic =
        find_characteristic(device, characteristic_id, &runtime);
    if (characteristic == NULL || data == NULL || length == 0 || length > UINT16_MAX ||
        runtime == NULL || !device->connected || runtime->value_handle == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int result;
    if (mode == BLE_WRITE_WITHOUT_RESPONSE) {
        if ((runtime->properties & BLE_GATT_CHR_PROP_WRITE_NO_RSP) == 0) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        result = ble_gattc_write_no_rsp_flat(device->connection_handle,
                                              runtime->value_handle, data, length);
        const ble_characteristic_callback_t *callback =
            ble_driver_find_callback(device->driver, characteristic->callback_set);
        if (callback != NULL && callback->on_write_complete != NULL) {
            callback->on_write_complete(device, characteristic->id, result);
        }
    } else {
        if ((runtime->properties & BLE_GATT_CHR_PROP_WRITE) == 0) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        result = ble_gattc_write_flat(device->connection_handle,
                                       runtime->value_handle, data, length,
                                       write_cb, runtime);
    }
    return result == 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Starts a GATT read from a configured characteristic.
 *
 * @param device Connected device.
 * @param characteristic_id JSON characteristic identifier.
 * @return ESP_OK when accepted by NimBLE, otherwise an error code.
 */
esp_err_t ble_bus_read(ble_device_t *device, const char *characteristic_id)
{
    ble_characteristic_runtime_t *runtime = NULL;
    ble_characteristic_profile_t *characteristic =
        find_characteristic(device, characteristic_id, &runtime);
    if (characteristic == NULL || runtime == NULL || !device->connected ||
        runtime->value_handle == 0 || (runtime->properties & BLE_GATT_CHR_PROP_READ) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return ble_gattc_read(device->connection_handle, runtime->value_handle,
                          read_cb, runtime) == 0 ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Checks whether all optional OTA operations are provided.
 *
 * @param driver Driver to inspect.
 * @return true when OTA is available, otherwise false.
 */
bool ble_bus_driver_supports_ota(const ble_driver_t *driver)
{
    return driver != NULL && driver->ota_begin != NULL && driver->ota_write != NULL &&
           driver->ota_finish != NULL;
}

/**
 * @brief Starts driver-specific OTA through the common device API.
 *
 * @param device Ready device.
 * @param image_size Total OTA image size in bytes.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_begin(ble_device_t *device, size_t image_size)
{
    return device != NULL && device->ready && ble_bus_driver_supports_ota(device->driver) ?
        device->driver->ota_begin(device, image_size) : ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Sends one OTA data block through the common device API.
 *
 * @param device Ready device.
 * @param data OTA data block.
 * @param length OTA block length in bytes.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_write(ble_device_t *device, const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return device != NULL && device->ready && ble_bus_driver_supports_ota(device->driver) ?
        device->driver->ota_write(device, data, length) : ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Finishes driver-specific OTA through the common device API.
 *
 * @param device Ready device.
 * @return Driver result, or ESP_ERR_NOT_SUPPORTED when OTA is unavailable.
 */
esp_err_t ble_bus_ota_finish(ble_device_t *device)
{
    return device != NULL && device->ready && ble_bus_driver_supports_ota(device->driver) ?
        device->driver->ota_finish(device) : ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Clears one slot's connection-specific GATT discovery state.
 *
 * @param device Device slot whose runtime state is reset.
 * @return None.
 */
static void clear_runtime_handles(ble_device_t *device)
{
    if (device == NULL || device->profile == NULL) {
        return;
    }
    memset(device->services, 0, sizeof(device->services));
    for (size_t service_index = 0; service_index < device->profile->service_count; ++service_index) {
        ble_service_profile_t *service = &device->profile->services[service_index];
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            ble_characteristic_runtime_t *runtime =
                &device->services[service_index].characteristics[index];
            runtime->device = device;
            runtime->profile = &service->characteristics[index];
        }
    }
}

/**
 * @brief Sets descriptor search boundaries for one slot service.
 *
 * @param device Device slot that owns the service runtime state.
 * @param service Service whose characteristics were discovered.
 * @return None.
 */
static void set_descriptor_ranges(ble_device_t *device, ble_service_profile_t *service)
{
    if (device == NULL || service == NULL ||
        device->discovery_service_index >= device->profile->service_count) {
        return;
    }
    ble_service_runtime_t *runtime_service =
        &device->services[device->discovery_service_index];
    for (size_t index = 0; index < service->characteristic_count; ++index) {
        ble_characteristic_runtime_t *current = &runtime_service->characteristics[index];
        current->descriptor_end_handle = runtime_service->end_handle;
        for (size_t next_index = 0; next_index < service->characteristic_count; ++next_index) {
            uint16_t next_handle = runtime_service->characteristics[next_index].definition_handle;
            if (next_handle > current->definition_handle &&
                next_handle - 1 < current->descriptor_end_handle) {
                current->descriptor_end_handle = next_handle - 1;
            }
        }
    }
}

/**
 * @brief Starts active BLE scanning.
 *
 * @return ESP_OK when scanning starts, otherwise an error code.
 */
static esp_err_t scan_start(void)
{
    if (s_suspended || s_connecting_device != NULL || !connection_capacity_available()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ble_gap_disc_active()) {
        return ESP_OK;
    }
    uint8_t own_addr_type;
    struct ble_gap_disc_params params = {0};
    int result = ble_hs_id_infer_auto(0, &own_addr_type);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot infer local address type: %d", result);
        return ESP_FAIL;
    }
    params.filter_duplicates = 0;
    params.passive = 0;
    result = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot start BLE scan: %d", result);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BLE scan started");
    return ESP_OK;
}

/**
 * @brief Checks whether one parsed advertisement contains a configured 128-bit service UUID.
 *
 * @param fields Parsed BLE advertisement fields.
 * @param uuid Configured service UUID.
 * @return true when the UUID is present, otherwise false.
 */
static bool advertisement_has_service(const struct ble_hs_adv_fields *fields,
                                     const ble_uuid128_t *uuid)
{
    if (fields == NULL || uuid == NULL) {
        return false;
    }
    for (size_t index = 0; index < fields->num_uuids128; ++index) {
        if (ble_uuid_cmp(&fields->uuids128[index].u, &uuid->u) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Applies the optional address and manufacturer-data constraints to one advertisement.
 *
 * @param profile Profile being tested.
 * @param fields Parsed BLE advertisement fields.
 * @param address Advertiser address from the scan report.
 * @return true when every configured constraint matches, otherwise false.
 */
static bool profile_matches_advertisement(const ble_profile_t *profile,
                                          const struct ble_hs_adv_fields *fields,
                                          const ble_addr_t *address)
{
    if (profile == NULL || fields == NULL || address == NULL) {
        return false;
    }
    if (profile->has_match_address &&
        memcmp(profile->match_address, address->val, sizeof(profile->match_address)) != 0) {
        return false;
    }
    if (profile->manufacturer_data_prefix_length > 0) {
        if (fields->mfg_data == NULL ||
            fields->mfg_data_len < profile->manufacturer_data_prefix_length ||
            memcmp(fields->mfg_data, profile->manufacturer_data_prefix,
                   profile->manufacturer_data_prefix_length) != 0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Stops scanning and initiates a connection to the pending UUID-matched device.
 *
 * @return None.
 */
static void connect_scan_candidate(void)
{
    ble_device_t *device = s_scan_candidate;
    if (device == NULL || s_suspended || s_connecting_device != NULL ||
        !connection_capacity_available() || device->connected || device->connecting) {
        s_scan_candidate = NULL;
        return;
    }
    if (s_scan_candidate_timer != NULL &&
        esp_timer_is_active(s_scan_candidate_timer)) {
        esp_timer_stop(s_scan_candidate_timer);
    }
    ble_addr_t peer_address = s_scan_candidate_address;
    int8_t rssi = s_scan_candidate_rssi;
    char name[sizeof(s_scan_candidate_name)];
    memcpy(name, s_scan_candidate_name, sizeof(name));
    s_scan_candidate = NULL;

    if (ble_gap_disc_active()) {
        int cancel_result = ble_gap_disc_cancel();
        if (cancel_result != 0 && cancel_result != BLE_HS_EALREADY) {
            ESP_LOGW(TAG, "Cannot stop BLE scan before connection: %d", cancel_result);
            scan_start();
            return;
        }
    }
    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        scan_start();
        return;
    }

    clear_runtime_handles(device);
    device->connection_handle = 0;
    device->discovery_service_index = 0;
    device->discovery_characteristic_index = 0;
    device->ready = false;
    device->connected = false;
    device->last_error = 0;
    device->address_type = peer_address.type;
    memcpy(device->address, peer_address.val, sizeof(device->address));
    device->rssi = rssi;
    memcpy(device->observed_advertising_name, name,
           sizeof(device->observed_advertising_name));
    device->connecting = true;
    s_connecting_device = device;
    if (name[0] != '\0') {
        ESP_LOGI(TAG, "Target %s matched UUID, name=%s, address=%02X:%02X:%02X:%02X:%02X:%02X, RSSI=%d dBm; connecting",
                 device->profile->device_id, name,
                 peer_address.val[5], peer_address.val[4], peer_address.val[3],
                 peer_address.val[2], peer_address.val[1], peer_address.val[0], rssi);
    } else {
        ESP_LOGI(TAG, "Target %s matched UUID, name=<not provided>, address=%02X:%02X:%02X:%02X:%02X:%02X, RSSI=%d dBm; connecting",
                 device->profile->device_id,
                 peer_address.val[5], peer_address.val[4], peer_address.val[3],
                 peer_address.val[2], peer_address.val[1], peer_address.val[0], rssi);
    }
    int result = ble_gap_connect(own_addr_type, &peer_address, 30000, NULL, gap_event, device);
    if (result != 0) {
        device->last_error = result;
        ESP_LOGE(TAG, "Cannot initiate connection to %s: %d",
                 device->profile->device_id, result);
        device->connecting = false;
        s_connecting_device = NULL;
        resume_discovery_if_needed();
    }
}

/**
 * @brief Connects after the short wait for a matching advertiser's Scan Response expires.
 *
 * @param arg Unused timer argument.
 * @return None.
 */
static void scan_candidate_timeout(void *arg)
{
    (void)arg;
    connect_scan_candidate();
}

/**
 * @brief Handles GATT service discovery and starts characteristic discovery.
 *
 * @param conn_handle Connection handle.
 * @param error Discovery status.
 * @param service Discovered service.
 * @param arg Device instance.
 * @return 0 to continue.
 */
static int service_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *service, void *arg)
{
    ble_device_t *device = arg;
    ble_service_profile_t *profile = &device->profile->services[device->discovery_service_index];
    ble_service_runtime_t *runtime = &device->services[device->discovery_service_index];
    if (error->status == 0 && service != NULL) {
        runtime->start_handle = service->start_handle;
        runtime->end_handle = service->end_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        device->last_error = error->status;
        ESP_LOGE(TAG, "Service discovery failed: %d", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (runtime->start_handle == 0 || runtime->end_handle == 0) {
        device->last_error = BLE_HS_ENOENT;
        ESP_LOGW(TAG, "Configured service not found: %s", profile->uuid_text);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (ble_gattc_disc_all_chrs(conn_handle, runtime->start_handle, runtime->end_handle,
                                characteristic_cb, device) != 0) {
        device->last_error = BLE_HS_EAPP;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/**
 * @brief Binds discovered characteristics and begins descriptor discovery.
 *
 * @param conn_handle Connection handle.
 * @param error Discovery status.
 * @param characteristic Discovered characteristic.
 * @param arg Device instance.
 * @return 0 to continue.
 */
static int characteristic_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *characteristic, void *arg)
{
    ble_device_t *device = arg;
    ble_service_profile_t *service = &device->profile->services[device->discovery_service_index];
    ble_service_runtime_t *runtime_service = &device->services[device->discovery_service_index];
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        device->last_error = error->status;
        ESP_LOGE(TAG, "Characteristic discovery failed: %d", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (characteristic != NULL) {
        for (size_t index = 0; index < service->characteristic_count; ++index) {
            ble_characteristic_profile_t *entry = &service->characteristics[index];
            ble_characteristic_runtime_t *runtime = &runtime_service->characteristics[index];
            if (ble_uuid_cmp(&characteristic->uuid.u, &entry->uuid.u) == 0) {
                if ((characteristic->properties & entry->required_properties) !=
                    entry->required_properties) {
                    device->last_error = BLE_HS_EINVAL;
                    ESP_LOGW(TAG, "Characteristic properties do not match: %s", entry->id);
                    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                    return 0;
                }
                runtime->value_handle = characteristic->val_handle;
                runtime->definition_handle = characteristic->def_handle;
                runtime->properties = characteristic->properties;
            }
        }
    }
    if (error->status != BLE_HS_EDONE) {
        return 0;
    }
    for (size_t index = 0; index < service->characteristic_count; ++index) {
        if (runtime_service->characteristics[index].value_handle == 0) {
            device->last_error = BLE_HS_ENOENT;
            ESP_LOGW(TAG, "Configured characteristic not found: %s", service->characteristics[index].id);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
    }
    set_descriptor_ranges(device, service);
    device->discovery_characteristic_index = 0;
    while (device->discovery_characteristic_index < service->characteristic_count &&
           (runtime_service->characteristics[device->discovery_characteristic_index].properties &
            BLE_GATT_CHR_PROP_NOTIFY) == 0) {
        ++device->discovery_characteristic_index;
    }
    if (device->discovery_characteristic_index == service->characteristic_count) {
        ++device->discovery_service_index;
        if (device->discovery_service_index == device->profile->service_count) {
            finish_device_discovery(device, conn_handle);
            return 0;
        }
        ble_service_profile_t *next = &device->profile->services[device->discovery_service_index];
        if (ble_gattc_disc_svc_by_uuid(conn_handle, &next->uuid.u, service_cb, device) != 0) {
            device->last_error = BLE_HS_EAPP;
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    ble_characteristic_runtime_t *notify = &runtime_service->characteristics[device->discovery_characteristic_index];
    if (ble_gattc_disc_all_dscs(conn_handle, notify->value_handle, notify->descriptor_end_handle,
                                descriptor_cb, device) != 0) {
        device->last_error = BLE_HS_EAPP;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/**
 * @brief Finds the CCCD for one notification characteristic.
 *
 * @param conn_handle Connection handle.
 * @param error Discovery status.
 * @param chr_val_handle Parent value handle.
 * @param descriptor Discovered descriptor.
 * @param arg Device instance.
 * @return 0 to continue.
 */
static int descriptor_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *descriptor,
                         void *arg)
{
    ble_device_t *device = arg;
    ble_characteristic_runtime_t *notify =
        &device->services[device->discovery_service_index].characteristics[
            device->discovery_characteristic_index];
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
        device->last_error = error->status;
        ESP_LOGE(TAG, "Descriptor discovery failed: %d", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    if (descriptor != NULL && chr_val_handle == notify->value_handle &&
        ble_uuid_cmp(&descriptor->uuid.u, BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) == 0) {
        notify->cccd_handle = descriptor->handle;
    }
    if (error->status != BLE_HS_EDONE) {
        return 0;
    }
    if (notify->cccd_handle == 0) {
        device->last_error = BLE_HS_ENOENT;
        ESP_LOGW(TAG, "Notify characteristic has no CCCD: %s", notify->profile->id);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    const uint8_t enable[2] = {1, 0};
    if (ble_gattc_write_flat(conn_handle, notify->cccd_handle, enable, sizeof(enable),
                             subscription_cb, device) != 0) {
        device->last_error = BLE_HS_EAPP;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/**
 * @brief Continues notification subscription or starts the next service.
 *
 * @param conn_handle Connection handle.
 * @param error Subscription status.
 * @param attr Written descriptor.
 * @param arg Device instance.
 * @return 0 after handling.
 */
static int subscription_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    ble_device_t *device = arg;
    ble_service_profile_t *service = &device->profile->services[device->discovery_service_index];
    ble_characteristic_runtime_t *notify =
        &device->services[device->discovery_service_index].characteristics[
            device->discovery_characteristic_index];
    if (error->status != 0) {
        device->last_error = error->status;
        ESP_LOGE(TAG, "Notification subscription failed: %d", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    notify->notification_enabled = true;
    ++device->discovery_characteristic_index;
    while (device->discovery_characteristic_index < service->characteristic_count &&
           (device->services[device->discovery_service_index].characteristics[
                device->discovery_characteristic_index].properties &
            BLE_GATT_CHR_PROP_NOTIFY) == 0) {
        ++device->discovery_characteristic_index;
    }
    if (device->discovery_characteristic_index < service->characteristic_count) {
        ble_characteristic_runtime_t *next =
            &device->services[device->discovery_service_index].characteristics[
                device->discovery_characteristic_index];
        if (ble_gattc_disc_all_dscs(conn_handle, next->value_handle, next->descriptor_end_handle,
                                    descriptor_cb, device) != 0) {
            device->last_error = BLE_HS_EAPP;
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    ++device->discovery_service_index;
    if (device->discovery_service_index < device->profile->service_count) {
        ble_service_profile_t *next = &device->profile->services[device->discovery_service_index];
        if (ble_gattc_disc_svc_by_uuid(conn_handle, &next->uuid.u, service_cb, device) != 0) {
            device->last_error = BLE_HS_EAPP;
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    finish_device_discovery(device, conn_handle);
    return 0;
}

/**
 * @brief Marks one slot ready and notifies its matched driver.
 *
 * @param device Device slot whose configured GATT discovery completed.
 * @param conn_handle NimBLE connection handle.
 * @return None.
 */
static void finish_device_discovery(ble_device_t *device, uint16_t conn_handle)
{
    if (device == NULL || device->driver == NULL || device->driver->ops == NULL) {
        if (device != NULL) {
            device->last_error = BLE_HS_EINVAL;
        }
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    device->ready = true;
    esp_err_t result = device->driver->ops->on_connected == NULL ? ESP_OK :
        device->driver->ops->on_connected(device);
    if (result != ESP_OK) {
        device->last_error = result;
        device->ready = false;
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    ESP_LOGI(TAG, "BLE device ready: %s, conn=%u",
             device->profile->device_id, conn_handle);
}

/**
 * @brief Logs bytes returned from a generic read operation.
 *
 * @param conn_handle Connection handle.
 * @param error Read status.
 * @param attr Read attribute.
 * @param arg Characteristic runtime context.
 * @return 0 after handling.
 */
static int read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    ble_characteristic_runtime_t *runtime = arg;
    ble_device_t *device = runtime == NULL ? NULL : runtime->device;
    ble_characteristic_profile_t *characteristic = runtime == NULL ? NULL : runtime->profile;
    if (device == NULL || characteristic == NULL) {
        return 0;
    }
    if (error->status != 0) {
        device->last_error = error->status;
        ESP_LOGE(TAG, "Characteristic read failed: %d", error->status);
        return 0;
    }
    if (attr != NULL && attr->om != NULL) {
        uint16_t length = OS_MBUF_PKTLEN(attr->om);
        uint8_t buffer[BLE_BUS_MAX_NOTIFICATION];
        if (length <= sizeof(buffer)) {
            os_mbuf_copydata(attr->om, 0, length, buffer);
            const ble_characteristic_callback_t *callback =
                device == NULL ? NULL :
                ble_driver_find_callback(device->driver, characteristic->callback_set);
            if (callback != NULL && callback->on_read != NULL) {
                callback->on_read(device, characteristic->id, buffer, length);
            }
        }
    }
    return 0;
}

/**
 * @brief Routes acknowledged write completion to a characteristic callback.
 *
 * @param conn_handle Connection handle.
 * @param error Write status.
 * @param attr Written attribute.
 * @param arg Characteristic runtime context.
 * @return 0 after handling.
 */
static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    ble_characteristic_runtime_t *runtime = arg;
    ble_device_t *device = runtime == NULL ? NULL : runtime->device;
    ble_characteristic_profile_t *characteristic = runtime == NULL ? NULL : runtime->profile;
    if (device != NULL && error->status != 0) {
        device->last_error = error->status;
    }
    const ble_characteristic_callback_t *callback =
        characteristic == NULL || device == NULL ? NULL :
        ble_driver_find_callback(device->driver, characteristic->callback_set);
    if (callback != NULL && callback->on_write_complete != NULL) {
        callback->on_write_complete(device, characteristic->id, error->status);
    }
    return 0;
}

/**
 * @brief Handles scan, connection, notification, and disconnect events.
 *
 * @param event NimBLE GAP event.
 * @param arg Device slot for connection-scoped events, or NULL during scanning.
 * @return 0 after handling.
 */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        if (s_suspended || s_connecting_device != NULL || !connection_capacity_available()) {
            return 0;
        }
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        if (s_scan_candidate != NULL &&
            event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP &&
            event->disc.addr.type == s_scan_candidate_address.type &&
            memcmp(event->disc.addr.val, s_scan_candidate_address.val,
                   sizeof(event->disc.addr.val)) == 0) {
            if (fields.name != NULL && fields.name_len > 0 &&
                fields.name_len < sizeof(s_scan_candidate_name)) {
                memcpy(s_scan_candidate_name, fields.name, fields.name_len);
                s_scan_candidate_name[fields.name_len] = '\0';
            }
            connect_scan_candidate();
            return 0;
        }
        if (s_scan_candidate != NULL ||
            event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP) {
            return 0;
        }
        ble_device_t *device = NULL;
        bool ambiguous = false;
        for (size_t index = 0; index < s_binding_count; ++index) {
            if (s_devices[index].registered &&
                !s_devices[index].connected && !s_devices[index].connecting &&
                advertisement_has_service(&fields,
                                          &s_devices[index].profile->advertised_service_uuid) &&
                profile_matches_advertisement(s_devices[index].profile, &fields,
                                              &event->disc.addr)) {
                if (device != NULL) {
                    ambiguous = true;
                    break;
                }
                device = &s_devices[index];
            }
        }
        if (device == NULL || ambiguous) {
            if (ambiguous) {
                ESP_LOGW(TAG, "Advertisement matches multiple device profiles; ignoring it");
            }
            return 0;
        }
        s_scan_candidate = device;
        s_scan_candidate_address = event->disc.addr;
        s_scan_candidate_rssi = event->disc.rssi;
        memset(s_scan_candidate_name, 0, sizeof(s_scan_candidate_name));
        if (fields.name != NULL && fields.name_len > 0 &&
            fields.name_len < sizeof(s_scan_candidate_name)) {
            memcpy(s_scan_candidate_name, fields.name, fields.name_len);
            s_scan_candidate_name[fields.name_len] = '\0';
        }
        if (s_scan_candidate_timer == NULL ||
            esp_timer_start_once(s_scan_candidate_timer, BLE_SCAN_NAME_WAIT_US) != ESP_OK) {
            connect_scan_candidate();
        }
        return 0;
    }
    case BLE_GAP_EVENT_CONNECT: {
        ble_device_t *device = arg == NULL ? s_connecting_device : arg;
        if (device == NULL) {
            ESP_LOGE(TAG, "Connection event has no slot context");
            resume_discovery_if_needed();
            return 0;
        }
        if (event->connect.status != 0) {
            device->last_error = event->connect.status;
            ESP_LOGW(TAG, "BLE connection failed: %d", event->connect.status);
            device->connecting = false;
            if (s_connecting_device == device) {
                s_connecting_device = NULL;
            }
            resume_discovery_if_needed();
            return 0;
        }
        if (s_connecting_device == device) {
            s_connecting_device = NULL;
        }
        device->connecting = false;
        device->connection_handle = event->connect.conn_handle;
        device->connected = true;
        device->discovery_service_index = 0;
        if (device->observed_advertising_name[0] != '\0') {
            ESP_LOGI(TAG, "Connected device_id=%s name=%s address=%02X:%02X:%02X:%02X:%02X:%02X RSSI=%d dBm",
                     device->profile->device_id, device->observed_advertising_name,
                     device->address[5], device->address[4], device->address[3],
                     device->address[2], device->address[1], device->address[0],
                     device->rssi);
        } else {
            ESP_LOGI(TAG, "Connected device_id=%s name=<not provided> address=%02X:%02X:%02X:%02X:%02X:%02X RSSI=%d dBm",
                     device->profile->device_id,
                     device->address[5], device->address[4], device->address[3],
                     device->address[2], device->address[1], device->address[0],
                     device->rssi);
        }
        device->negotiated_mtu = ble_att_mtu(event->connect.conn_handle);
        int result = ble_gattc_exchange_mtu(event->connect.conn_handle,
                                            mtu_exchange_cb, device);
        if (result != 0) {
            ESP_LOGW(TAG, "Cannot start MTU exchange for %s: %d; using MTU=%u",
                     device->profile->device_id, result,
                     (unsigned int)device->negotiated_mtu);
            start_service_discovery(device, event->connect.conn_handle);
        }
        resume_discovery_if_needed();
        return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        ble_device_t *device = arg == NULL ?
            find_device_by_connection(event->notify_rx.conn_handle) : arg;
        ble_characteristic_runtime_t *runtime = NULL;
        if (device == NULL) {
            return 0;
        }
        for (size_t service_index = 0; service_index < device->profile->service_count; ++service_index) {
            ble_service_profile_t *service = &device->profile->services[service_index];
            for (size_t index = 0; index < service->characteristic_count; ++index) {
                if (device->services[service_index].characteristics[index].value_handle ==
                    event->notify_rx.attr_handle) {
                    runtime = &device->services[service_index].characteristics[index];
                    break;
                }
            }
        }
        if (runtime != NULL && runtime->profile != NULL && device->ready) {
            uint16_t length = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint8_t buffer[BLE_BUS_MAX_NOTIFICATION];
            if (length <= sizeof(buffer)) {
                os_mbuf_copydata(event->notify_rx.om, 0, length, buffer);
                const ble_characteristic_callback_t *callback =
                    ble_driver_find_callback(device->driver, runtime->profile->callback_set);
                if (callback != NULL && callback->on_notification != NULL) {
                    callback->on_notification(device, runtime->profile->id, buffer, length);
                }
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT: {
        ble_device_t *device = arg == NULL ?
            find_device_by_connection(event->disconnect.conn.conn_handle) : arg;
        ESP_LOGW(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        if (device != NULL) {
            device->last_error = event->disconnect.reason;
            if (device->driver != NULL && device->driver->ops != NULL &&
                device->driver->ops->on_disconnected != NULL) {
                device->driver->ops->on_disconnected(device, event->disconnect.reason);
            }
            device->ready = false;
            device->connected = false;
            device->connecting = false;
            clear_runtime_handles(device);
        }
        if (s_connecting_device == device) {
            s_connecting_device = NULL;
        }
        resume_discovery_if_needed();
        return 0;
    }
    default:
        return 0;
    }
}

/**
 * @brief Records a NimBLE host reset.
 *
 * @param reason Reset reason.
 * @return None.
 */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset: %d", reason);
    for (size_t index = 0; index < s_binding_count; ++index) {
        s_devices[index].ready = false;
        s_devices[index].connected = false;
        s_devices[index].connecting = false;
        s_devices[index].last_error = reason;
        clear_runtime_handles(&s_devices[index]);
    }
    s_connecting_device = NULL;
    s_scan_candidate = NULL;
}

/**
 * @brief Finds the device slot that owns a BLE connection handle.
 *
 * @param conn_handle NimBLE connection handle.
 * @return Matching device slot, or NULL when no slot owns the connection.
 */
static ble_device_t *find_device_by_connection(uint16_t conn_handle)
{
    for (size_t index = 0; index < s_binding_count; ++index) {
        if (s_devices[index].connected &&
            s_devices[index].connection_handle == conn_handle) {
            return &s_devices[index];
        }
    }
    return NULL;
}

/**
 * @brief Starts scanning after NimBLE synchronization.
 *
 * @return None.
 */
static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0) {
        ESP_LOGE(TAG, "Cannot ensure BLE identity address");
        return;
    }
    scan_start();
    esp_err_t result = ble_provisioning_service_start();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Cannot start provisioning advertisement: %s",
                 esp_err_to_name(result));
    }
}

/**
 * @brief Runs the NimBLE host event loop.
 *
 * @param arg Unused task argument.
 * @return None.
 */
static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
    ESP_LOGI(TAG,"task kill\n");
}

/*text*/
