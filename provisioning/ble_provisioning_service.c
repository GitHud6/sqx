#include "ble_provisioning_service.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "wifi_manager.h"

#define PROVISIONING_SSID_MAX_LENGTH 32U
#define PROVISIONING_PASSWORD_MAX_LENGTH 64U
#define PROVISIONING_STATUS_MAX_LENGTH 96U
#define BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16 0x2901U

static const char *TAG = "ble_provisioning";
static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x7c, 0x64, 0x13, 0x97, 0x68, 0x3a, 0x49, 0x6a,
                     0x8e, 0xe3, 0x40, 0x84, 0x2f, 0x16, 0xd9, 0xa0);
static const ble_uuid128_t s_ssid_uuid =
    BLE_UUID128_INIT(0x7d, 0x64, 0x13, 0x97, 0x68, 0x3a, 0x49, 0x6a,
                     0x8e, 0xe3, 0x40, 0x84, 0x2f, 0x16, 0xd9, 0xa0);
static const ble_uuid128_t s_password_uuid =
    BLE_UUID128_INIT(0x7e, 0x64, 0x13, 0x97, 0x68, 0x3a, 0x49, 0x6a,
                     0x8e, 0xe3, 0x40, 0x84, 0x2f, 0x16, 0xd9, 0xa0);
static const ble_uuid128_t s_control_uuid =
    BLE_UUID128_INIT(0x7f, 0x64, 0x13, 0x97, 0x68, 0x3a, 0x49, 0x6a,
                     0x8e, 0xe3, 0x40, 0x84, 0x2f, 0x16, 0xd9, 0xa0);
static const ble_uuid128_t s_status_uuid =
    BLE_UUID128_INIT(0x80, 0x64, 0x13, 0x97, 0x68, 0x3a, 0x49, 0x6a,
                     0x8e, 0xe3, 0x40, 0x84, 0x2f, 0x16, 0xd9, 0xa0);

static char s_ssid[PROVISIONING_SSID_MAX_LENGTH + 1U];
static char s_password[PROVISIONING_PASSWORD_MAX_LENGTH + 1U];
static char s_status[PROVISIONING_STATUS_MAX_LENGTH];
static uint16_t s_status_handle;
static uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_initialized;
static const char s_ssid_description[] = "Wi-Fi SSID";
static const char s_password_description[] = "Wi-Fi Password";
static const char s_control_description[] = "Provisioning Control";
static const char s_status_description[] = "Provisioning Status";

static int provisioning_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static int provisioning_gap_event(struct ble_gap_event *event, void *arg);
static void set_status(const char *state, const char *detail);
static int write_text(struct os_mbuf *om, char *target, size_t target_size);
static void on_wifi_state(bool connected, void *context);
static int description_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_ssid_uuid.u,
                .access_cb = provisioning_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_ssid_description,
                    },
                    { 0 },
                },
            },
            {
                .uuid = &s_password_uuid.u,
                .access_cb = provisioning_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_password_description,
                    },
                    { 0 },
                },
            },
            {
                .uuid = &s_control_uuid.u,
                .access_cb = provisioning_access,
                .flags = BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_control_description,
                    },
                    { 0 },
                },
            },
            {
                .uuid = &s_status_uuid.u,
                .access_cb = provisioning_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_status_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_status_description,
                    },
                    { 0 },
                },
            },
            { 0 },
        },
    },
    { 0 },
};

/**
 * @brief Registers the BLE GATT service used to provision Wi-Fi credentials.
 *
 * @return ESP_OK when the service is registered, otherwise an error code.
 */
esp_err_t ble_provisioning_service_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int result = ble_gatts_count_cfg(s_services);
    if (result == 0) {
        result = ble_gatts_add_svcs(s_services);
    }
    if (result != 0) {
        return ESP_FAIL;
    }
    set_status("idle", "write SSID, password, then START");
    wifi_manager_set_provisioning_handler(on_wifi_state, NULL);
    s_initialized = true;
    return ESP_OK;
}

/**
 * @brief Starts connectable provisioning advertisements.
 *
 * @return ESP_OK when advertising starts, otherwise an error code.
 */
esp_err_t ble_provisioning_service_start(void)
{
    if (!s_initialized || ble_gap_adv_active()) {
        return s_initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    uint8_t address_type;
    int result = ble_hs_id_infer_auto(0, &address_type);
    if (result != 0) {
        return ESP_FAIL;
    }
    uint8_t address[6];
    if (ble_hs_id_copy_addr(address_type, address, NULL) != 0) {
        return ESP_FAIL;
    }
    char name[22];
    snprintf(name, sizeof(name), "NETGATE-%02X%02X%02X", address[2], address[1], address[0]);
    if (ble_svc_gap_device_name_set(name) != 0) {
        return ESP_FAIL;
    }
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    if (ble_gap_adv_set_fields(&fields) != 0) {
        return ESP_FAIL;
    }
    struct ble_hs_adv_fields response = {0};
    response.name = (uint8_t *)name;
    response.name_len = strlen(name);
    response.name_is_complete = 1;
    if (ble_gap_adv_rsp_set_fields(&response) != 0) {
        return ESP_FAIL;
    }
    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    result = ble_gap_adv_start(address_type, NULL, BLE_HS_FOREVER, &params,
                               provisioning_gap_event, NULL);
    if (result != 0) {
        ESP_LOGW(TAG, "Cannot start provisioning advertising: %d", result);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Provisioning advertisement started: %s", name);
    return ESP_OK;
}

/**
 * @brief Publishes the current provisioning state through the Status characteristic.
 *
 * @param connected True when Wi-Fi owns an IPv4 address.
 * @return None.
 */
void ble_provisioning_service_set_wifi_state(bool connected)
{
    set_status(connected ? "connected" : "failed",
               connected ? "credentials saved" : "Wi-Fi disconnected");
}

/**
 * @brief Adapts Wi-Fi manager state changes to the provisioning Status characteristic.
 *
 * @param connected True when Wi-Fi owns an IPv4 address.
 * @param context Unused callback context.
 * @return None.
 */
static void on_wifi_state(bool connected, void *context)
{
    (void)context;
    ble_provisioning_service_set_wifi_state(connected);
}

/**
 * @brief Writes one textual status value and notifies the connected central.
 *
 * @param state Short provisioning state text.
 * @param detail Human-readable status detail.
 * @return None.
 */
static void set_status(const char *state, const char *detail)
{
    snprintf(s_status, sizeof(s_status), "{\"state\":\"%s\",\"detail\":\"%s\"}",
             state, detail);
    if (s_connection_handle != BLE_HS_CONN_HANDLE_NONE && s_status_handle != 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_status, strlen(s_status));
        if (om != NULL) {
            ble_gatts_notify_custom(s_connection_handle, s_status_handle, om);
        }
    }
}

/**
 * @brief Copies a bounded GATT write into a null-terminated text buffer.
 *
 * @param om NimBLE write payload.
 * @param target Output text buffer.
 * @param target_size Output buffer capacity.
 * @return 0 when copied, otherwise an ATT error code.
 */
static int write_text(struct os_mbuf *om, char *target, size_t target_size)
{
    uint16_t length = OS_MBUF_PKTLEN(om);
    if (length == 0 || length >= target_size ||
        ble_hs_mbuf_to_flat(om, target, length, NULL) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    target[length] = '\0';
    return 0;
}

/**
 * @brief Returns a standard Characteristic User Description descriptor value.
 *
 * @param conn_handle Client connection handle.
 * @param attr_handle Unused descriptor attribute handle.
 * @param ctxt NimBLE descriptor access context.
 * @param arg Null-terminated description text supplied by the descriptor definition.
 * @return 0 on success, otherwise an ATT error code.
 */
static int description_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    const char *description = arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC || description == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(ctxt->om, description, strlen(description)) == 0 ?
        0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/**
 * @brief Handles GATT writes and status reads for the provisioning service.
 *
 * @param conn_handle Client connection handle.
 * @param attr_handle Unused attribute handle.
 * @param ctxt NimBLE access context.
 * @param arg Unused callback context.
 * @return 0 on success, otherwise an ATT error code.
 */
static int provisioning_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, s_status, strlen(s_status)) == 0 ?
            0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (ble_uuid_cmp(ctxt->chr->uuid, &s_ssid_uuid.u) == 0) {
        int result = write_text(ctxt->om, s_ssid, sizeof(s_ssid));
        if (result == 0) {
            set_status("credentials", "SSID received");
        }
        return result;
    }
    if (ble_uuid_cmp(ctxt->chr->uuid, &s_password_uuid.u) == 0) {
        int result = write_text(ctxt->om, s_password, sizeof(s_password));
        if (result == 0) {
            set_status("credentials", "password received");
        }
        return result;
    }
    if (ble_uuid_cmp(ctxt->chr->uuid, &s_control_uuid.u) == 0) {
        char command[16];
        int result = write_text(ctxt->om, command, sizeof(command));
        if (result != 0) {
            return result;
        }
        if (strcmp(command, "START") == 0) {
            if (s_ssid[0] == '\0' || s_password[0] == '\0') {
                set_status("failed", "SSID or password missing");
                return BLE_ATT_ERR_UNLIKELY;
            }
            set_status("connecting", "Wi-Fi connection requested");
            esp_err_t wifi_result = wifi_manager_apply_credentials(s_ssid, s_password);
            if (wifi_result != ESP_OK) {
                set_status("failed", "Wi-Fi request rejected");
                return BLE_ATT_ERR_UNLIKELY;
            }
            return 0;
        }
        if (strcmp(command, "CANCEL") == 0) {
            s_ssid[0] = '\0';
            s_password[0] = '\0';
            set_status("idle", "candidate cleared");
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/**
 * @brief Keeps provisioning advertisements available after mobile disconnects.
 *
 * @param event NimBLE GAP event.
 * @param arg Unused callback context.
 * @return 0 after handling.
 */
static int provisioning_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            s_connection_handle = event->connect.conn_handle;
            set_status("ready", "write SSID, password, then START");
        } else {
            ble_provisioning_service_start();
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_provisioning_service_start();
    }
    return 0;
}
