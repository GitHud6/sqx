#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define NETGATE_C3_FIXTURE_VARIANT_RAY7 0
#define NETGATE_C3_FIXTURE_VARIANT_DEVICE1 1

/* Override this macro in the build or change the default for a local fixture. */
#ifndef NETGATE_C3_FIXTURE_VARIANT
#define NETGATE_C3_FIXTURE_VARIANT NETGATE_C3_FIXTURE_VARIANT_RAY7
#endif

#define FIXTURE_NOTIFY_PERIOD_US (2000000ULL)
#define BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16 0x2901U

#if NETGATE_C3_FIXTURE_VARIANT == NETGATE_C3_FIXTURE_VARIANT_RAY7
#define FIXTURE_DEVICE_NAME "Netgate-C3-Device0"
#define FIXTURE_SERVICE_UUID_TEXT "b3a5f6c0-1f3d-4d91-8a50-53d0a4a07301"
#define FIXTURE_RX_UUID_TEXT "b3a5f6c0-1f3d-4d91-8a50-53d0a4a07302"
#define FIXTURE_TX_UUID_TEXT "b3a5f6c0-1f3d-4d91-8a50-53d0a4a07303"
#define FIXTURE_SERVICE_UUID_INIT \
    BLE_UUID128_INIT(0x01, 0x73, 0xa0, 0xa4, 0xd0, 0x53, 0x50, 0x8a, \
                     0x91, 0x4d, 0x3d, 0x1f, 0xc0, 0xf6, 0xa5, 0xb3)
#define FIXTURE_RX_UUID_INIT \
    BLE_UUID128_INIT(0x02, 0x73, 0xa0, 0xa4, 0xd0, 0x53, 0x50, 0x8a, \
                     0x91, 0x4d, 0x3d, 0x1f, 0xc0, 0xf6, 0xa5, 0xb3)
#define FIXTURE_TX_UUID_INIT \
    BLE_UUID128_INIT(0x03, 0x73, 0xa0, 0xa4, 0xd0, 0x53, 0x50, 0x8a, \
                     0x91, 0x4d, 0x3d, 0x1f, 0xc0, 0xf6, 0xa5, 0xb3)
#elif NETGATE_C3_FIXTURE_VARIANT == NETGATE_C3_FIXTURE_VARIANT_DEVICE1
#define FIXTURE_DEVICE_NAME "Netgate-C3-Device1"
#define FIXTURE_SERVICE_UUID_TEXT "c4b6f7d1-2f4e-5e92-9b61-64e1b5b08412"
#define FIXTURE_RX_UUID_TEXT "c4b6f7d1-2f4e-5e92-9b61-64e1b5b08413"
#define FIXTURE_TX_UUID_TEXT "c4b6f7d1-2f4e-5e92-9b61-64e1b5b08414"
#define FIXTURE_SERVICE_UUID_INIT \
    BLE_UUID128_INIT(0x12, 0x84, 0xb0, 0xb5, 0xe1, 0x64, 0x61, 0x9b, \
                     0x92, 0x5e, 0x4e, 0x2f, 0xd1, 0xf7, 0xb6, 0xc4)
#define FIXTURE_RX_UUID_INIT \
    BLE_UUID128_INIT(0x13, 0x84, 0xb0, 0xb5, 0xe1, 0x64, 0x61, 0x9b, \
                     0x92, 0x5e, 0x4e, 0x2f, 0xd1, 0xf7, 0xb6, 0xc4)
#define FIXTURE_TX_UUID_INIT \
    BLE_UUID128_INIT(0x14, 0x84, 0xb0, 0xb5, 0xe1, 0x64, 0x61, 0x9b, \
                     0x92, 0x5e, 0x4e, 0x2f, 0xd1, 0xf7, 0xb6, 0xc4)
#else
#error "Unsupported NETGATE_C3_FIXTURE_VARIANT"
#endif

static const char *TAG = "ble_fixture";
/* Service UUID: selected by NETGATE_C3_FIXTURE_VARIANT. */
static const ble_uuid128_t s_service_uuid =
    FIXTURE_SERVICE_UUID_INIT;
/* RX UUID: selected by NETGATE_C3_FIXTURE_VARIANT. */
static const ble_uuid128_t s_rx_uuid =
    FIXTURE_RX_UUID_INIT;
/* TX UUID: selected by NETGATE_C3_FIXTURE_VARIANT. */
static const ble_uuid128_t s_tx_uuid =
    FIXTURE_TX_UUID_INIT;
static const char s_rx_description[] = "Gateway command input";
static const char s_tx_description[] = "Fixture telemetry notification";

static uint16_t s_tx_value_handle;
static uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint32_t s_sequence;
static esp_timer_handle_t s_notify_timer;
static bool s_notifications_enabled;

static int fixture_gap_event(struct ble_gap_event *event, void *arg);
static int fixture_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static int description_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
static void on_sync(void);
static void host_task(void *arg);
static esp_err_t start_advertising(void);
static void notify_timer_callback(void *arg);
static void send_telemetry(void);

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_rx_uuid.u,
                .access_cb = fixture_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_rx_description,
                    },
                    {0},
                },
            },
            {
                .uuid = &s_tx_uuid.u,
                .access_cb = fixture_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_value_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CHAR_USER_DESCRIPTION_UUID16),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = description_access,
                        .arg = (void *)s_tx_description,
                    },
                    {0},
                },
            },
            {0},
        },
    },
    {0},
};

/**
 * @brief Starts the ESP32-C3 BLE test fixture.
 *
 * @return None.
 */
void app_main(void)
{
    int result = nimble_port_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE initialization failed: %d", result);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(FIXTURE_DEVICE_NAME);
    result = ble_gatts_count_cfg(s_services);
    if (result == 0) {
        result = ble_gatts_add_svcs(s_services);
    }
    if (result != 0) {
        ESP_LOGE(TAG, "GATT registration failed: %d", result);
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = notify_timer_callback,
        .name = "fixture_notify",
    };
    esp_err_t timer_result = esp_timer_create(&timer_args, &s_notify_timer);
    if (timer_result != ESP_OK) {
        ESP_LOGE(TAG, "Notification timer initialization failed: %s",
                 esp_err_to_name(timer_result));
        return;
    }

    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
}

/**
 * @brief Starts advertising after the NimBLE host obtains an identity address.
 *
 * @return None.
 */
static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0) {
        ESP_LOGE(TAG, "Cannot obtain a BLE identity address");
        return;
    }
    esp_err_t result = start_advertising();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Cannot start advertising: %s", esp_err_to_name(result));
    }
}

/**
 * @brief Configures the connectable advertisement for the test fixture.
 *
 * @return ESP_OK when advertising starts, otherwise ESP_FAIL.
 */
static esp_err_t start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int result = ble_gap_adv_set_fields(&fields);
    if (result != 0) {
        ESP_LOGE(TAG, "Advertising data setup failed: %d", result);
        return ESP_FAIL;
    }
    struct ble_hs_adv_fields response_fields = {0};
    response_fields.name = (const uint8_t *)FIXTURE_DEVICE_NAME;
    response_fields.name_len = strlen(FIXTURE_DEVICE_NAME);
    response_fields.name_is_complete = 1;
    result = ble_gap_adv_rsp_set_fields(&response_fields);
    if (result != 0) {
        ESP_LOGE(TAG, "Scan response data setup failed: %d", result);
        return ESP_FAIL;
    }
    struct ble_gap_adv_params parameters = {0};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
    result = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &parameters, fixture_gap_event, NULL);
    if (result != 0) {
        ESP_LOGE(TAG, "Advertising start failed: %d", result);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Advertising variant=%d as %s, service_uuid=%s",
             NETGATE_C3_FIXTURE_VARIANT, FIXTURE_DEVICE_NAME,
             FIXTURE_SERVICE_UUID_TEXT);
    return ESP_OK;
}

/**
 * @brief Handles central connection lifecycle events for the test fixture.
 *
 * @param event NimBLE GAP event data.
 * @param arg Unused user context.
 * @return 0 after handling the event.
 */
static int fixture_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connection_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Gateway connected, handle=%u", s_connection_handle);
        } else {
            ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Gateway disconnected, reason=%d", event->disconnect.reason);
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notifications_enabled = false;
        esp_timer_stop(s_notify_timer);
        start_advertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE: {
        bool was_enabled = s_notifications_enabled;
        s_notifications_enabled = event->subscribe.attr_handle == s_tx_value_handle &&
                                  event->subscribe.cur_notify;
        ESP_LOGI(TAG, "Subscription: attr=%u notify=%d", event->subscribe.attr_handle,
                 event->subscribe.cur_notify);
        if (!was_enabled && s_notifications_enabled) {
            esp_timer_start_periodic(s_notify_timer, FIXTURE_NOTIFY_PERIOD_US);
        } else if (was_enabled && !s_notifications_enabled) {
            esp_timer_stop(s_notify_timer);
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

/**
 * @brief Accepts RX writes and logs the raw command from the gateway.
 *
 * @param conn_handle GATT connection that accessed the characteristic.
 * @param attr_handle Accessed attribute handle.
 * @param ctxt NimBLE GATT access context.
 * @param arg Unused user context.
 * @return 0 on success, otherwise an ATT error code.
 */
static int fixture_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
        ble_uuid_cmp(ctxt->chr->uuid, &s_rx_uuid.u) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t length = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t command[BLE_ATT_ATTR_MAX_LEN + 1U];
    if (length > BLE_ATT_ATTR_MAX_LEN ||
        os_mbuf_copydata(ctxt->om, 0, length, command) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    command[length] = '\0';
    ESP_LOGI(TAG, "RX command (%u bytes): %s", length, command);
    return 0;
}

/**
 * @brief Returns a Characteristic User Description descriptor string.
 *
 * @param conn_handle GATT connection that read the descriptor.
 * @param attr_handle Descriptor attribute handle.
 * @param ctxt NimBLE GATT access context.
 * @param arg Null-terminated descriptor text.
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
 * @brief Sends one deterministic JSON telemetry notification to the gateway.
 *
 * @return None.
 */
static void send_telemetry(void)
{
    if (s_connection_handle == BLE_HS_CONN_HANDLE_NONE || !s_notifications_enabled) {
        return;
    }
    char telemetry[96];
    uint32_t sequence = s_sequence++;
    int length = snprintf(telemetry, sizeof(telemetry),
                          "{\"sequence\":%" PRIu32 ",\"temperature_c\":%" PRIu32
                          ",\"voltage_mv\":%" PRIu32 "}",
                          sequence, 25U + (sequence % 5U),
                          3700U + (sequence % 30U));
    if (length < 0 || (size_t)length >= sizeof(telemetry)) {
        ESP_LOGE(TAG, "Cannot encode telemetry frame");
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(telemetry, (uint16_t)length);
    if (om == NULL) {
        ESP_LOGW(TAG, "Cannot allocate telemetry buffer");
        return;
    }
    int result = ble_gatts_notify_custom(s_connection_handle, s_tx_value_handle, om);
    if (result != 0) {
        ESP_LOGW(TAG, "Telemetry notification failed: %d", result);
    } else {
        ESP_LOGI(TAG, "TX telemetry: %s", telemetry);
    }
}

/**
 * @brief Bridges the ESP timer callback into one GATT telemetry notification.
 *
 * @param arg Unused timer context.
 * @return None.
 */
static void notify_timer_callback(void *arg)
{
    (void)arg;
    send_telemetry();
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
}
