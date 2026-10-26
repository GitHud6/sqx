#include "ota_manager.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_gateway.h"
#include "sdkconfig.h"

#define OTA_URL_MAX_LENGTH 256U
#define OTA_QUEUE_LENGTH 1U
#define OTA_TASK_STACK_SIZE 12288U
#define OTA_TASK_PRIORITY 5U
#define OTA_HTTP_TIMEOUT_MS 10000
#define OTA_READ_BUFFER_SIZE 4096U

typedef struct {
    char url[OTA_URL_MAX_LENGTH];
} ota_request_t;

static const char *TAG = "ota_manager";
static QueueHandle_t s_request_queue;

static esp_err_t enqueue_ota_request(const char *json, size_t length, void *context);
static void ota_task(void *argument);
static esp_err_t download_and_install(const char *url);

/**
 * @brief Validates an HTTP OTA command and queues one firmware URL.
 *
 * @param json OTA command JSON bytes.
 * @param length OTA command JSON length.
 * @param context Unused callback context.
 * @return ESP_OK when queued, otherwise an error code.
 */
static esp_err_t enqueue_ota_request(const char *json, size_t length, void *context)
{
    (void)context;
    if (json == NULL || length == 0 ||
        length > CONFIG_NETGATE_MQTT_OTA_MAX_BYTES || s_request_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    cJSON *root = cJSON_ParseWithLength(json, length);
    cJSON *url_item = root == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(root, "url");
    if (!cJSON_IsString(url_item) || url_item->valuestring == NULL ||
        strncmp(url_item->valuestring, "http://", 7U) != 0 ||
        strlen(url_item->valuestring) >= OTA_URL_MAX_LENGTH) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    ota_request_t request = {0};
    snprintf(request.url, sizeof(request.url), "%s", url_item->valuestring);
    cJSON_Delete(root);
    return xQueueSend(s_request_queue, &request, 0) == pdTRUE ?
        ESP_OK : ESP_ERR_TIMEOUT;
}

/**
 * @brief Runs queued OTA downloads outside MQTT event context.
 *
 * @param argument Unused task argument.
 * @return None.
 */
static void ota_task(void *argument)
{
    (void)argument;
    ota_request_t request;
    while (true) {
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) == pdTRUE) {
            mqtt_gateway_publish_ota_status("queued", ESP_OK, 0, 0);
            esp_err_t result = download_and_install(request.url);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "OTA update failed: %s", esp_err_to_name(result));
                mqtt_gateway_publish_ota_status("failed", result, 0, 0);
            }
        }
    }
}

/**
 * @brief Downloads an HTTP image into the inactive OTA slot and selects it for boot.
 *
 * @param url HTTP URL of the firmware image.
 * @return ESP_OK is returned only if boot selection succeeds.
 */
static esp_err_t download_and_install(const char *url)
{
    if (url == NULL || strncmp(url, "http://", 7U) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    const esp_http_client_config_t http_config = {
        .url = url,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .buffer_size = OTA_READ_BUFFER_SIZE,
        .buffer_size_tx = 1024,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_config);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t result = esp_http_client_open(client, 0);
    int64_t content_length = result == ESP_OK ? esp_http_client_fetch_headers(client) : -1;
    if (result == ESP_OK && esp_http_client_get_status_code(client) != 200) {
        result = ESP_FAIL;
    }
    if (result == ESP_OK &&
        (content_length <= 0 || (uint64_t)content_length > partition->size)) {
        result = ESP_ERR_INVALID_SIZE;
    }

    esp_ota_handle_t ota_handle = 0;
    bool ota_started = false;
    if (result == ESP_OK) {
        result = esp_ota_begin(partition, (size_t)content_length, &ota_handle);
        ota_started = result == ESP_OK;
    }
    if (result == ESP_OK) {
        mqtt_gateway_publish_ota_status("downloading", ESP_OK, 0,
                                        (size_t)content_length);
    }

    char buffer[OTA_READ_BUFFER_SIZE];
    size_t received = 0;
    size_t next_report = 10U;
    while (result == ESP_OK && received < (size_t)content_length) {
        size_t remaining = (size_t)content_length - received;
        int read_length = esp_http_client_read(client, buffer,
                                               remaining < sizeof(buffer) ?
                                               (int)remaining : (int)sizeof(buffer));
        if (read_length <= 0) {
            result = ESP_FAIL;
            break;
        }
        result = esp_ota_write(ota_handle, buffer, (size_t)read_length);
        if (result != ESP_OK) {
            break;
        }
        received += (size_t)read_length;
        size_t percent = received * 100U / (size_t)content_length;
        if (percent >= next_report) {
            ESP_LOGI(TAG, "OTA download progress: %u%%", (unsigned int)percent);
            mqtt_gateway_publish_ota_status("downloading", ESP_OK, received,
                                            (size_t)content_length);
            next_report = percent + 10U;
        }
    }

    if (ota_started) {
        if (result == ESP_OK) {
            result = esp_ota_end(ota_handle);
            ota_started = false;
        } else {
            esp_ota_abort(ota_handle);
            ota_started = false;
        }
    }
    if (result == ESP_OK) {
        result = esp_ota_set_boot_partition(partition);
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "OTA image accepted; rebooting into the update slot");
        mqtt_gateway_publish_ota_status("rebooting", ESP_OK, received,
                                        (size_t)content_length);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    return result;
}

/**
 * @brief Creates the OTA worker and binds the MQTT OTA topic consumer.
 *
 * @return ESP_OK when initialized, otherwise an error code.
 */
esp_err_t ota_manager_init(void)
{
    if (s_request_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    s_request_queue = xQueueCreate(OTA_QUEUE_LENGTH, sizeof(ota_request_t));
    if (s_request_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ota_task, "ota_update", OTA_TASK_STACK_SIZE, NULL,
                    OTA_TASK_PRIORITY, NULL) != pdPASS) {
        vQueueDelete(s_request_queue);
        s_request_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return mqtt_gateway_set_ota_handler(enqueue_ota_request, NULL);
}

/**
 * @brief Confirms a pending image after critical gateway services initialize.
 *
 * @return ESP_OK when no confirmation is needed or the image is accepted.
 */
esp_err_t ota_manager_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    esp_ota_img_states_t state;
    esp_err_t result = esp_ota_get_state_partition(running, &state);
    if (result != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY) {
        return result == ESP_ERR_NOT_SUPPORTED || result == ESP_ERR_NOT_FOUND ?
            ESP_OK : result;
    }
    result = esp_ota_mark_app_valid_cancel_rollback();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "OTA image validation failed; requesting rollback");
        esp_ota_mark_app_invalid_rollback_and_reboot();
        return result;
    }
    ESP_LOGI(TAG, "OTA image confirmed after core service initialization");
    return ESP_OK;
}

/**
 * @brief Rejects a pending image and requests bootloader rollback.
 *
 * @return ESP_OK when no pending image exists, otherwise an error code.
 */
esp_err_t ota_manager_reject_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    esp_ota_img_states_t state;
    esp_err_t result = esp_ota_get_state_partition(running, &state);
    if (result != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY) {
        return result == ESP_ERR_NOT_SUPPORTED || result == ESP_ERR_NOT_FOUND ?
            ESP_OK : result;
    }
    result = esp_ota_mark_app_invalid_rollback_and_reboot();
    ESP_LOGE(TAG, "Rejected pending OTA image; rollback request returned: %s",
             esp_err_to_name(result));
    return result;
}
