#include "protocol_tx.hpp"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "runtime_affinity.hpp"

#include "uart0_driver.hpp"

namespace laserctl::protocol {
namespace {

constexpr std::size_t kNormalTxQueueLength = 4;
constexpr std::size_t kRealtimeStatusCapacity = 256;
constexpr std::size_t kNormalMessagesPerStatus = 4;

struct TxMessage {
    std::size_t length = 0;
    std::uint8_t data[kProtocolTxMessageCapacity] = {};
};

QueueHandle_t s_normal_tx_queue = nullptr;
QueueHandle_t s_free_tx_queue = nullptr;
TaskHandle_t s_tx_task = nullptr;
portMUX_TYPE s_realtime_status_lock = portMUX_INITIALIZER_UNLOCKED;
std::size_t s_realtime_status_length = 0;
std::uint8_t s_realtime_status[kRealtimeStatusCapacity] = {};
bool s_realtime_status_pending = false;

struct RealtimeStatusMessage {
    std::size_t length = 0;
    std::uint8_t data[kRealtimeStatusCapacity] = {};
};

TxMessage s_normal_tx_messages[kNormalTxQueueLength] = {};

bool take_realtime_status(RealtimeStatusMessage& message) {
    portENTER_CRITICAL(&s_realtime_status_lock);
    if (!s_realtime_status_pending) {
        portEXIT_CRITICAL(&s_realtime_status_lock);
        return false;
    }
    message.length = s_realtime_status_length;
    std::memcpy(message.data, s_realtime_status, message.length);
    s_realtime_status_pending = false;
    portEXIT_CRITICAL(&s_realtime_status_lock);
    return true;
}

void protocol_tx_task(void*) {
    TxMessage* normal_message = nullptr;
    RealtimeStatusMessage realtime_status = {};

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (true) {
            bool sent_any = false;
            for (std::size_t index = 0; index < kNormalMessagesPerStatus; ++index) {
                if (xQueueReceive(s_normal_tx_queue, &normal_message, 0) != pdTRUE) {
                    break;
                }
                (void)laserctl::driver::uart0_write_all(normal_message->data, normal_message->length);
                xQueueSend(s_free_tx_queue, &normal_message, portMAX_DELAY);
                sent_any = true;
            }

            if (take_realtime_status(realtime_status)) {
                (void)laserctl::driver::uart0_write_all(realtime_status.data, realtime_status.length);
                sent_any = true;
            }

            if (!sent_any) {
                break;
            }
        }
    }
}

}  // namespace

esp_err_t protocol_tx_start() {
    if (s_tx_task != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    s_normal_tx_queue = xQueueCreate(kNormalTxQueueLength, sizeof(TxMessage*));
    if (s_normal_tx_queue == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    s_free_tx_queue = xQueueCreate(kNormalTxQueueLength, sizeof(TxMessage*));
    if (s_free_tx_queue == nullptr) {
        vQueueDelete(s_normal_tx_queue);
        s_normal_tx_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    for (std::size_t index = 0; index < kNormalTxQueueLength; ++index) {
        TxMessage* message = &s_normal_tx_messages[index];
        xQueueSend(s_free_tx_queue, &message, 0);
    }
    if (xTaskCreatePinnedToCore(protocol_tx_task, "protocol_tx", 4096, nullptr, 7, &s_tx_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        vQueueDelete(s_free_tx_queue);
        s_free_tx_queue = nullptr;
        vQueueDelete(s_normal_tx_queue);
        s_normal_tx_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool protocol_tx_enqueue_normal(const std::uint8_t* data, const std::size_t length) {
    if (data == nullptr || length == 0 || length > kProtocolTxMessageCapacity ||
        s_normal_tx_queue == nullptr || s_free_tx_queue == nullptr || s_tx_task == nullptr) {
        return false;
    }

    TxMessage* message = nullptr;
    if (xQueueReceive(s_free_tx_queue, &message, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    message->length = length;
    std::memcpy(message->data, data, length);
    if (xQueueSend(s_normal_tx_queue, &message, portMAX_DELAY) != pdTRUE) {
        xQueueSend(s_free_tx_queue, &message, portMAX_DELAY);
        return false;
    }
    xTaskNotifyGive(s_tx_task);
    return true;
}

bool protocol_tx_publish_realtime_status(const std::uint8_t* data, const std::size_t length) {
    if (data == nullptr || length == 0 || length > kRealtimeStatusCapacity || s_tx_task == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&s_realtime_status_lock);
    if (s_realtime_status_pending) {
    }
    std::memcpy(s_realtime_status, data, length);
    s_realtime_status_length = length;
    s_realtime_status_pending = true;
    portEXIT_CRITICAL(&s_realtime_status_lock);
    xTaskNotifyGive(s_tx_task);
    return true;
}

void protocol_tx_abort_flush() {
    if (s_normal_tx_queue != nullptr && s_free_tx_queue != nullptr) {
        TxMessage* message = nullptr;
        while (xQueueReceive(s_normal_tx_queue, &message, 0) == pdTRUE) {
            xQueueSend(s_free_tx_queue, &message, 0);
        }
    }
    portENTER_CRITICAL(&s_realtime_status_lock);
    s_realtime_status_pending = false;
    s_realtime_status_length = 0;
    portEXIT_CRITICAL(&s_realtime_status_lock);
}

}  // namespace laserctl::protocol
