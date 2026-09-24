#include "network_command_adapter.hpp"

#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "command_dispatch.hpp"
#include "line_assembler.hpp"
#include "planner.hpp"
#include "runtime_affinity.hpp"
#include "service.hpp"
#include "websocket_service.hpp"

namespace laserctl::network {
namespace {

constexpr char kLogTag[] = "network";
// HTTP and WebSocket are control transports, not a bulk G-code stream.
// Eight complete lines preserve short command bursts while bounding DRAM.
constexpr std::size_t kIngressQueueLength = 8;
constexpr std::size_t kMaximumBatchLines = kIngressQueueLength - 2;
constexpr std::uint32_t kTaskStackBytes = 4096;
constexpr std::size_t kResponseCapacity = 1600;

enum class MessageType : std::uint8_t {
    BatchStart,
    Line,
    Realtime,
    BatchEnd,
};

struct Message {
    MessageType type = MessageType::Line;
    std::uint32_t generation = 0;
    std::size_t length = 0;
    char text[protocol::kLineBufferSize] = {};
};

struct ResponseCollector {
    std::size_t length = 0;
    std::uint8_t data[kResponseCapacity] = {};
};

QueueHandle_t s_ingress_queue = nullptr;
SemaphoreHandle_t s_submit_mutex = nullptr;
TaskHandle_t s_task = nullptr;
std::atomic<std::uint32_t> s_generation = 1;
std::atomic<bool> s_batch_reserved = false;
ResponseCollector s_response_collector = {};

bool response_writer(const std::uint8_t* const data, const std::size_t length, void* const user_data) {
    auto* const collector = static_cast<ResponseCollector*>(user_data);
    if (data == nullptr || length == 0 || collector == nullptr ||
        length > kResponseCapacity - collector->length) {
        return false;
    }
    std::memcpy(collector->data + collector->length, data, length);
    collector->length += length;
    return true;
}

bool equals_ignore_case(const char* const text, const std::size_t length,
                        const char* const expected) {
    const std::size_t expected_length = std::strlen(expected);
    if (length != expected_length) return false;
    for (std::size_t index = 0; index < length; ++index) {
        if (std::toupper(static_cast<unsigned char>(text[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) {
            return false;
        }
    }
    return true;
}

bool is_realtime_text(const char* const text, const std::size_t length) {
    return length == 1 && (text[0] == '?' || text[0] == '!' || text[0] == '~' ||
                           static_cast<std::uint8_t>(text[0]) == 0x18U);
}

void publish_text(const char* const text) {
    if (text == nullptr) return;
    (void)websocket::websocket_service_publish_binary(
        reinterpret_cast<const std::uint8_t*>(text), std::strlen(text));
}

void process_realtime(const Message& message) {
    if (message.length == 1 && message.text[0] == '?') {
        char status[160] = {};
        std::size_t status_length = 0;
        if (protocol::command_format_realtime_status(status, sizeof(status), status_length)) {
            (void)websocket::websocket_service_publish_realtime_status(
                reinterpret_cast<const std::uint8_t*>(status), status_length);
        } else {
            publish_text("error:protocol-no-mem\r\n");
        }
        return;
    }

    const bool hard_reset = message.length == 1 &&
                            static_cast<std::uint8_t>(message.text[0]) == 0x18U;
    const bool accepted = hard_reset ? service::service_request_hard_reset()
                          : message.length == 1 && message.text[0] == '!'
                              ? service::service_request_feed_hold()
                          : message.length == 1 && message.text[0] == '~'
                              ? service::service_request_cycle_start()
                              : false;
    publish_text(accepted ? "ok\r\n" : "error:realtime-rejected\r\n");
}

bool process_line(const Message& message) {
    while (message.generation == s_generation.load(std::memory_order_acquire)) {
        bool capacity_blocked = false;
        s_response_collector.length = 0;
        if (!protocol::command_dispatch_with_response(message.text, message.length, capacity_blocked,
                                                      response_writer, &s_response_collector,
                                                      service::InputSource::WebSocket)) {
            publish_text("error:network-dispatch-failed\r\n");
            return false;
        }
        if (!capacity_blocked) {
            if (s_response_collector.length != 0) {
                (void)websocket::websocket_service_publish_binary(s_response_collector.data,
                                                                   s_response_collector.length);
            }
            return true;
        }
        planner::planner_wait_for_free_block();
    }
    return false;
}

void finish_batch(const std::uint32_t generation) {
    if (generation != s_generation.load(std::memory_order_acquire)) return;
    (void)service::service_input_ended(service::InputSource::WebSocket);

    while (generation == s_generation.load(std::memory_order_acquire) &&
           !planner::planner_execution_is_drained()) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void network_task(void*) {
    bool batch_active = false;
    bool batch_has_data = false;
    bool batch_rejected = false;
    std::uint32_t batch_generation = 0;
    Message message = {};

    while (true) {
        if (xQueueReceive(s_ingress_queue, &message, portMAX_DELAY) != pdTRUE) continue;

        if (message.generation != s_generation.load(std::memory_order_acquire)) continue;
        switch (message.type) {
            case MessageType::BatchStart:
                batch_active = true;
                batch_has_data = false;
                batch_rejected = !service::service_input_started(service::InputSource::WebSocket);
                batch_generation = message.generation;
                if (batch_rejected) publish_text("error:network-input-unavailable\r\n");
                break;
            case MessageType::Line:
                if (batch_active && batch_rejected) break;
                if (batch_active && !batch_has_data) {
                    batch_has_data = true;
                    if (!service::service_input_data_available(service::InputSource::WebSocket)) {
                        batch_rejected = true;
                        publish_text("error:network-input-unavailable\r\n");
                        break;
                    }
                }
                (void)process_line(message);
                break;
            case MessageType::Realtime:
                process_realtime(message);
                break;
            case MessageType::BatchEnd:
                if (batch_active && !batch_rejected) finish_batch(batch_generation);
                if (message.generation == s_generation.load(std::memory_order_acquire)) {
                    s_batch_reserved.store(false, std::memory_order_release);
                }
                batch_active = false;
                batch_has_data = false;
                batch_rejected = false;
                break;
        }
    }
}

bool trim_line(const char*& text, std::size_t& length) {
    while (length != 0 && std::isspace(static_cast<unsigned char>(*text)) != 0) {
        ++text;
        --length;
    }
    while (length != 0 && std::isspace(static_cast<unsigned char>(text[length - 1])) != 0) {
        --length;
    }
    return length != 0;
}

bool count_lines(const char* const text, const std::size_t length, std::size_t& line_count) {
    line_count = 0;
    std::size_t start = 0;
    while (start < length) {
        std::size_t end = start;
        while (end < length && text[end] != '\r' && text[end] != '\n') ++end;
        const char* line = text + start;
        std::size_t line_length = end - start;
        if (trim_line(line, line_length)) {
            if (line_length >= protocol::kLineBufferSize || ++line_count > kMaximumBatchLines) {
                return false;
            }
        }
        while (end < length && (text[end] == '\r' || text[end] == '\n')) ++end;
        start = end;
    }
    return line_count != 0;
}

bool plain_line_is_supported(const char* const text, const std::size_t length) {
    const char* line = text;
    std::size_t line_length = length;
    if (!trim_line(line, line_length)) return false;
    return line[0] == '$' || is_realtime_text(line, line_length);
}

bool single_trimmed_line(const char* const text, const std::size_t length, const char*& line,
                         std::size_t& line_length) {
    line = text;
    line_length = length;
    if (!trim_line(line, line_length)) return false;
    for (std::size_t index = 0; index < line_length; ++index) {
        if (line[index] == '\r' || line[index] == '\n') return false;
    }
    return true;
}

bool enqueue_message(const MessageType type, const char* const text, const std::size_t length,
                     const std::uint32_t generation) {
    Message message = {};
    message.type = type;
    message.generation = generation;
    message.length = length;
    if (text != nullptr && length != 0) std::memcpy(message.text, text, length);
    return xQueueSend(s_ingress_queue, &message, 0) == pdTRUE;
}

bool enqueue_lines(const char* const text, const std::size_t length, const std::uint32_t generation) {
    std::size_t start = 0;
    while (start < length) {
        std::size_t end = start;
        while (end < length && text[end] != '\r' && text[end] != '\n') ++end;
        const char* line = text + start;
        std::size_t line_length = end - start;
        if (trim_line(line, line_length)) {
            const MessageType type = is_realtime_text(line, line_length)
                                         ? MessageType::Realtime
                                         : MessageType::Line;
            if (!enqueue_message(type, line, line_length, generation)) return false;
        }
        while (end < length && (text[end] == '\r' || text[end] == '\n')) ++end;
        start = end;
    }
    return true;
}

}  // namespace

esp_err_t network_command_adapter_start() {
    if (s_task != nullptr) return ESP_ERR_INVALID_STATE;
    s_ingress_queue = xQueueCreate(kIngressQueueLength, sizeof(Message));
    if (s_ingress_queue == nullptr) return ESP_ERR_NO_MEM;
    s_submit_mutex = xSemaphoreCreateMutex();
    if (s_submit_mutex == nullptr) {
        vQueueDelete(s_ingress_queue);
        s_ingress_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(network_task, "network_input", kTaskStackBytes, nullptr, 3, &s_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        vSemaphoreDelete(s_submit_mutex);
        s_submit_mutex = nullptr;
        vQueueDelete(s_ingress_queue);
        s_ingress_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(kLogTag, "started queue=%u", static_cast<unsigned>(kIngressQueueLength));
    return ESP_OK;
}

bool network_command_adapter_is_started() {
    return s_task != nullptr;
}

SubmitResult network_command_adapter_submit(const char* const text, const std::size_t length,
                                            const bool command_text) {
    if (text == nullptr || length == 0 || s_ingress_queue == nullptr || s_submit_mutex == nullptr ||
        s_task == nullptr) {
        return SubmitResult::Unavailable;
    }
    if (std::memchr(text, '\0', length) != nullptr) return SubmitResult::Invalid;

    std::size_t line_count = 0;
    if (!count_lines(text, length, line_count) || (!command_text && line_count != 1)) {
        return SubmitResult::Invalid;
    }
    if (!command_text && !plain_line_is_supported(text, length)) return SubmitResult::Invalid;
    const char* single_line = nullptr;
    std::size_t single_line_length = 0;
    if (line_count == 1 && single_trimmed_line(text, length, single_line, single_line_length) &&
        is_realtime_text(single_line, single_line_length)) {
        Message realtime = {};
        realtime.type = MessageType::Realtime;
        realtime.length = single_line_length;
        std::memcpy(realtime.text, single_line, single_line_length);
        process_realtime(realtime);
        return SubmitResult::Accepted;
    }
    // A standalone `$` command is a control request, not an input producer.
    // In particular, `$SD/RUN` starts its own SdCard lifecycle and must not
    // inherit a short-lived WebSocket lifecycle that would end immediately.
    const bool system_command = line_count == 1 && single_trimmed_line(text, length, single_line,
                                                                         single_line_length) &&
                                single_line[0] == '$';
    const bool lifecycle_batch = command_text && !system_command;
    const std::size_t required_slots = line_count + (lifecycle_batch ? 2U : 0U);
    if (required_slots > kIngressQueueLength) return SubmitResult::Invalid;
    if (lifecycle_batch && s_batch_reserved.exchange(true, std::memory_order_acq_rel)) {
        return SubmitResult::Busy;
    }
    if (xSemaphoreTake(s_submit_mutex, 0) != pdTRUE) {
        if (lifecycle_batch) s_batch_reserved.store(false, std::memory_order_release);
        return SubmitResult::Busy;
    }
    if (uxQueueSpacesAvailable(s_ingress_queue) < required_slots) {
        xSemaphoreGive(s_submit_mutex);
        if (lifecycle_batch) s_batch_reserved.store(false, std::memory_order_release);
        return SubmitResult::Busy;
    }

    const std::uint32_t generation = s_generation.load(std::memory_order_acquire);
    bool queued = !lifecycle_batch || enqueue_message(MessageType::BatchStart, nullptr, 0, generation);
    if (queued) queued = enqueue_lines(text, length, generation);
    if (queued && lifecycle_batch) queued = enqueue_message(MessageType::BatchEnd, nullptr, 0, generation);
    xSemaphoreGive(s_submit_mutex);

    if (!queued) {
        // Queue capacity was reserved under the mutex; this is an internal
        // consistency failure, so discard any partial batch before returning.
        network_command_adapter_abort_pending();
        return SubmitResult::Unavailable;
    }
    return SubmitResult::Accepted;
}

void network_command_adapter_abort_pending() {
    s_generation.fetch_add(1, std::memory_order_acq_rel);
    s_batch_reserved.store(false, std::memory_order_release);
    if (s_ingress_queue != nullptr) xQueueReset(s_ingress_queue);
}

}  // namespace laserctl::network
