#include "protocol_task.hpp"

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "runtime_affinity.hpp"

#include "command_dispatch.hpp"
#include "line_assembler.hpp"
#include "planner.hpp"
#include "protocol_tx.hpp"
#include "realtime_command.hpp"
#include "service.hpp"
#include "uart0_driver.hpp"

namespace laserctl::protocol {

namespace {

TaskHandle_t s_protocol_task = nullptr;
TaskHandle_t s_realtime_task = nullptr;

// Keep the protocol task stack sized for normal command and status handling.
constexpr std::uint32_t kProtocolTaskStackBytes = 7168;

std::atomic<bool> s_status_report_pending = false;

bool protocol_write_all(const std::uint8_t* data, std::size_t length) {
    return protocol_tx_enqueue_normal(data, length);
}

bool protocol_handle_ready_line(LineAssembler& assembler) {
    while (true) {
        // `planner_wait_for_free_block()` is allowed to sleep while a stream
        // is full.  Ctrl-X must also cancel that partially handled line once
        // planner_reset() wakes the capacity waiter.
        bool capacity_blocked = false;
        const bool dispatched = command_dispatch(
            assembler.line(), assembler.length(), capacity_blocked, service::InputSource::Uart);
        if (!capacity_blocked) {
            assembler.reset_line();
            return dispatched;
        }
        planner::planner_wait_for_free_block();
    }
}

bool protocol_handle_overflow() {
    static constexpr std::uint8_t kOverflow[] = "error:overflow\r\n";
    return protocol_write_all(kOverflow, sizeof(kOverflow) - 1);
}

bool protocol_handle_realtime_command(const RealtimeCommand realtime_command) {
    switch (realtime_command) {
        case RealtimeCommand::StatusReport:
            return command_report_realtime_status();
        case RealtimeCommand::HardReset:
            service::service_request_hard_reset();
            return true;
        case RealtimeCommand::FeedHold:
            service::service_request_feed_hold();
            return true;
        case RealtimeCommand::CycleStart:
            service::service_request_cycle_start();
            return true;
    }
    return true;
}

bool protocol_uart_ingress_handler(const std::uint8_t byte, void*) {
    RealtimeCommand realtime_command = {};
    if (!realtime_command_from_byte(byte, realtime_command)) {
        return false;
    }

    realtime_command_receive(realtime_command);
    switch (realtime_command) {
        case RealtimeCommand::StatusReport:
            s_status_report_pending.store(true, std::memory_order_release);
            break;
        case RealtimeCommand::HardReset:
            service::service_request_hard_reset();
            break;
        case RealtimeCommand::FeedHold:
            service::service_request_feed_hold();
            break;
        case RealtimeCommand::CycleStart:
            service::service_request_cycle_start();
            break;
    }

    if (s_realtime_task != nullptr) {
        xTaskNotifyGive(s_realtime_task);
    }
    return true;
}

bool protocol_process_byte(LineAssembler& assembler, const std::uint8_t byte) {
    // Keep this fallback for bytes received before protocol_start() has
    // registered the UART ingress handler.
    RealtimeCommand realtime_command = {};
    if (realtime_command_from_byte(byte, realtime_command)) {
        realtime_command_receive(realtime_command);
        return protocol_handle_realtime_command(realtime_command);
    }

    switch (assembler.push(byte)) {
        case LineEvent::Ready:
            return protocol_handle_ready_line(assembler);
        case LineEvent::Overflow:
            return protocol_handle_overflow();
        case LineEvent::None:
            return true;
    }

    return false;
}

void protocol_task(void*) {
    laserctl::driver::Uart0RxChunk chunk = {};
    QueueHandle_t rx_queue = laserctl::driver::uart0_rx_queue();
    LineAssembler assembler;

    while (true) {
        if (xQueueReceive(rx_queue, &chunk, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // A packet tagged before Ctrl-X is obsolete. New packets must remain
        // usable immediately after the reset, including the first `$H` or
        // A status request sent by a host.
        if (chunk.generation != laserctl::driver::uart0_normal_input_generation()) {
            assembler.reset_line();
            continue;
        }

        for (std::size_t index = 0; index < chunk.length; ++index) {
            if (!protocol_process_byte(assembler, chunk.data[index])) {
                break;
            }
        }
    }
}

void realtime_task(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_status_report_pending.exchange(false, std::memory_order_acq_rel)) {
            protocol_handle_realtime_command(RealtimeCommand::StatusReport);
        }
    }
}

}  // namespace

esp_err_t protocol_start() {
    if (s_protocol_task != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (laserctl::driver::uart0_rx_queue() == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t dispatch_start_result = command_dispatch_start();
    if (dispatch_start_result != ESP_OK) {
        return dispatch_start_result;
    }
    const esp_err_t tx_start_result = protocol_tx_start();
    if (tx_start_result != ESP_OK) {
        return tx_start_result;
    }

    if (xTaskCreatePinnedToCore(realtime_task, "realtime", 3072, nullptr, 7,
                                &s_realtime_task, runtime::kNonRealtimeCore) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(protocol_task, "protocol", kProtocolTaskStackBytes, nullptr, 6,
                                &s_protocol_task, runtime::kNonRealtimeCore) != pdPASS) {
        vTaskDelete(s_realtime_task);
        s_realtime_task = nullptr;
        return ESP_ERR_NO_MEM;
    }
    if (laserctl::driver::uart0_set_ingress_handler(protocol_uart_ingress_handler, nullptr) != ESP_OK) {
        vTaskDelete(s_protocol_task);
        s_protocol_task = nullptr;
        vTaskDelete(s_realtime_task);
        s_realtime_task = nullptr;
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

}  // namespace laserctl::protocol
