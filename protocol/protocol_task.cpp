#include "protocol_task.hpp"

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "command_dispatch.hpp"
#include "line_assembler.hpp"
#include "planner.hpp"
#include "protocol_tx.hpp"
#include "realtime_command.hpp"
#include "uart0_driver.hpp"

namespace ray2::protocol {

namespace {

TaskHandle_t s_protocol_task = nullptr;
TaskHandle_t s_realtime_task = nullptr;

// `$POS/STATUS` deliberately aggregates protocol, planner and driver snapshots.
// Its formatting path is deeper than ordinary G-code acknowledgement handling.
constexpr std::uint32_t kProtocolTaskStackBytes = 6144;

enum class PendingControl : std::uint8_t {
    None,
    Hold,
    Resume,
    Reset,
};

std::atomic<bool> s_status_report_pending = false;
std::atomic<PendingControl> s_control_pending = PendingControl::None;

bool protocol_write_all(const std::uint8_t* data, std::size_t length) {
    return protocol_tx_enqueue_normal(data, length);
}

bool protocol_handle_ready_line(LineAssembler& assembler) {
    while (true) {
        bool capacity_blocked = false;
        const bool dispatched = command_dispatch(
            assembler.line(), assembler.length(), capacity_blocked);
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
    if (realtime_command == RealtimeCommand::StatusReport) {
        return command_report_realtime_status();
    }
    if (realtime_command == RealtimeCommand::FeedHold) {
        planner::planner_request_feed_hold();
    } else if (realtime_command == RealtimeCommand::CycleStart) {
        planner::planner_resume();
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
        case RealtimeCommand::FeedHold:
            s_control_pending.store(PendingControl::Hold, std::memory_order_release);
            break;
        case RealtimeCommand::CycleStart:
            s_control_pending.store(PendingControl::Resume, std::memory_order_release);
            break;
        case RealtimeCommand::Reset:
            s_control_pending.store(PendingControl::Reset, std::memory_order_release);
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
    ray2::driver::Uart0RxChunk chunk = {};
    QueueHandle_t rx_queue = ray2::driver::uart0_rx_queue();
    LineAssembler assembler;

    while (true) {
        if (xQueueReceive(rx_queue, &chunk, portMAX_DELAY) != pdTRUE) {
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

        const PendingControl control = s_control_pending.exchange(
            PendingControl::None, std::memory_order_acq_rel);
        switch (control) {
            case PendingControl::Hold:
                protocol_handle_realtime_command(RealtimeCommand::FeedHold);
                break;
            case PendingControl::Resume:
                protocol_handle_realtime_command(RealtimeCommand::CycleStart);
                break;
            case PendingControl::Reset:
                // Reset/Abort cleanup is intentionally implemented in phase three.
                break;
            case PendingControl::None:
                break;
        }

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
    if (ray2::driver::uart0_rx_queue() == nullptr) {
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

    if (xTaskCreate(realtime_task, "realtime", 3072, nullptr, 7, &s_realtime_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(protocol_task, "protocol", kProtocolTaskStackBytes, nullptr, 6,
                    &s_protocol_task) != pdPASS) {
        vTaskDelete(s_realtime_task);
        s_realtime_task = nullptr;
        return ESP_ERR_NO_MEM;
    }
    if (ray2::driver::uart0_set_ingress_handler(protocol_uart_ingress_handler, nullptr) != ESP_OK) {
        vTaskDelete(s_protocol_task);
        s_protocol_task = nullptr;
        vTaskDelete(s_realtime_task);
        s_realtime_task = nullptr;
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

}  // namespace ray2::protocol
