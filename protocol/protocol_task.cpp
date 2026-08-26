#include "protocol_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "command_dispatch.hpp"
#include "line_assembler.hpp"
#include "realtime_command.hpp"
#include "uart0_driver.hpp"

namespace ray2::protocol {

namespace {

TaskHandle_t s_protocol_task = nullptr;

bool protocol_write_all(const std::uint8_t* data, std::size_t length) {
    std::size_t offset = 0;
    while (offset < length) {
        const int written = ray2::driver::uart0_write(data + offset, length - offset);
        if (written <= 0) {
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }
    return true;
}

bool protocol_handle_ready_line(LineAssembler& assembler) {
    const bool dispatched = command_dispatch(assembler.line(), assembler.length());
    assembler.reset_line();
    return dispatched;
}

bool protocol_handle_overflow() {
    static constexpr std::uint8_t kOverflow[] = "error:overflow\r\n";
    return protocol_write_all(kOverflow, sizeof(kOverflow) - 1);
}

bool protocol_process_byte(LineAssembler& assembler, std::uint8_t byte) {
    RealtimeCommand realtime_command = {};
    if (realtime_command_from_byte(byte, realtime_command)) {
        realtime_command_receive(realtime_command);
        if (realtime_command == RealtimeCommand::StatusReport) {
            return command_report_realtime_status();
        }
        return true;
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

}  // namespace

esp_err_t protocol_start() {
    if (s_protocol_task != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ray2::driver::uart0_rx_queue() == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(protocol_task, "protocol", 4096, nullptr, 6, &s_protocol_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

}  // namespace ray2::protocol
