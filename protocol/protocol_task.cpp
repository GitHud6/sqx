#include "protocol_task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "uart0_driver.hpp"

namespace ray2::protocol {

namespace {

TaskHandle_t s_protocol_task = nullptr;

void protocol_task(void*) {
    ray2::driver::Uart0RxChunk chunk = {};
    QueueHandle_t rx_queue = ray2::driver::uart0_rx_queue();

    while (true) {
        if (xQueueReceive(rx_queue, &chunk, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        std::size_t offset = 0;
        while (offset < chunk.length) {
            const int written = ray2::driver::uart0_write(chunk.data + offset, chunk.length - offset);
            if (written <= 0) {
                break;
            }
            offset += static_cast<std::size_t>(written);
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
