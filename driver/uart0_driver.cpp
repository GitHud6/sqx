#include "uart0_driver.hpp"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ray2::driver {

namespace {

QueueHandle_t s_rx_queue = nullptr;
TaskHandle_t s_rx_task = nullptr;

esp_err_t uart0_init() {
    const uart_config_t config = {
        .baud_rate = kUart0BaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    esp_err_t result = uart_param_config(UART_NUM_0, &config);
    if (result != ESP_OK) {
        return result;
    }

    result = uart_set_pin(UART_NUM_0, kUart0TxPin, kUart0RxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        return result;
    }

    return uart_driver_install(UART_NUM_0, kUart0RxBufferSize, kUart0TxBufferSize, 0, nullptr, 0);
}

void uart0_rx_task(void*) {
    Uart0RxChunk chunk = {};

    while (true) {
        const int bytes_read = uart_read_bytes(
            UART_NUM_0, chunk.data, sizeof(chunk.data), pdMS_TO_TICKS(20));
        if (bytes_read <= 0) {
            continue;
        }

        chunk.length = static_cast<std::size_t>(bytes_read);
        xQueueSend(s_rx_queue, &chunk, portMAX_DELAY);
    }
}

}  // namespace

esp_err_t uart0_start() {
    if (s_rx_task != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = uart0_init();
    if (result != ESP_OK) {
        return result;
    }

    s_rx_queue = xQueueCreate(kUart0RxQueueLength, sizeof(Uart0RxChunk));
    if (s_rx_queue == nullptr) {
        uart_driver_delete(UART_NUM_0);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(uart0_rx_task, "uart0_rx", 4096, nullptr, 8, &s_rx_task) != pdPASS) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = nullptr;
        uart_driver_delete(UART_NUM_0);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

QueueHandle_t uart0_rx_queue() {
    return s_rx_queue;
}

int uart0_write(const std::uint8_t* data, std::size_t length) {
    return uart_write_bytes(UART_NUM_0, reinterpret_cast<const char*>(data), length);
}

}  // namespace ray2::driver
