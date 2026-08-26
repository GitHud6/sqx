#include "uart0_driver.hpp"

#include <atomic>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace ray2::driver {

namespace {

QueueHandle_t s_rx_queue = nullptr;
TaskHandle_t s_rx_task = nullptr;
SemaphoreHandle_t s_tx_mutex = nullptr;
std::atomic<Uart0IngressHandler> s_ingress_handler = nullptr;
std::atomic<void*> s_ingress_user_data = nullptr;
std::atomic<std::uint64_t> s_received_bytes = 0;
std::atomic<std::uint64_t> s_realtime_bytes = 0;
std::atomic<std::uint64_t> s_normal_bytes = 0;
std::atomic<std::size_t> s_queue_high_water_chunks = 0;

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
    Uart0RxChunk read_chunk = {};
    Uart0RxChunk normal_chunk = {};

    while (true) {
        const int bytes_read = uart_read_bytes(
            UART_NUM_0, read_chunk.data, sizeof(read_chunk.data), pdMS_TO_TICKS(20));
        if (bytes_read <= 0) {
            continue;
        }

        s_received_bytes.fetch_add(static_cast<std::uint64_t>(bytes_read), std::memory_order_relaxed);
        normal_chunk.length = 0;
        const Uart0IngressHandler handler = s_ingress_handler.load(std::memory_order_acquire);
        void* const user_data = s_ingress_user_data.load(std::memory_order_acquire);

        for (int index = 0; index < bytes_read; ++index) {
            const std::uint8_t byte = read_chunk.data[index];
            if (handler != nullptr && handler(byte, user_data)) {
                s_realtime_bytes.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            normal_chunk.data[normal_chunk.length++] = byte;
        }

        if (normal_chunk.length == 0) {
            continue;
        }

        s_normal_bytes.fetch_add(normal_chunk.length, std::memory_order_relaxed);
        xQueueSend(s_rx_queue, &normal_chunk, portMAX_DELAY);

        const std::size_t queued_chunks = static_cast<std::size_t>(uxQueueMessagesWaiting(s_rx_queue));
        std::size_t observed_high_water = s_queue_high_water_chunks.load(std::memory_order_relaxed);
        while (queued_chunks > observed_high_water &&
               !s_queue_high_water_chunks.compare_exchange_weak(
                   observed_high_water, queued_chunks, std::memory_order_relaxed)) {
        }
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

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == nullptr) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = nullptr;
        uart_driver_delete(UART_NUM_0);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(uart0_rx_task, "uart0_rx", 4096, nullptr, 8, &s_rx_task) != pdPASS) {
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = nullptr;
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

esp_err_t uart0_set_ingress_handler(const Uart0IngressHandler handler, void* const user_data) {
    if (s_rx_task == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    s_ingress_user_data.store(user_data, std::memory_order_release);
    s_ingress_handler.store(handler, std::memory_order_release);
    return ESP_OK;
}

Uart0RxDiagnostics uart0_rx_diagnostics() {
    return {
        .received_bytes = s_received_bytes.load(std::memory_order_relaxed),
        .realtime_bytes = s_realtime_bytes.load(std::memory_order_relaxed),
        .normal_bytes = s_normal_bytes.load(std::memory_order_relaxed),
        .queue_high_water_chunks = s_queue_high_water_chunks.load(std::memory_order_relaxed),
    };
}

int uart0_write_unlocked(const std::uint8_t* data, const std::size_t length) {
    return uart_write_bytes(UART_NUM_0, reinterpret_cast<const char*>(data), length);
}

bool uart0_write_all(const std::uint8_t* data, const std::size_t length) {
    if (data == nullptr || s_tx_mutex == nullptr) {
        return false;
    }

    if (xSemaphoreTake(s_tx_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    std::size_t offset = 0;
    while (offset < length) {
        const int written = uart0_write_unlocked(data + offset, length - offset);
        if (written <= 0) {
            xSemaphoreGive(s_tx_mutex);
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }

    xSemaphoreGive(s_tx_mutex);
    return true;
}

int uart0_write(const std::uint8_t* data, const std::size_t length) {
    if (data == nullptr || s_tx_mutex == nullptr) {
        return -1;
    }
    if (xSemaphoreTake(s_tx_mutex, portMAX_DELAY) != pdTRUE) {
        return -1;
    }
    const int written = uart0_write_unlocked(data, length);
    xSemaphoreGive(s_tx_mutex);
    return written;
}

}  // namespace ray2::driver
