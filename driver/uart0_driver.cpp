#include "uart0_driver.hpp"

#include <atomic>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "runtime_affinity.hpp"

namespace ray2::driver {

namespace {

QueueHandle_t s_rx_queue = nullptr;
TaskHandle_t s_rx_task = nullptr;
SemaphoreHandle_t s_tx_mutex = nullptr;
std::atomic<Uart0IngressHandler> s_ingress_handler = nullptr;
std::atomic<void*> s_ingress_user_data = nullptr;
std::atomic<bool> s_discard_normal_input_requested = false;
std::atomic<std::uint32_t> s_normal_input_generation = 0;

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

        normal_chunk.length = 0;
        const Uart0IngressHandler handler = s_ingress_handler.load(std::memory_order_acquire);
        void* const user_data = s_ingress_user_data.load(std::memory_order_acquire);

        for (int index = 0; index < bytes_read; ++index) {
            const std::uint8_t byte = read_chunk.data[index];
            if (handler != nullptr && handler(byte, user_data)) {
                continue;
            }
            normal_chunk.data[normal_chunk.length++] = byte;
        }

        // A realtime abort can arrive in this same chunk.  The UART task owns
        // both the driver RX buffer and the normal protocol queue, so it is
        // the only task allowed to discard them together.
        if (s_discard_normal_input_requested.exchange(false, std::memory_order_acq_rel)) {
            normal_chunk.length = 0;
            xQueueReset(s_rx_queue);
            uart_flush_input(UART_NUM_0);
        }

        if (normal_chunk.length == 0) {
            continue;
        }

        normal_chunk.generation = s_normal_input_generation.load(std::memory_order_acquire);
        xQueueSend(s_rx_queue, &normal_chunk, portMAX_DELAY);
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

    if (xTaskCreatePinnedToCore(uart0_rx_task, "uart0_rx", 4096, nullptr, 8, &s_rx_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
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

void uart0_discard_pending_normal_input() {
    s_normal_input_generation.fetch_add(1, std::memory_order_acq_rel);
    s_discard_normal_input_requested.store(true, std::memory_order_release);
}

std::uint32_t uart0_normal_input_generation() {
    return s_normal_input_generation.load(std::memory_order_acquire);
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
