#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace laserctl::driver {

constexpr int kUart0TxPin = 43;
constexpr int kUart0RxPin = 44;
constexpr int kUart0BaudRate = 115200;
constexpr int kUart0RxBufferSize = 2048;
constexpr int kUart0TxBufferSize = 2048;
constexpr std::size_t kUart0RxChunkSize = 128;
constexpr std::size_t kUart0RxQueueLength = 16;

// Called from the UART receive task. Return true only when the byte has been
// fully consumed and must not be forwarded to the normal command stream.
using Uart0IngressHandler = bool (*)(std::uint8_t byte, void* user_data);

struct Uart0RxChunk {
    std::size_t length;
    std::uint32_t generation;
    std::uint8_t data[kUart0RxChunkSize];
};

esp_err_t uart0_start();
QueueHandle_t uart0_rx_queue();
esp_err_t uart0_set_ingress_handler(Uart0IngressHandler handler, void* user_data);
// Drops normal (non-realtime) input waiting for the protocol task.
void uart0_discard_pending_normal_input();
// Monotonically advances whenever Ctrl-X discards the normal command stream.
std::uint32_t uart0_normal_input_generation();
int uart0_write(const std::uint8_t* data, std::size_t length);
bool uart0_write_all(const std::uint8_t* data, std::size_t length);

}  // namespace laserctl::driver
