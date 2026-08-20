#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace ray2::driver {

constexpr int kUart0TxPin = 43;
constexpr int kUart0RxPin = 44;
constexpr int kUart0BaudRate = 115200;
constexpr int kUart0RxBufferSize = 2048;
constexpr int kUart0TxBufferSize = 2048;
constexpr std::size_t kUart0RxChunkSize = 128;
constexpr std::size_t kUart0RxQueueLength = 16;

struct Uart0RxChunk {
    std::size_t length;
    std::uint8_t data[kUart0RxChunkSize];
};

esp_err_t uart0_start();
QueueHandle_t uart0_rx_queue();
int uart0_write(const std::uint8_t* data, std::size_t length);

}  // namespace ray2::driver
