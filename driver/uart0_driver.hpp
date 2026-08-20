#pragma once

#include "esp_err.h"

namespace ray2::driver {

constexpr int kUart0TxPin = 43;
constexpr int kUart0RxPin = 44;
constexpr int kUart0BaudRate = 115200;
constexpr int kUart0RxBufferSize = 2048;
constexpr int kUart0TxBufferSize = 2048;

esp_err_t uart0_init();

}  // namespace ray2::driver
