#include "uart0_driver.hpp"

#include "driver/uart.h"

namespace ray2::driver {

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

}  // namespace ray2::driver
