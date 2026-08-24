#include "esp_log.h"

#include "laser_driver.hpp"
#include "protocol_task.hpp"
#include "planner.hpp"
#include "uart0_driver.hpp"

extern "C" void app_main(void) {
    static const char* tag = "ray2-idf";

    ESP_ERROR_CHECK(ray2::laser::laser_init());
    ESP_ERROR_CHECK(ray2::driver::uart0_start());
    ESP_ERROR_CHECK(ray2::planner::planner_start());
    ESP_ERROR_CHECK(ray2::protocol::protocol_start());
    ESP_LOGI(tag,
             "UART0 protocol and planner ready: tx=%d, rx=%d, baud=%d",
             ray2::driver::kUart0TxPin,
             ray2::driver::kUart0RxPin,
             ray2::driver::kUart0BaudRate);
}
