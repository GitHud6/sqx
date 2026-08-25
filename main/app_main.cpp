#include "esp_log.h"

#include "laser_driver.hpp"
#include "laser_planner_adapter.hpp"
#include "limit_switch_driver.hpp"
#include "protocol_task.hpp"
#include "planner.hpp"
#include "sdcard_driver.hpp"
#include "timer_stepper_executor.hpp"
#include "uart0_driver.hpp"

extern "C" void app_main(void) {
    static const char* tag = "ray2-idf";

    ESP_ERROR_CHECK(ray2::laser::laser_init());
    const esp_err_t sd_result = ray2::driver::sdcard_start();
    if (sd_result != ESP_OK) {
        ESP_LOGW(tag, "SD card mount unavailable: %s", esp_err_to_name(sd_result));
    }
    ESP_ERROR_CHECK(ray2::driver::uart0_start());
    ESP_ERROR_CHECK(ray2::stepper::timer_stepper_executor_init());
    ESP_ERROR_CHECK(ray2::driver::limit_switch_driver_init());
    ESP_ERROR_CHECK(ray2::planner::planner_start());
    ESP_ERROR_CHECK(ray2::laser::laser_attach_planner());
    ESP_ERROR_CHECK(ray2::protocol::protocol_start());
    ESP_LOGI(tag,
             "UART0 protocol and planner ready: tx=%d, rx=%d, baud=%d",
             ray2::driver::kUart0TxPin,
             ray2::driver::kUart0RxPin,
             ray2::driver::kUart0BaudRate);
}
