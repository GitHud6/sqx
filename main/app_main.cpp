#include "esp_log.h"

#include "laser_driver.hpp"
#include "laser_planner_adapter.hpp"
#include "limit_switch_driver.hpp"
#include "lis2dh12_driver.hpp"
#include "accelerometer.hpp"
#include "homing_service.hpp"
#include "protocol_task.hpp"
#include "planner.hpp"
#include "sdcard_driver.hpp"
#include "service.hpp"
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
    ESP_ERROR_CHECK(ray2::driver::lis2dh12_init());
    ESP_ERROR_CHECK(ray2::accelerometer::init());
    ESP_ERROR_CHECK(ray2::homing::homing_init());
    ESP_ERROR_CHECK(ray2::planner::planner_start());
    ESP_ERROR_CHECK(ray2::laser::laser_attach_planner());
    ESP_ERROR_CHECK(ray2::service::service_start());
    ESP_ERROR_CHECK(ray2::protocol::protocol_start());
}
