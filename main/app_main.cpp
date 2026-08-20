#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart0_driver.hpp"

extern "C" void app_main(void) {
    static const char* tag = "ray2-idf";

    ESP_ERROR_CHECK(ray2::driver::uart0_init());
    ESP_LOGI(tag,
             "UART0 initialized: tx=%d, rx=%d, baud=%d",
             ray2::driver::kUart0TxPin,
             ray2::driver::kUart0RxPin,
             ray2::driver::kUart0BaudRate);
    ESP_LOGI(tag, "Hello from ray2-idf");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(tag, "Hello from ray2-idf");
    }
}
