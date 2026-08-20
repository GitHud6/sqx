#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void) {
    static const char* tag = "ray2-idf";

    ESP_LOGI(tag, "Hello from ray2-idf");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(tag, "Hello from ray2-idf");
    }
}
