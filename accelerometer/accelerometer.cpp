#include "accelerometer.hpp"

#include <atomic>
#include <cstdint>
#include <cstdlib>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lis2dh12_driver.hpp"
#include "planner.hpp"
#include "runtime_affinity.hpp"
#include "service.hpp"

namespace laserctl::accelerometer {
namespace {

constexpr char kLogTag[] = "accelerometer";
constexpr TickType_t kCheckPeriod =
    pdMS_TO_TICKS(150) == 0 ? static_cast<TickType_t>(1) : pdMS_TO_TICKS(150);
constexpr TickType_t kConfirmDelay =
    pdMS_TO_TICKS(15) == 0 ? static_cast<TickType_t>(1) : pdMS_TO_TICKS(15);
// Production validation established 500 raw counts as the normal-motion-safe threshold.
constexpr int kAxisThreshold = 500;
constexpr std::uint32_t kConfirmSamples = 10;

TaskHandle_t s_task = nullptr;
std::atomic<bool> s_initialized = false;

void sample_task(void*) {
    TickType_t next_wake = xTaskGetTickCount();
    laserctl::driver::Lis2dh12RawSample latest = {};

    while (true) {
        std::uint32_t abnormal_samples = 0;
        if (!laserctl::planner::planner_is_idle()) {
            for (std::uint32_t index = 0; index < kConfirmSamples; ++index) {
                if (!laserctl::driver::lis2dh12_read_raw(latest)) break;

                const bool abnormal = std::abs(static_cast<int>(latest.raw_x)) > kAxisThreshold ||
                                      std::abs(static_cast<int>(latest.raw_y)) > kAxisThreshold;
                if (!abnormal) break;
                ++abnormal_samples;
                vTaskDelay(kConfirmDelay);
            }
        }

        if (abnormal_samples == kConfirmSamples) {
            laserctl::service::service_request_safety_abort(
                laserctl::service::FaultReason::MotionAlarm);
            ESP_LOGE(kLogTag, "motion alarm: XY acceleration threshold exceeded");
        }
        vTaskDelayUntil(&next_wake, kCheckPeriod);
    }
}

}  // namespace

esp_err_t init() {
    if (s_initialized.load(std::memory_order_acquire)) return ESP_OK;
    if (!laserctl::driver::lis2dh12_is_initialized()) return ESP_ERR_INVALID_STATE;

    if (xTaskCreatePinnedToCore(sample_task, "accelerometer", 3072, nullptr, 3, &s_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        s_task = nullptr;
        return ESP_ERR_NO_MEM;
    }
    s_initialized.store(true, std::memory_order_release);
    ESP_LOGI(kLogTag, "sampling task started");
    return ESP_OK;
}

bool is_initialized() {
    return s_initialized.load(std::memory_order_acquire);
}

}  // namespace laserctl::accelerometer
