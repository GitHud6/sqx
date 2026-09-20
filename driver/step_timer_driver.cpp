#include "step_timer_driver.hpp"

#include <atomic>

// The Timer Group backend is deliberately selected for deterministic
// auto-reload behavior. Suppress the generic migration warning locally.
#ifndef CONFIG_GPTIMER_SUPPRESS_DEPRECATE_WARN
#define CONFIG_GPTIMER_SUPPRESS_DEPRECATE_WARN 1
#endif
#include "driver/timer.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"

namespace laserctl::driver {
namespace {

constexpr timer_group_t kStepTimerGroup = TIMER_GROUP_0;
constexpr timer_idx_t kStepTimerIndex = TIMER_0;
constexpr std::uint32_t kTimerResolutionHz = 20000000;
constexpr std::uint32_t kTimerSourceFrequencyHz = 40000000;
constexpr std::uint32_t kTimerDivider = kTimerSourceFrequencyHz / kTimerResolutionHz;
constexpr int kTimerInterruptFlags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3;

static_assert(kTimerDivider >= 2, "Timer Group divider must be at least two");

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_running = false;
std::atomic<int> s_init_core = -1;
std::atomic<int> s_isr_core = -1;
StepTimerAlarmCallback s_alarm_callback = nullptr;
void* s_alarm_user_data = nullptr;

bool IRAM_ATTR timer_alarm_callback(void*) {
    s_isr_core.store(xPortGetCoreID(), std::memory_order_relaxed);
    if (!s_running.load(std::memory_order_relaxed) || s_alarm_callback == nullptr) {
        return false;
    }
    return s_alarm_callback(s_alarm_user_data);
}

}  // namespace

esp_err_t step_timer_driver_init(const StepTimerAlarmCallback callback, void* const user_data) {
    if (callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized.load()) {
        return callback == s_alarm_callback && user_data == s_alarm_user_data ? ESP_OK
                                                                               : ESP_ERR_INVALID_STATE;
    }

    const timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .clk_src = TIMER_SRC_CLK_XTAL,
        .divider = kTimerDivider,
    };
    esp_err_t result = timer_init(kStepTimerGroup, kStepTimerIndex, &config);
    if (result != ESP_OK) {
        return result;
    }
    result = timer_set_counter_value(kStepTimerGroup, kStepTimerIndex, 0);
    if (result == ESP_OK) {
        result = timer_enable_intr(kStepTimerGroup, kStepTimerIndex);
    }
    if (result == ESP_OK) {
        s_alarm_callback = callback;
        s_alarm_user_data = user_data;
        result = timer_isr_callback_add(kStepTimerGroup, kStepTimerIndex,
                                        timer_alarm_callback, nullptr,
                                        kTimerInterruptFlags);
    }
    if (result != ESP_OK) {
        s_alarm_callback = nullptr;
        s_alarm_user_data = nullptr;
        timer_deinit(kStepTimerGroup, kStepTimerIndex);
        return result;
    }

    s_initialized.store(true, std::memory_order_release);
    s_init_core.store(xPortGetCoreID(), std::memory_order_release);
    return ESP_OK;
}

esp_err_t step_timer_driver_start(const std::uint32_t interval_ticks) {
    if (!s_initialized.load() || interval_ticks == 0 || s_running.load()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = timer_set_counter_value(kStepTimerGroup, kStepTimerIndex, 0);
    if (result == ESP_OK) {
        result = timer_set_alarm_value(kStepTimerGroup, kStepTimerIndex, interval_ticks);
    }
    if (result == ESP_OK) {
        s_running.store(true, std::memory_order_release);
        result = timer_start(kStepTimerGroup, kStepTimerIndex);
        if (result != ESP_OK) {
            s_running.store(false, std::memory_order_release);
        }
    }
    return result;
}

esp_err_t step_timer_driver_stop() {
    if (!s_initialized.load()) {
        return ESP_ERR_INVALID_STATE;
    }
    s_running.store(false, std::memory_order_release);
    return timer_pause(kStepTimerGroup, kStepTimerIndex);
}

bool step_timer_driver_is_initialized() {
    return s_initialized.load(std::memory_order_acquire);
}

bool step_timer_driver_is_running() {
    return s_running.load(std::memory_order_acquire);
}

int step_timer_driver_init_core() {
    return s_init_core.load(std::memory_order_acquire);
}

int step_timer_driver_isr_core() {
    return s_isr_core.load(std::memory_order_acquire);
}

void IRAM_ATTR step_timer_driver_stop_from_isr() {
    s_running.store(false, std::memory_order_relaxed);
    timer_group_set_counter_enable_in_isr(kStepTimerGroup, kStepTimerIndex, TIMER_PAUSE);
}

void IRAM_ATTR step_timer_driver_update_interval_from_isr(const std::uint32_t interval_ticks) {
    if (!s_running.load(std::memory_order_relaxed) || interval_ticks == 0) {
        return;
    }
    // Timer Group hardware reloads immediately at the alarm edge. Updating
    // the threshold here changes the following period without stopping the
    // counter or reconfiguring the interrupt service.
    timer_group_set_alarm_value_in_isr(kStepTimerGroup, kStepTimerIndex, interval_ticks);
}

std::uint64_t IRAM_ATTR step_timer_driver_lateness_ticks_from_isr() {
    return timer_group_get_counter_value_in_isr(kStepTimerGroup, kStepTimerIndex);
}

}  // namespace laserctl::driver
