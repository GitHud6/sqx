#include "step_timer_driver.hpp"

#include <atomic>

#include "driver/gptimer.h"

namespace ray2::driver {
namespace {

constexpr std::uint32_t kTimerResolutionHz = 1000000;
constexpr int kTimerInterruptPriority = 3;

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_running = false;
gptimer_handle_t s_timer = nullptr;
StepTimerAlarmCallback s_alarm_callback = nullptr;
void* s_alarm_user_data = nullptr;

bool IRAM_ATTR timer_alarm_callback(gptimer_handle_t,
                                    const gptimer_alarm_event_data_t*,
                                    void*) {
    if (!s_running.load() || s_alarm_callback == nullptr) {
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

    const gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = kTimerResolutionHz,
        .intr_priority = kTimerInterruptPriority,
        .flags = {},
    };
    esp_err_t result = gptimer_new_timer(&config, &s_timer);
    if (result != ESP_OK) {
        return result;
    }

    s_alarm_callback = callback;
    s_alarm_user_data = user_data;
    const gptimer_event_callbacks_t callbacks = {.on_alarm = timer_alarm_callback};
    result = gptimer_register_event_callbacks(s_timer, &callbacks, nullptr);
    if (result != ESP_OK) {
        gptimer_del_timer(s_timer);
        s_timer = nullptr;
        s_alarm_callback = nullptr;
        s_alarm_user_data = nullptr;
        return result;
    }
    result = gptimer_enable(s_timer);
    if (result != ESP_OK) {
        gptimer_del_timer(s_timer);
        s_timer = nullptr;
        s_alarm_callback = nullptr;
        s_alarm_user_data = nullptr;
        return result;
    }

    s_initialized.store(true);
    return ESP_OK;
}

esp_err_t step_timer_driver_start(const std::uint32_t interval_us) {
    if (!s_initialized.load() || interval_us == 0 || s_running.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    const gptimer_alarm_config_t alarm = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    esp_err_t result = gptimer_set_raw_count(s_timer, 0);
    if (result == ESP_OK) {
        result = gptimer_set_alarm_action(s_timer, &alarm);
    }
    if (result == ESP_OK) {
        s_running.store(true);
        result = gptimer_start(s_timer);
        if (result != ESP_OK) {
            s_running.store(false);
        }
    }
    return result;
}

esp_err_t step_timer_driver_stop() {
    if (!s_initialized.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running.store(false);
    return gptimer_stop(s_timer);
}

bool step_timer_driver_is_initialized() {
    return s_initialized.load();
}

bool step_timer_driver_is_running() {
    return s_running.load();
}

void step_timer_driver_stop_from_isr() {
    s_running.store(false);
    if (s_timer != nullptr) {
        gptimer_stop(s_timer);
    }
}

void step_timer_driver_update_interval_from_isr(const std::uint32_t interval_us) {
    if (!s_running.load() || s_timer == nullptr || interval_us == 0) {
        return;
    }
    const gptimer_alarm_config_t alarm = {
        .alarm_count = interval_us,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    gptimer_set_alarm_action(s_timer, &alarm);
}

}  // namespace ray2::driver
