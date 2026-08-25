#include "timer_stepper_executor.hpp"

#include <algorithm>
#include <atomic>

#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/rmt_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"

#include "stepper_driver.hpp"

namespace ray2::stepper {
namespace {

constexpr std::int32_t kMaximumMajorSteps = 256;
constexpr std::uint32_t kRmtClockDivider = 80;
constexpr std::uint32_t kRmtPulseSymbol = 3U | (1U << 15) | (1U << 16);
constexpr char kLogTag[] = "timer_stepper";

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_busy = false;
std::atomic<std::uint64_t> s_timer_interrupts = 0;
SemaphoreHandle_t s_completion_semaphore = nullptr;
gptimer_handle_t s_timer = nullptr;
portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

struct MotionState {
    std::int32_t axis_steps[machine::kAxisCount] = {};
    std::int32_t major_steps = 0;
    std::int32_t minor_steps = 0;
    std::int32_t completed_steps = 0;
    std::int64_t error = 0;
    bool x_is_major = true;
    bool enabled[machine::kAxisCount] = {};
};

MotionState s_motion = {};

void IRAM_ATTR trigger_pulse(const std::size_t axis) {
    rmt_ll_tx_reset_pointer(&RMT, axis);
    rmt_ll_tx_start(&RMT, axis);
}

bool IRAM_ATTR timer_alarm_callback(gptimer_handle_t timer,
                                    const gptimer_alarm_event_data_t*,
                                    void*) {
    BaseType_t task_woken = pdFALSE;
    portENTER_CRITICAL_ISR(&s_motion_lock);
    if (!s_busy.load()) {
        portEXIT_CRITICAL_ISR(&s_motion_lock);
        return false;
    }

    const machine::Axis major_axis = s_motion.x_is_major ? machine::Axis::X : machine::Axis::Y;
    const machine::Axis minor_axis = s_motion.x_is_major ? machine::Axis::Y : machine::Axis::X;
    trigger_pulse(static_cast<std::size_t>(major_axis));
    stepper_record_executed_steps_from_isr(major_axis, 1);
    s_motion.error += s_motion.minor_steps;
    if (s_motion.error * 2 >= s_motion.major_steps) {
        trigger_pulse(static_cast<std::size_t>(minor_axis));
        stepper_record_executed_steps_from_isr(minor_axis, 1);
        s_motion.error -= s_motion.major_steps;
    }

    ++s_motion.completed_steps;
    s_timer_interrupts.fetch_add(1);
    const bool finished = s_motion.completed_steps == s_motion.major_steps;
    if (finished) {
        s_busy.store(false);
        gptimer_stop(timer);
    }
    portEXIT_CRITICAL_ISR(&s_motion_lock);
    if (finished && s_completion_semaphore != nullptr) {
        xSemaphoreGiveFromISR(s_completion_semaphore, &task_woken);
    }
    return task_woken == pdTRUE;
}

esp_err_t configure_rmt_axis(const std::size_t axis_index) {
    const auto& axis = machine::raycube_axis(static_cast<machine::Axis>(axis_index));
    rmt_ll_enable_group_clock(&RMT, true);
    rmt_ll_tx_stop(&RMT, axis_index);
    rmt_ll_tx_set_channel_clock_div(&RMT, axis_index, kRmtClockDivider);
    rmt_ll_tx_set_mem_blocks(&RMT, axis_index, 1);
    rmt_ll_tx_enable_wrap(&RMT, axis_index, false);
    rmt_ll_tx_enable_loop(&RMT, axis_index, false);
    rmt_ll_tx_fix_idle_level(&RMT, axis_index, 0, true);
    rmt_ll_tx_enable_carrier_modulation(&RMT, axis_index, false);
    rmt_ll_tx_reset_pointer(&RMT, axis_index);
    RMT.chndata[axis_index].chndata = kRmtPulseSymbol;
    RMT.chndata[axis_index].chndata = 0;
    esp_rom_gpio_connect_out_signal(axis.step_pin, RMT_SIG_OUT0_IDX + axis_index, false, false);
    return ESP_OK;
}

void disable_active_axes(const MotionState& motion) {
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (motion.enabled[axis]) stepper_set_enabled(static_cast<machine::Axis>(axis), false);
    }
}

}  // namespace

esp_err_t timer_stepper_executor_init() {
    if (s_initialized.load()) return ESP_OK;
    if (stepper_init() != ESP_OK) return ESP_FAIL;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const esp_err_t result = configure_rmt_axis(axis);
        if (result != ESP_OK) return result;
    }
    const gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, .intr_priority = 3, .flags = {},
    };
    esp_err_t result = gptimer_new_timer(&config, &s_timer);
    if (result != ESP_OK) return result;
    const gptimer_event_callbacks_t callbacks = {.on_alarm = timer_alarm_callback};
    result = gptimer_register_event_callbacks(s_timer, &callbacks, nullptr);
    if (result != ESP_OK) return result;
    result = gptimer_enable(s_timer);
    if (result != ESP_OK) return result;
    s_completion_semaphore = xSemaphoreCreateBinary();
    if (s_completion_semaphore == nullptr) return ESP_ERR_NO_MEM;
    s_initialized.store(true);
    return ESP_OK;
}

esp_err_t timer_stepper_executor_execute(const std::int32_t axis_steps[], const bool positive_direction[],
                                         std::uint32_t interval_us) {
    if (!s_initialized.load() || axis_steps == nullptr || positive_direction == nullptr || interval_us <= 3) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=invalid-request initialized=%d interval=%lu",
                 s_initialized.load() ? 1 : 0, static_cast<unsigned long>(interval_us));
        return ESP_ERR_INVALID_ARG;
    }
    const std::int32_t major = std::max(axis_steps[0], axis_steps[1]);
    if (s_busy.load() || major <= 0 || major > kMaximumMajorSteps) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=busy-or-range busy=%d steps=(%ld,%ld) maximum=%ld",
                 s_busy.load() ? 1 : 0, static_cast<long>(axis_steps[0]),
                 static_cast<long>(axis_steps[1]), static_cast<long>(kMaximumMajorSteps));
        return ESP_ERR_NOT_SUPPORTED;
    }
    MotionState motion = {};
    motion.axis_steps[0] = axis_steps[0]; motion.axis_steps[1] = axis_steps[1];
    motion.major_steps = major; motion.minor_steps = std::min(axis_steps[0], axis_steps[1]);
    motion.x_is_major = axis_steps[0] >= axis_steps[1];
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (axis_steps[axis] == 0) continue;
        const auto current = static_cast<machine::Axis>(axis);
        const esp_err_t direction_result =
            stepper_set_direction(current, positive_direction[axis] ? Direction::Positive : Direction::Negative);
        const esp_err_t enable_result = direction_result == ESP_OK
                                            ? stepper_set_enabled(current, true)
                                            : direction_result;
        if (direction_result != ESP_OK || enable_result != ESP_OK) {
            ESP_LOGE(kLogTag, "TIMER-REJECT reason=axis-config axis=%u direction=%s enable=%s",
                     static_cast<unsigned>(axis), esp_err_to_name(direction_result),
                     esp_err_to_name(enable_result));
            disable_active_axes(motion);
            return ESP_FAIL;
        }
        motion.enabled[axis] = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    xSemaphoreTake(s_completion_semaphore, 0);
    portENTER_CRITICAL(&s_motion_lock);
    s_motion = motion; s_busy.store(true);
    portEXIT_CRITICAL(&s_motion_lock);
    const gptimer_alarm_config_t alarm = {.alarm_count = interval_us, .reload_count = 0, .flags = {.auto_reload_on_alarm = true}};
    esp_err_t result = gptimer_set_raw_count(s_timer, 0);
    if (result == ESP_OK) result = gptimer_set_alarm_action(s_timer, &alarm);
    if (result == ESP_OK) result = gptimer_start(s_timer);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "TIMER-START-REJECT error=%s steps=(%ld,%ld) interval=%lu",
                 esp_err_to_name(result), static_cast<long>(axis_steps[0]),
                 static_cast<long>(axis_steps[1]), static_cast<unsigned long>(interval_us));
        timer_stepper_executor_stop(); return ESP_FAIL;
    }
    xSemaphoreTake(s_completion_semaphore, portMAX_DELAY);
    disable_active_axes(motion);
    return ESP_OK;
}

bool timer_stepper_executor_is_busy() { return s_busy.load(); }

void timer_stepper_executor_stop() {
    if (s_timer != nullptr) gptimer_stop(s_timer);
    MotionState motion = {};
    portENTER_CRITICAL(&s_motion_lock);
    motion = s_motion; s_motion = {}; s_busy.store(false);
    portEXIT_CRITICAL(&s_motion_lock);
    disable_active_axes(motion);
}

TimerStepperDiagnostics timer_stepper_executor_diagnostics() {
    TimerStepperDiagnostics diagnostics = {};
    diagnostics.initialized = s_initialized.load(); diagnostics.owns_step_pins = diagnostics.initialized;
    diagnostics.timer_running = s_busy.load(); diagnostics.timer_interrupts = s_timer_interrupts.load();
    return diagnostics;
}

}  // namespace ray2::stepper
