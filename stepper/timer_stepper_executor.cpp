#include "timer_stepper_executor.hpp"

#include <algorithm>
#include <atomic>

#include "rmt_step_pulse_driver.hpp"
#include "step_timer_driver.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "stepper_driver.hpp"
#include "stepper_prep.hpp"

namespace ray2::stepper {
namespace {

constexpr char kLogTag[] = "timer_stepper";

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_busy = false;
std::atomic<std::uint64_t> s_timer_interrupts = 0;
portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

struct MotionState {
    std::int32_t axis_steps[machine::kAxisCount] = {};
    std::int32_t major_steps = 0;
    std::int32_t minor_steps = 0;
    std::int32_t completed_steps = 0;
    std::int64_t error = 0;
    std::uint64_t execution_id = 0;
    bool x_is_major = true;
    bool enabled[machine::kAxisCount] = {};
};

MotionState s_motion = {};

void IRAM_ATTR trigger_pulse(const std::size_t axis) {
    driver::rmt_step_pulse_driver_trigger(axis);
}

bool IRAM_ATTR timer_alarm_callback(void*) {
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
        driver::step_timer_driver_stop_from_isr();
        stepper_segment_clear_active_from_isr();
        const SegmentCompletion completion = {
            .execution_id = s_motion.execution_id,
            .status = SegmentCompletionStatus::Completed,
        };
        stepper_completion_publish_from_isr(completion, &task_woken);
    }
    portEXIT_CRITICAL_ISR(&s_motion_lock);
    return task_woken == pdTRUE;
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
    const esp_err_t pulse_driver_result = driver::rmt_step_pulse_driver_init();
    if (pulse_driver_result != ESP_OK) return pulse_driver_result;
    const esp_err_t result = driver::step_timer_driver_init(timer_alarm_callback, nullptr);
    if (result != ESP_OK) return result;
    s_initialized.store(true);
    return ESP_OK;
}

esp_err_t timer_stepper_executor_start_segment(const StepSegment& segment) {
    if (!s_initialized.load() || segment.major_steps <= 0 || segment.step_interval_us <= 3) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=invalid-request initialized=%d interval=%lu",
                 s_initialized.load() ? 1 : 0,
                 static_cast<unsigned long>(segment.step_interval_us));
        return ESP_ERR_INVALID_ARG;
    }
    const std::int32_t major = std::max(segment.axis_steps[0], segment.axis_steps[1]);
    if (s_busy.load() || major != segment.major_steps) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=busy-or-segment-mismatch busy=%d steps=(%ld,%ld) major=%ld",
                 s_busy.load() ? 1 : 0, static_cast<long>(segment.axis_steps[0]),
                 static_cast<long>(segment.axis_steps[1]), static_cast<long>(segment.major_steps));
        return ESP_ERR_NOT_SUPPORTED;
    }
    MotionState motion = {};
    motion.axis_steps[0] = segment.axis_steps[0]; motion.axis_steps[1] = segment.axis_steps[1];
    motion.major_steps = major;
    motion.minor_steps = std::min(segment.axis_steps[0], segment.axis_steps[1]);
    motion.x_is_major = segment.axis_steps[0] >= segment.axis_steps[1];
    motion.execution_id = segment.execution_id;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (segment.axis_steps[axis] == 0) continue;
        const auto current = static_cast<machine::Axis>(axis);
        const esp_err_t direction_result =
            stepper_set_direction(current, segment.positive_direction[axis] ? Direction::Positive : Direction::Negative);
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
    portENTER_CRITICAL(&s_motion_lock);
    s_motion = motion; s_busy.store(true);
    portEXIT_CRITICAL(&s_motion_lock);
    stepper_segment_set_active(segment.execution_id);
    const esp_err_t result = driver::step_timer_driver_start(segment.step_interval_us);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "TIMER-START-REJECT error=%s steps=(%ld,%ld) interval=%lu",
                 esp_err_to_name(result), static_cast<long>(segment.axis_steps[0]),
                 static_cast<long>(segment.axis_steps[1]),
                 static_cast<unsigned long>(segment.step_interval_us));
        timer_stepper_executor_stop(); return ESP_FAIL;
    }
    return ESP_OK;
}

bool timer_stepper_executor_is_busy() { return s_busy.load(); }

void timer_stepper_executor_stop() {
    if (driver::step_timer_driver_is_initialized()) driver::step_timer_driver_stop();
    driver::rmt_step_pulse_driver_stop();
    stepper_segment_clear_active_from_isr();
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
