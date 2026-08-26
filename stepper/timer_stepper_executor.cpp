#include "timer_stepper_executor.hpp"

#include <algorithm>
#include <atomic>

#include "rmt_step_pulse_driver.hpp"
#include "step_timer_driver.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "laser_driver.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"

namespace ray2::stepper {
namespace {

constexpr char kLogTag[] = "timer_stepper";

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_busy = false;
std::atomic<bool> s_feed_hold_requested = false;
std::atomic<std::uint64_t> s_timer_interrupts = 0;
portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

struct MotionState {
    std::int32_t axis_steps[machine::kAxisCount] = {};
    bool positive_direction[machine::kAxisCount] = {};
    std::int32_t execution_ticks = 0;
    std::int32_t completed_ticks = 0;
    std::int64_t axis_error[machine::kAxisCount] = {};
    std::uint32_t effective_step_interval_us = 0;
    std::uint64_t execution_id = 0;
    std::uint32_t start_speed_tenths_mm_per_min = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    bool completes_block = false;
    bool feed_hold_segment = false;
    bool completes_feed_hold = false;
    bool system_motion = false;
    bool axis_stopped[machine::kAxisCount] = {};
    bool enabled[machine::kAxisCount] = {};
};

MotionState s_motion = {};
StepSegment s_feed_hold_boundary = {};
std::atomic<bool> s_feed_hold_boundary_valid = false;

MotionState motion_from_segment(const StepSegment& segment) {
    MotionState motion = {};
    motion.axis_steps[0] = segment.axis_steps[0];
    motion.axis_steps[1] = segment.axis_steps[1];
    motion.positive_direction[0] = segment.positive_direction[0];
    motion.positive_direction[1] = segment.positive_direction[1];
    const std::uint32_t amass_scale = 1U << segment.amass_level;
    motion.execution_ticks = segment.major_steps * static_cast<std::int32_t>(amass_scale);
    motion.effective_step_interval_us = std::max<std::uint32_t>(
        4U, segment.step_interval_us / amass_scale);
    motion.execution_id = segment.execution_id;
    motion.start_speed_tenths_mm_per_min = segment.start_speed_tenths_mm_per_min;
    motion.end_speed_tenths_mm_per_min = segment.end_speed_tenths_mm_per_min;
    motion.completes_block = segment.completes_block;
    motion.feed_hold_segment = segment.feed_hold_segment;
    motion.completes_feed_hold = segment.completes_feed_hold;
    motion.system_motion = segment.system_motion;
    return motion;
}

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

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        s_motion.axis_error[axis] += s_motion.axis_steps[axis];
        if (s_motion.axis_error[axis] >= s_motion.execution_ticks) {
            if (!s_motion.axis_stopped[axis]) {
                const auto current_axis = static_cast<machine::Axis>(axis);
                trigger_pulse(axis);
                stepper_record_executed_steps_from_isr(current_axis, 1);
            }
            s_motion.axis_error[axis] -= s_motion.execution_ticks;
        }
    }

    ++s_motion.completed_ticks;
    s_timer_interrupts.fetch_add(1);
    const bool finished = s_motion.completed_ticks == s_motion.execution_ticks;
    if (finished) {
        const MotionState completed_motion = s_motion;
        if (completed_motion.completes_block) {
            const SegmentCompletion completion = {
                .execution_id = completed_motion.execution_id,
                .status = SegmentCompletionStatus::Completed,
            };
            stepper_completion_publish_from_isr(completion, &task_woken);
        }
        StepSegment next = {};
        if (completed_motion.system_motion) {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
        } else if (completed_motion.completes_feed_hold) {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            stepper_segment_request_refill_from_isr(&task_woken);
        } else if (completed_motion.feed_hold_segment &&
                   s_feed_hold_requested.load(std::memory_order_relaxed) &&
                   stepper_feed_hold_segment_consume(next)) {
            for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                stepper_set_direction_from_isr(static_cast<machine::Axis>(axis),
                                                next.positive_direction[axis] ? Direction::Positive : Direction::Negative);
            }
            s_motion = motion_from_segment(next);
            laser::laser_disable_from_isr();
            stepper_segment_set_active(next);
            driver::step_timer_driver_update_interval_from_isr(s_motion.effective_step_interval_us);
        } else if (s_feed_hold_requested.load(std::memory_order_relaxed)) {
            s_feed_hold_boundary = {};
            s_feed_hold_boundary.execution_id = completed_motion.execution_id;
            s_feed_hold_boundary.axis_steps[0] = completed_motion.axis_steps[0];
            s_feed_hold_boundary.axis_steps[1] = completed_motion.axis_steps[1];
            s_feed_hold_boundary.major_steps = completed_motion.execution_ticks;
            s_feed_hold_boundary.step_interval_us = completed_motion.effective_step_interval_us;
            s_feed_hold_boundary.start_speed_tenths_mm_per_min =
                completed_motion.start_speed_tenths_mm_per_min;
            s_feed_hold_boundary.end_speed_tenths_mm_per_min =
                completed_motion.end_speed_tenths_mm_per_min;
            s_feed_hold_boundary.start_speed_mm_per_min =
                static_cast<double>(completed_motion.start_speed_tenths_mm_per_min) / 10.0;
            s_feed_hold_boundary.end_speed_mm_per_min =
                static_cast<double>(completed_motion.end_speed_tenths_mm_per_min) / 10.0;
            for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                s_feed_hold_boundary.positive_direction[axis] =
                    completed_motion.positive_direction[axis];
            }
            s_feed_hold_boundary_valid.store(true, std::memory_order_release);
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            stepper_segment_request_refill_from_isr(&task_woken);
        } else if (stepper_segment_consume(next)) {
            for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                stepper_set_direction_from_isr(static_cast<machine::Axis>(axis),
                                                next.positive_direction[axis] ? Direction::Positive : Direction::Negative);
            }
            s_motion = motion_from_segment(next);
            laser::laser_apply_segment_power_from_isr(next.laser_power_per_mille,
                                                       next.execution_id);
            stepper_segment_set_active(next);
            driver::step_timer_driver_update_interval_from_isr(s_motion.effective_step_interval_us);
            if (stepper_segment_needs_refill_from_isr()) {
                stepper_segment_request_refill_from_isr(&task_woken);
            }
        } else {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            if (!completed_motion.completes_block) {
                stepper_segment_record_underflow_from_isr();
                const SegmentCompletion fault = {
                    .execution_id = completed_motion.execution_id,
                    .status = SegmentCompletionStatus::Faulted,
                };
                stepper_completion_publish_from_isr(fault, &task_woken);
            }
        }
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
    if (!s_initialized.load() || segment.major_steps <= 0 || segment.step_interval_us <= 3 ||
        segment.amass_level > kAmassMaximumLevel) {
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
    MotionState motion = motion_from_segment(segment);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
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
    laser::laser_apply_segment_power_from_isr(segment.laser_power_per_mille,
                                               segment.execution_id);
    stepper_segment_set_active(segment);
    const esp_err_t result = driver::step_timer_driver_start(motion.effective_step_interval_us);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "TIMER-START-REJECT error=%s steps=(%ld,%ld) interval=%lu",
                 esp_err_to_name(result), static_cast<long>(segment.axis_steps[0]),
                 static_cast<long>(segment.axis_steps[1]),
                 static_cast<unsigned long>(segment.step_interval_us));
        timer_stepper_executor_stop(); return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t timer_stepper_executor_start_from_segment_ring() {
    StepSegment segment = {};
    if (!stepper_segment_consume(segment)) {
        return ESP_ERR_NOT_FOUND;
    }
    return timer_stepper_executor_start_segment(segment);
}

esp_err_t timer_stepper_executor_start_feed_hold_segment() {
    StepSegment segment = {};
    if (!stepper_feed_hold_segment_consume(segment)) {
        return ESP_ERR_NOT_FOUND;
    }
    return timer_stepper_executor_start_segment(segment);
}

bool timer_stepper_executor_is_busy() { return s_busy.load(); }

void timer_stepper_executor_request_feed_hold() {
    s_feed_hold_boundary_valid.store(false, std::memory_order_release);
    s_feed_hold_requested.store(true, std::memory_order_release);
}

bool timer_stepper_executor_take_feed_hold_boundary(StepSegment& segment) {
    if (!s_feed_hold_boundary_valid.exchange(false, std::memory_order_acq_rel)) {
        return false;
    }
    portENTER_CRITICAL(&s_motion_lock);
    segment = s_feed_hold_boundary;
    portEXIT_CRITICAL(&s_motion_lock);
    return true;
}

void timer_stepper_executor_resume() {
    s_feed_hold_requested.store(false, std::memory_order_release);
    s_feed_hold_boundary_valid.store(false, std::memory_order_release);
    stepper_feed_hold_segments_reset();
}

void timer_stepper_executor_stop() {
    if (driver::step_timer_driver_is_initialized()) driver::step_timer_driver_stop();
    driver::rmt_step_pulse_driver_stop();
    laser::laser_disable();
    stepper_segment_clear_active_from_isr();
    MotionState motion = {};
    portENTER_CRITICAL(&s_motion_lock);
    motion = s_motion; s_motion = {}; s_busy.store(false);
    portEXIT_CRITICAL(&s_motion_lock);
    disable_active_axes(motion);
    s_feed_hold_requested.store(false, std::memory_order_release);
    stepper_feed_hold_segments_reset();
}

void IRAM_ATTR timer_stepper_executor_stop_from_isr() {
    if (!s_initialized.load(std::memory_order_relaxed)) return;
    driver::step_timer_driver_stop_from_isr();
    driver::rmt_step_pulse_driver_stop_from_isr();
    laser::laser_disable_from_isr();
    stepper_segment_clear_active_from_isr();
    portENTER_CRITICAL_ISR(&s_motion_lock);
    s_busy.store(false, std::memory_order_relaxed);
    portEXIT_CRITICAL_ISR(&s_motion_lock);
}

bool IRAM_ATTR timer_stepper_executor_stop_axis_from_isr(const std::size_t axis_index) {
    if (axis_index >= machine::kAxisCount || !s_initialized.load(std::memory_order_relaxed)) return false;
    portENTER_CRITICAL_ISR(&s_motion_lock);
    if (!s_busy.load(std::memory_order_relaxed) || !s_motion.system_motion) {
        portEXIT_CRITICAL_ISR(&s_motion_lock);
        return false;
    }
    s_motion.axis_stopped[axis_index] = true;
    driver::rmt_step_pulse_driver_stop_axis_from_isr(axis_index);
    bool all_stopped = true;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (s_motion.axis_steps[axis] > 0 && !s_motion.axis_stopped[axis]) all_stopped = false;
    }
    if (all_stopped) {
        driver::step_timer_driver_stop_from_isr();
        laser::laser_disable_from_isr();
        stepper_segment_clear_active_from_isr();
        s_busy.store(false, std::memory_order_relaxed);
    }
    portEXIT_CRITICAL_ISR(&s_motion_lock);
    return all_stopped;
}

TimerStepperDiagnostics timer_stepper_executor_diagnostics() {
    TimerStepperDiagnostics diagnostics = {};
    diagnostics.initialized = s_initialized.load(); diagnostics.owns_step_pins = diagnostics.initialized;
    diagnostics.timer_running = s_busy.load(); diagnostics.timer_interrupts = s_timer_interrupts.load();
    return diagnostics;
}

}  // namespace ray2::stepper
