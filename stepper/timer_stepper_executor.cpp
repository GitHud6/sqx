#include "timer_stepper_executor.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>

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
// These counters are sampled once per second for diagnostics. 32-bit values
// are sufficient for that window and avoid expensive 64-bit atomics in ISR.
std::atomic<std::uint32_t> s_scheduled_timer_ticks = 0;
std::atomic<std::uint32_t> s_scheduled_timer_events = 0;
std::atomic<std::uint32_t> s_segment_handoffs = 0;
std::atomic<std::uint32_t> s_block_phase_continuations = 0;
std::atomic<std::uint32_t> s_block_phase_initializations = 0;
std::atomic<std::uint32_t> s_pulse_callbacks = 0;
std::atomic<std::uint32_t> s_rmt_triggers[machine::kAxisCount] = {};
std::atomic<std::uint32_t> s_direction_writes = 0;
portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

struct MotionState {
    std::int32_t axis_steps[machine::kAxisCount] = {};
    std::int32_t emitted_axis_steps[machine::kAxisCount] = {};
    std::int32_t execution_ticks = 0;
    std::int32_t completed_ticks = 0;
    std::int64_t axis_error[machine::kAxisCount] = {};
    std::int64_t axis_increment[machine::kAxisCount] = {};
    std::int64_t phase_denominator = 0;
    std::uint32_t effective_step_interval_ticks = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    std::uint64_t execution_id = 0;
    bool completes_block = false;
    bool feed_hold = false;
    bool ends_feed_hold = false;
    bool system_motion = false;
    bool uses_block_phase = false;
    bool axis_stopped[machine::kAxisCount] = {};
    bool enabled[machine::kAxisCount] = {};
    std::uint8_t prepared_step_mask = 0;
    bool prepared_step_mask_valid = false;
};

MotionState s_motion = {};
bool s_applied_direction[machine::kAxisCount] = {};
bool s_applied_direction_valid = false;

void IRAM_ATTR record_segment_schedule(const MotionState& motion) {
    s_scheduled_timer_ticks.fetch_add(
        static_cast<std::uint32_t>(motion.effective_step_interval_ticks) *
            static_cast<std::uint32_t>(motion.execution_ticks),
        std::memory_order_relaxed);
    s_scheduled_timer_events.fetch_add(static_cast<std::uint32_t>(motion.execution_ticks),
                                       std::memory_order_relaxed);
}

std::uint32_t segment_timer_ticks(const StepSegment& segment) {
    if (segment.step_interval_ticks != 0) return segment.step_interval_ticks;
    if (segment.step_interval_us == 0) return 0;
    return segment.step_interval_us * kStepTimerTicksPerMicrosecond;
}

MotionState motion_from_segment(const StepSegment& segment) {
    MotionState motion = {};
    motion.axis_steps[0] = segment.axis_steps[0];
    motion.axis_steps[1] = segment.axis_steps[1];
    const std::uint32_t amass_scale = 1U << segment.amass_level;
    motion.execution_ticks = segment.major_steps * static_cast<std::int32_t>(amass_scale);
    motion.effective_step_interval_ticks = std::max<std::uint32_t>(
        1U, segment_timer_ticks(segment) / amass_scale);
    motion.end_speed_tenths_mm_per_min = segment.end_speed_tenths_mm_per_min;
    motion.execution_id = segment.execution_id;
    motion.completes_block = segment.completes_block;
    motion.feed_hold = segment.feed_hold;
    motion.ends_feed_hold = segment.ends_feed_hold;
    motion.system_motion = segment.system_motion;
    if (segment.block_major_steps > 0 &&
        std::max(segment.block_axis_steps[0], segment.block_axis_steps[1]) ==
            segment.block_major_steps) {
        motion.uses_block_phase = true;
        motion.phase_denominator =
            static_cast<std::int64_t>(segment.block_major_steps) << kAmassMaximumLevel;
        for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
            motion.axis_increment[axis] = static_cast<std::int64_t>(segment.block_axis_steps[axis])
                                          << (kAmassMaximumLevel - segment.amass_level);
        }
    } else {
        motion.phase_denominator = motion.execution_ticks;
        for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
            motion.axis_increment[axis] = segment.axis_steps[axis];
        }
    }
    return motion;
}

void initialize_block_phase(MotionState& motion) {
    if (!motion.uses_block_phase || motion.phase_denominator <= 0) return;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        motion.axis_error[axis] = motion.phase_denominator >> 1;
    }
}

void IRAM_ATTR trigger_pulse(const std::size_t axis) {
    driver::rmt_step_pulse_driver_trigger(axis);
    s_rmt_triggers[axis].fetch_add(1, std::memory_order_relaxed);
}

void IRAM_ATTR prepare_next_step_mask(MotionState& motion) {
    std::uint8_t step_mask = 0;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        motion.axis_error[axis] += motion.axis_increment[axis];
        // Classic Grbl initializes a block DDA counter at half the major-axis
        // count and steps on a strict overflow. Keep the legacy >= behavior
        // for system/local segments, which start from zero instead.
        const bool should_step = motion.uses_block_phase
                                     ? motion.axis_error[axis] > motion.phase_denominator
                                     : motion.axis_error[axis] >= motion.phase_denominator;
        if (!should_step) continue;

        if (!motion.axis_stopped[axis]) {
            step_mask |= static_cast<std::uint8_t>(1U << axis);
        }
        motion.axis_error[axis] -= motion.phase_denominator;
    }
    motion.prepared_step_mask = step_mask;
    motion.prepared_step_mask_valid = true;
}

void IRAM_ATTR emit_prepared_step_mask(MotionState& motion) {
    if (!motion.prepared_step_mask_valid) return;

    const std::uint8_t step_mask = motion.prepared_step_mask;
    motion.prepared_step_mask = 0;
    motion.prepared_step_mask_valid = false;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if ((step_mask & static_cast<std::uint8_t>(1U << axis)) == 0 || motion.axis_stopped[axis]) {
            continue;
        }
        const auto current_axis = static_cast<machine::Axis>(axis);
        trigger_pulse(axis);
        // System motion may be stopped by a limit ISR, so it must keep
        // position current after every emitted pulse. Normal planner motion
        // commits its already-emitted pulses at segment end.
        if (motion.system_motion) {
            stepper_record_executed_steps_from_isr(current_axis, 1);
        }
        ++motion.emitted_axis_steps[axis];
    }
}

void IRAM_ATTR apply_segment_direction_from_isr(const StepSegment& segment) {
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const bool positive_direction = segment.positive_direction[axis];
        if (s_applied_direction_valid && s_applied_direction[axis] == positive_direction) {
            continue;
        }
        stepper_set_direction_from_isr(static_cast<machine::Axis>(axis),
                                       positive_direction ? Direction::Positive : Direction::Negative);
        s_applied_direction[axis] = positive_direction;
        s_direction_writes.fetch_add(1, std::memory_order_relaxed);
    }
    s_applied_direction_valid = true;
}

bool IRAM_ATTR timer_alarm_callback(void*) {
    BaseType_t task_woken = pdFALSE;
    portENTER_CRITICAL_ISR(&s_motion_lock);
    if (!s_busy.load()) {
        portEXIT_CRITICAL_ISR(&s_motion_lock);
        return false;
    }

    // Emit the DDA decision prepared by the preceding timer tick before any
    // segment bookkeeping for this tick can delay the RMT trigger.
    emit_prepared_step_mask(s_motion);
    s_pulse_callbacks.fetch_add(1, std::memory_order_relaxed);
    ++s_motion.completed_ticks;
    const bool finished = s_motion.completed_ticks == s_motion.execution_ticks;
    if (finished) {
        const MotionState completed_motion = s_motion;
        StepSegment completed_segment = {};
        completed_segment.execution_id = completed_motion.execution_id;
        completed_segment.axis_steps = {
            completed_motion.emitted_axis_steps[0],
            completed_motion.emitted_axis_steps[1],
        };
        completed_segment.major_steps = completed_motion.axis_steps[0] > completed_motion.axis_steps[1]
                                            ? completed_motion.axis_steps[0]
                                            : completed_motion.axis_steps[1];
        completed_segment.end_speed_tenths_mm_per_min = completed_motion.end_speed_tenths_mm_per_min;
        if (!completed_motion.system_motion) {
            for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                stepper_record_executed_steps_from_isr(
                    static_cast<machine::Axis>(axis), completed_motion.emitted_axis_steps[axis]);
            }
        }
        stepper_prep_record_segment_completed_from_isr(completed_segment);
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
        } else if (completed_motion.ends_feed_hold) {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            stepper_prep_record_hold_complete_from_isr();
            stepper_segment_request_refill_from_isr(&task_woken);
        } else if (s_feed_hold_requested.load(std::memory_order_relaxed) && !completed_motion.feed_hold) {
            // The executor only ends the active segment. The persistent prep
            // context owns the later deceleration and resume work.
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            stepper_prep_record_hold_boundary_from_isr();
            stepper_segment_request_refill_from_isr(&task_woken);
        } else if (stepper_segment_consume(next)) {
            apply_segment_direction_from_isr(next);
            MotionState next_motion = motion_from_segment(next);
            s_segment_handoffs.fetch_add(1, std::memory_order_relaxed);
            if (completed_motion.uses_block_phase && next_motion.uses_block_phase &&
                completed_motion.execution_id == next_motion.execution_id &&
                completed_motion.phase_denominator == next_motion.phase_denominator) {
                for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                    next_motion.axis_error[axis] = completed_motion.axis_error[axis];
                }
                s_block_phase_continuations.fetch_add(1, std::memory_order_relaxed);
            } else {
                initialize_block_phase(next_motion);
                if (next_motion.uses_block_phase) {
                    s_block_phase_initializations.fetch_add(1, std::memory_order_relaxed);
                }
            }
            s_motion = next_motion;
            prepare_next_step_mask(s_motion);
            record_segment_schedule(s_motion);
            laser::laser_apply_segment_power_from_isr(next.laser_power_per_mille,
                                                       next.execution_id);
            stepper_segment_set_active(next);
            driver::step_timer_driver_update_interval_from_isr(s_motion.effective_step_interval_ticks);
            if (stepper_segment_needs_refill_from_isr()) {
                stepper_segment_request_refill_from_isr(&task_woken);
            }
        } else {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
            if (completed_motion.completes_block) {
                stepper_segment_record_block_boundary_stop_from_isr();
            } else {
                stepper_segment_record_underflow_from_isr();
                const SegmentCompletion fault = {
                    .execution_id = completed_motion.execution_id,
                    .status = SegmentCompletionStatus::Faulted,
                };
                stepper_completion_publish_from_isr(fault, &task_woken);
            }
        }
    } else {
        prepare_next_step_mask(s_motion);
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
    const std::uint32_t timer_ticks = segment_timer_ticks(segment);
    if (!s_initialized.load() || segment.major_steps <= 0 ||
        timer_ticks <= 3U * kStepTimerTicksPerMicrosecond ||
        segment.amass_level > kAmassMaximumLevel) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=invalid-request initialized=%d ticks=%lu",
                 s_initialized.load() ? 1 : 0,
                 static_cast<unsigned long>(timer_ticks));
        return ESP_ERR_INVALID_ARG;
    }
    const std::int32_t major = std::max(segment.axis_steps[0], segment.axis_steps[1]);
    if (s_busy.load() || major != segment.major_steps) {
        ESP_LOGE(kLogTag, "TIMER-REJECT reason=busy-or-segment-mismatch busy=%d steps=(%ld,%ld) major=%ld",
                 s_busy.load() ? 1 : 0, static_cast<long>(segment.axis_steps[0]),
                 static_cast<long>(segment.axis_steps[1]), static_cast<long>(segment.major_steps));
        return ESP_ERR_NOT_SUPPORTED;
    }
    s_applied_direction_valid = false;
    MotionState motion = motion_from_segment(segment);
    initialize_block_phase(motion);
    if (motion.uses_block_phase) {
        s_block_phase_initializations.fetch_add(1, std::memory_order_relaxed);
    }
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
        s_applied_direction[axis] = segment.positive_direction[axis];
        s_direction_writes.fetch_add(1, std::memory_order_relaxed);
    }
    portENTER_CRITICAL(&s_motion_lock);
    s_applied_direction_valid = true;
    prepare_next_step_mask(motion);
    s_motion = motion;
    s_busy.store(true);
    portEXIT_CRITICAL(&s_motion_lock);
    record_segment_schedule(motion);
    laser::laser_apply_segment_power_from_isr(segment.laser_power_per_mille,
                                               segment.execution_id);
    stepper_segment_set_active(segment);
    const esp_err_t result = driver::step_timer_driver_start(motion.effective_step_interval_ticks);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "TIMER-START-REJECT error=%s steps=(%ld,%ld) ticks=%lu",
                 esp_err_to_name(result), static_cast<long>(segment.axis_steps[0]),
                 static_cast<long>(segment.axis_steps[1]),
                 static_cast<unsigned long>(timer_ticks));
        timer_stepper_executor_stop(); return ESP_FAIL;
    }
    stepper_segment_record_block_boundary_restart();
    return ESP_OK;
}

esp_err_t timer_stepper_executor_start_from_segment_ring() {
    StepSegment segment = {};
    if (!stepper_segment_consume(segment)) {
        return ESP_ERR_NOT_FOUND;
    }
    return timer_stepper_executor_start_segment(segment);
}

bool timer_stepper_executor_is_busy() { return s_busy.load(); }

void timer_stepper_executor_request_feed_hold() {
    s_feed_hold_requested.store(true, std::memory_order_release);
}

void timer_stepper_executor_resume() {
    s_feed_hold_requested.store(false, std::memory_order_release);
    stepper_prep_clear_hold_boundary();
}

void timer_stepper_executor_stop() {
    if (driver::step_timer_driver_is_initialized()) driver::step_timer_driver_stop();
    driver::rmt_step_pulse_driver_stop();
    laser::laser_disable();
    stepper_segment_clear_active_from_isr();
    MotionState motion = {};
    portENTER_CRITICAL(&s_motion_lock);
    motion = s_motion;
    s_motion = {};
    s_applied_direction_valid = false;
    s_busy.store(false);
    portEXIT_CRITICAL(&s_motion_lock);
    disable_active_axes(motion);
    s_feed_hold_requested.store(false, std::memory_order_release);
}

void IRAM_ATTR timer_stepper_executor_stop_from_isr() {
    if (!s_initialized.load(std::memory_order_relaxed)) return;
    driver::step_timer_driver_stop_from_isr();
    driver::rmt_step_pulse_driver_stop_from_isr();
    laser::laser_disable_from_isr();
    stepper_segment_clear_active_from_isr();
    portENTER_CRITICAL_ISR(&s_motion_lock);
    s_motion.prepared_step_mask = 0;
    s_motion.prepared_step_mask_valid = false;
    s_applied_direction_valid = false;
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
    diagnostics.timer_running = s_busy.load();
    diagnostics.scheduled_timer_events = s_scheduled_timer_events.load();
    diagnostics.scheduled_timer_ticks = s_scheduled_timer_ticks.load();
    diagnostics.segment_handoffs = s_segment_handoffs.load();
    diagnostics.block_phase_continuations = s_block_phase_continuations.load();
    diagnostics.block_phase_initializations = s_block_phase_initializations.load();
    diagnostics.pulse_callbacks = s_pulse_callbacks.load();
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        diagnostics.rmt_triggers[axis] = s_rmt_triggers[axis].load();
    }
    diagnostics.direction_writes = s_direction_writes.load();
    return diagnostics;
}

}  // namespace ray2::stepper
