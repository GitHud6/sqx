#include "timer_stepper_executor.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>

#include "rmt_step_pulse_driver.hpp"
#include "step_timer_driver.hpp"
#include "esp_cpu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "laser_driver.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"

namespace ray2::stepper {
namespace {

constexpr char kLogTag[] = "timer_stepper";
constexpr std::uint32_t kTimerIsrCyclesPerMicrosecond = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
static_assert(kTimerIsrCyclesPerMicrosecond > 0, "CPU frequency must be configured");

std::atomic<bool> s_initialized = false;
std::atomic<bool> s_busy = false;
// These counters are sampled once per second for diagnostics. 32-bit values
// are sufficient for that window and avoid expensive 64-bit atomics in ISR.
std::atomic<std::uint32_t> s_scheduled_timer_ticks = 0;
std::atomic<std::uint32_t> s_scheduled_timer_events = 0;
std::atomic<std::uint32_t> s_direction_writes = 0;
std::atomic<std::uint32_t> s_interval_min_ticks = UINT32_MAX;
std::atomic<std::uint32_t> s_interval_max_ticks = 0;
std::atomic<std::uint32_t> s_interval_max_delta_ticks = 0;
struct TimerLatenessCounters {
    std::uint32_t samples = 0;
    std::uint32_t max_ticks = 0;
    std::uint32_t boundary_samples = 0;
    std::uint32_t boundary_max_ticks = 0;
    std::uint32_t over_2us = 0;
    std::uint32_t over_5us = 0;
    std::uint32_t over_10us = 0;
    std::uint32_t mask_samples[4] = {};
    std::uint32_t over_10us_masks[4] = {};
    std::uint32_t body_samples = 0;
    std::uint32_t body_max_ticks = 0;
    std::uint32_t body_boundary_samples = 0;
    std::uint32_t body_boundary_max_ticks = 0;
    std::uint32_t body_over_10us = 0;
    std::uint32_t body_counter_wrapped = 0;
};
struct TimerHotPathCounters {
    std::uint32_t samples = 0;
    std::uint32_t pulse_front_max_cycles = 0;
    std::uint32_t normal_samples = 0;
    std::uint32_t normal_max_cycles = 0;
    std::uint32_t normal_over_10us = 0;
    std::uint32_t boundary_samples = 0;
    std::uint32_t boundary_max_cycles = 0;
    std::uint32_t boundary_over_10us = 0;
    std::uint32_t boundary_peak_previous_interval_ticks = 0;
    std::uint32_t boundary_peak_next_interval_ticks = 0;
    std::uint8_t boundary_peak_previous_amass_level = 0;
    std::uint8_t boundary_peak_next_amass_level = 0;
    bool boundary_peak_completes_block = false;
};
// Updated while s_motion_lock is held. Plain counters avoid adding atomic-RMW
// traffic to the timing path being measured.
TimerLatenessCounters s_timer_lateness = {};
TimerHotPathCounters s_timer_hot_path = {};
std::atomic<std::uint32_t> s_boundary_max_delta_ticks = 0;
std::atomic<std::uint32_t> s_boundary_samples = 0;
std::atomic<std::uint32_t> s_amass_level_changes = 0;
std::atomic<std::uint32_t> s_amass_transitions[4][4] = {};
std::atomic<std::uint32_t> s_amass_3_to_0 = 0;
std::atomic<std::uint32_t> s_amass_0_to_3 = 0;
std::atomic<std::uint64_t> s_boundary_prev_execution_id = 0;
std::atomic<std::uint64_t> s_boundary_next_execution_id = 0;
std::atomic<std::uint32_t> s_boundary_prev_interval_ticks = 0;
std::atomic<std::uint32_t> s_boundary_next_interval_ticks = 0;
std::atomic<std::uint32_t> s_boundary_prev_base_interval_ticks = 0;
std::atomic<std::uint32_t> s_boundary_next_base_interval_ticks = 0;
std::atomic<std::uint32_t> s_boundary_prev_duration_us = 0;
std::atomic<std::uint32_t> s_boundary_next_duration_us = 0;
std::atomic<std::uint32_t> s_boundary_prev_start_speed_tenths_mm_per_min = 0;
std::atomic<std::uint32_t> s_boundary_prev_end_speed_tenths_mm_per_min = 0;
std::atomic<std::uint32_t> s_boundary_next_start_speed_tenths_mm_per_min = 0;
std::atomic<std::uint32_t> s_boundary_next_end_speed_tenths_mm_per_min = 0;
std::atomic<std::uint8_t> s_boundary_prev_phase = 0;
std::atomic<std::uint8_t> s_boundary_next_phase = 0;
std::atomic<std::uint8_t> s_boundary_prev_amass_level = 0;
std::atomic<std::uint8_t> s_boundary_next_amass_level = 0;
std::atomic<std::int32_t> s_boundary_prev_major_steps = 0;
std::atomic<std::int32_t> s_boundary_next_major_steps = 0;
portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;

struct MotionState {
    std::int32_t axis_steps[machine::kAxisCount] = {};
    std::int32_t emitted_axis_steps[machine::kAxisCount] = {};
    std::int32_t execution_ticks = 0;
    std::int32_t completed_ticks = 0;
    std::int64_t axis_error[machine::kAxisCount] = {};
    std::int64_t axis_increment[machine::kAxisCount] = {};
    std::int64_t phase_denominator = 0;
    std::uint32_t duration_us = 0;
    std::uint32_t base_step_interval_ticks = 0;
    std::uint32_t effective_step_interval_ticks = 0;
    std::uint8_t amass_level = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    std::uint32_t start_speed_tenths_mm_per_min = 0;
    StepSegmentPhase phase = StepSegmentPhase::Cruise;
    std::uint64_t execution_id = 0;
    bool completes_block = false;
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
std::uint8_t s_last_emitted_step_mask = 0;

void IRAM_ATTR record_segment_schedule(const MotionState& motion) {
    s_scheduled_timer_ticks.fetch_add(
        static_cast<std::uint32_t>(motion.effective_step_interval_ticks) *
            static_cast<std::uint32_t>(motion.execution_ticks),
        std::memory_order_relaxed);
    s_scheduled_timer_events.fetch_add(static_cast<std::uint32_t>(motion.execution_ticks),
                                       std::memory_order_relaxed);
}

void IRAM_ATTR record_interval_sample(const std::uint32_t interval_ticks) {
    std::uint32_t previous = s_interval_min_ticks.load(std::memory_order_relaxed);
    while (interval_ticks < previous &&
           !s_interval_min_ticks.compare_exchange_weak(previous, interval_ticks,
                                                        std::memory_order_relaxed)) {
    }
    previous = s_interval_max_ticks.load(std::memory_order_relaxed);
    while (interval_ticks > previous &&
           !s_interval_max_ticks.compare_exchange_weak(previous, interval_ticks,
                                                        std::memory_order_relaxed)) {
    }
}

void IRAM_ATTR record_interval_delta(const MotionState& previous_motion,
                                     const MotionState& next_motion,
                                     const bool at_boundary) {
    const std::uint32_t previous_ticks = previous_motion.effective_step_interval_ticks;
    const std::uint32_t next_ticks = next_motion.effective_step_interval_ticks;
    const std::uint32_t delta = previous_ticks >= next_ticks
                                    ? previous_ticks - next_ticks
                                    : next_ticks - previous_ticks;
    std::uint32_t previous = s_interval_max_delta_ticks.load(std::memory_order_relaxed);
    while (delta > previous &&
           !s_interval_max_delta_ticks.compare_exchange_weak(previous, delta,
                                                             std::memory_order_relaxed)) {
    }
    if (!at_boundary) return;
    s_boundary_samples.fetch_add(1, std::memory_order_relaxed);
    if (previous_motion.amass_level < 4 && next_motion.amass_level < 4) {
        s_amass_transitions[previous_motion.amass_level][next_motion.amass_level].fetch_add(
            1, std::memory_order_relaxed);
    }
    if (previous_motion.amass_level != next_motion.amass_level) {
        s_amass_level_changes.fetch_add(1, std::memory_order_relaxed);
        if (previous_motion.amass_level == 3 && next_motion.amass_level == 0) {
            s_amass_3_to_0.fetch_add(1, std::memory_order_relaxed);
        } else if (previous_motion.amass_level == 0 && next_motion.amass_level == 3) {
            s_amass_0_to_3.fetch_add(1, std::memory_order_relaxed);
        }
    }
    // A block's final stop necessarily stretches its final physical step as
    // velocity approaches zero. Keep it in the aggregate counters, but do not
    // let that expected terminal tail hide the largest in-motion discontinuity.
    if (next_motion.completes_block && next_motion.end_speed_tenths_mm_per_min == 0) {
        return;
    }
    previous = s_boundary_max_delta_ticks.load(std::memory_order_relaxed);
    if (delta <= previous ||
        !s_boundary_max_delta_ticks.compare_exchange_strong(previous, delta,
                                                             std::memory_order_relaxed)) {
        return;
    }
    s_boundary_prev_execution_id.store(previous_motion.execution_id, std::memory_order_relaxed);
    s_boundary_next_execution_id.store(next_motion.execution_id, std::memory_order_relaxed);
    s_boundary_prev_interval_ticks.store(previous_ticks, std::memory_order_relaxed);
    s_boundary_next_interval_ticks.store(next_ticks, std::memory_order_relaxed);
    s_boundary_prev_base_interval_ticks.store(previous_motion.base_step_interval_ticks,
                                               std::memory_order_relaxed);
    s_boundary_next_base_interval_ticks.store(next_motion.base_step_interval_ticks,
                                               std::memory_order_relaxed);
    s_boundary_prev_duration_us.store(previous_motion.duration_us, std::memory_order_relaxed);
    s_boundary_next_duration_us.store(next_motion.duration_us, std::memory_order_relaxed);
    s_boundary_prev_start_speed_tenths_mm_per_min.store(
        previous_motion.start_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_boundary_prev_end_speed_tenths_mm_per_min.store(
        previous_motion.end_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_boundary_next_start_speed_tenths_mm_per_min.store(
        next_motion.start_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_boundary_next_end_speed_tenths_mm_per_min.store(
        next_motion.end_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_boundary_prev_phase.store(static_cast<std::uint8_t>(previous_motion.phase),
                                std::memory_order_relaxed);
    s_boundary_next_phase.store(static_cast<std::uint8_t>(next_motion.phase),
                                std::memory_order_relaxed);
    s_boundary_prev_amass_level.store(previous_motion.amass_level, std::memory_order_relaxed);
    s_boundary_next_amass_level.store(next_motion.amass_level, std::memory_order_relaxed);
    s_boundary_prev_major_steps.store(
        std::max(previous_motion.axis_steps[0], previous_motion.axis_steps[1]),
        std::memory_order_relaxed);
    s_boundary_next_major_steps.store(
        std::max(next_motion.axis_steps[0], next_motion.axis_steps[1]),
        std::memory_order_relaxed);
}

void IRAM_ATTR record_timer_lateness(const std::uint64_t lateness_ticks,
                                     const bool at_segment_boundary,
                                     const std::uint8_t preceding_step_mask) {
    const std::uint32_t ticks = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(lateness_ticks, UINT32_MAX));
    ++s_timer_lateness.samples;
    const std::size_t mask_index = preceding_step_mask & 0x03U;
    ++s_timer_lateness.mask_samples[mask_index];
    s_timer_lateness.max_ticks = std::max(s_timer_lateness.max_ticks, ticks);
    if (ticks > 2U * kStepTimerTicksPerMicrosecond) {
        ++s_timer_lateness.over_2us;
    }
    if (ticks > 5U * kStepTimerTicksPerMicrosecond) {
        ++s_timer_lateness.over_5us;
    }
    if (ticks > 10U * kStepTimerTicksPerMicrosecond) {
        ++s_timer_lateness.over_10us;
        ++s_timer_lateness.over_10us_masks[mask_index];
    }
    if (!at_segment_boundary) return;
    ++s_timer_lateness.boundary_samples;
    s_timer_lateness.boundary_max_ticks = std::max(s_timer_lateness.boundary_max_ticks, ticks);
}

void IRAM_ATTR record_timer_isr_body(const std::uint64_t start_counter_ticks,
                                     const std::uint64_t end_counter_ticks,
                                     const bool at_segment_boundary) {
    // Auto-reload crossing makes a single counter subtraction ambiguous. Keep
    // those events visible but do not turn them into a false short duration.
    if (end_counter_ticks < start_counter_ticks) {
        ++s_timer_lateness.body_counter_wrapped;
        return;
    }
    const std::uint64_t elapsed_ticks = end_counter_ticks - start_counter_ticks;
    const std::uint32_t ticks = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(elapsed_ticks, UINT32_MAX));
    ++s_timer_lateness.body_samples;
    s_timer_lateness.body_max_ticks = std::max(s_timer_lateness.body_max_ticks, ticks);
    if (ticks > 10U * kStepTimerTicksPerMicrosecond) {
        ++s_timer_lateness.body_over_10us;
    }
    if (!at_segment_boundary) return;
    ++s_timer_lateness.body_boundary_samples;
    s_timer_lateness.body_boundary_max_ticks = std::max(
        s_timer_lateness.body_boundary_max_ticks, ticks);
}

void IRAM_ATTR record_timer_hot_path(const std::uint32_t start_cycles,
                                     const std::uint32_t pulse_front_cycles,
                                     const std::uint32_t work_end_cycles,
                                     const bool at_segment_boundary,
                                     const std::uint32_t boundary_previous_interval_ticks,
                                     const std::uint32_t boundary_next_interval_ticks,
                                     const std::uint8_t boundary_previous_amass_level,
                                     const std::uint8_t boundary_next_amass_level,
                                     const bool boundary_completes_block) {
    // CPU CCOUNT is monotonic across Timer Group auto-reloads. Unsigned
    // subtraction is valid for one ISR even when the 32-bit counter wraps.
    const std::uint32_t pulse_front_elapsed = pulse_front_cycles - start_cycles;
    const std::uint32_t work_elapsed = work_end_cycles - start_cycles;
    ++s_timer_hot_path.samples;
    s_timer_hot_path.pulse_front_max_cycles = std::max(
        s_timer_hot_path.pulse_front_max_cycles, pulse_front_elapsed);
    if (!at_segment_boundary) {
        ++s_timer_hot_path.normal_samples;
        s_timer_hot_path.normal_max_cycles = std::max(
            s_timer_hot_path.normal_max_cycles, work_elapsed);
        if (work_elapsed > 10U * kTimerIsrCyclesPerMicrosecond) {
            ++s_timer_hot_path.normal_over_10us;
        }
        return;
    }
    ++s_timer_hot_path.boundary_samples;
    if (work_elapsed > s_timer_hot_path.boundary_max_cycles) {
        s_timer_hot_path.boundary_max_cycles = work_elapsed;
        s_timer_hot_path.boundary_peak_previous_interval_ticks =
            boundary_previous_interval_ticks;
        s_timer_hot_path.boundary_peak_next_interval_ticks = boundary_next_interval_ticks;
        s_timer_hot_path.boundary_peak_previous_amass_level = boundary_previous_amass_level;
        s_timer_hot_path.boundary_peak_next_amass_level = boundary_next_amass_level;
        s_timer_hot_path.boundary_peak_completes_block = boundary_completes_block;
    }
    if (work_elapsed > 10U * kTimerIsrCyclesPerMicrosecond) {
        ++s_timer_hot_path.boundary_over_10us;
    }
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
    motion.duration_us = segment.duration_us;
    motion.base_step_interval_ticks = segment_timer_ticks(segment);
    const std::uint32_t amass_scale = 1U << segment.amass_level;
    motion.execution_ticks = segment.major_steps * static_cast<std::int32_t>(amass_scale);
    motion.effective_step_interval_ticks = std::max<std::uint32_t>(
        1U, motion.base_step_interval_ticks / amass_scale);
    motion.amass_level = segment.amass_level;
    motion.start_speed_tenths_mm_per_min = segment.start_speed_tenths_mm_per_min;
    motion.end_speed_tenths_mm_per_min = segment.end_speed_tenths_mm_per_min;
    motion.phase = segment.phase;
    motion.execution_id = segment.execution_id;
    motion.completes_block = segment.completes_block;
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
    if (motion.uses_block_phase && motion.axis_increment[0] != 0 && motion.axis_increment[1] != 0) {
        if ((step_mask & 0x03U) == 0x03U && !motion.axis_stopped[0] && !motion.axis_stopped[1]) {
            trigger_pulse(0);
            trigger_pulse(1);
            ++motion.emitted_axis_steps[0];
            ++motion.emitted_axis_steps[1];
            if (motion.system_motion) {
                stepper_record_executed_steps_from_isr(machine::Axis::X, 1);
                stepper_record_executed_steps_from_isr(machine::Axis::Y, 1);
            }
            return;
        }
        for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
            if ((step_mask & static_cast<std::uint8_t>(1U << axis)) == 0 || motion.axis_stopped[axis]) {
                continue;
            }
            trigger_pulse(axis);
            if (motion.system_motion) {
                stepper_record_executed_steps_from_isr(static_cast<machine::Axis>(axis), 1);
            }
            ++motion.emitted_axis_steps[axis];
        }
        return;
    }
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
    const std::uint32_t hot_path_start_cycles = esp_cpu_get_cycle_count();
    const std::uint64_t lateness_ticks = driver::step_timer_driver_lateness_ticks_from_isr();
    portENTER_CRITICAL_ISR(&s_motion_lock);
    if (!s_busy.load()) {
        portEXIT_CRITICAL_ISR(&s_motion_lock);
        return false;
    }

    const std::uint8_t prepared_step_mask = s_motion.prepared_step_mask_valid
                                                ? s_motion.prepared_step_mask
                                                : 0;
    // The time to this callback was affected by the work emitted by the prior
    // callback, not by the mask that will be emitted below.
    const std::uint8_t preceding_step_mask = s_last_emitted_step_mask;
    std::uint32_t boundary_previous_interval_ticks = 0;
    std::uint32_t boundary_next_interval_ticks = 0;
    std::uint8_t boundary_previous_amass_level = 0;
    std::uint8_t boundary_next_amass_level = 0;
    bool boundary_completes_block = false;
    // Emit the DDA decision prepared by the preceding timer tick before any
    // segment bookkeeping for this tick can delay the RMT trigger.
    emit_prepared_step_mask(s_motion);
    const std::uint32_t pulse_front_cycles = esp_cpu_get_cycle_count();
    s_last_emitted_step_mask = prepared_step_mask;
    ++s_motion.completed_ticks;
    const bool finished = s_motion.completed_ticks == s_motion.execution_ticks;
    if (finished) {
        const MotionState completed_motion = s_motion;
        boundary_previous_interval_ticks = completed_motion.effective_step_interval_ticks;
        boundary_previous_amass_level = completed_motion.amass_level;
        boundary_completes_block = completed_motion.completes_block;
        const SegmentCompletion completion = {
            .execution_id = completed_motion.execution_id,
            .status = SegmentCompletionStatus::Completed,
            .axis_steps = {
                completed_motion.emitted_axis_steps[0],
                completed_motion.emitted_axis_steps[1],
            },
            .major_steps = completed_motion.axis_steps[0] > completed_motion.axis_steps[1]
                               ? completed_motion.axis_steps[0]
                               : completed_motion.axis_steps[1],
            .end_speed_tenths_mm_per_min = completed_motion.end_speed_tenths_mm_per_min,
            .completes_block = completed_motion.completes_block,
        };
        if (!completed_motion.system_motion) {
            for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                stepper_record_executed_steps_from_isr(
                    static_cast<machine::Axis>(axis), completed_motion.emitted_axis_steps[axis]);
            }
        }
        if (!completed_motion.system_motion) {
            stepper_completion_publish_from_isr(completion, &task_woken);
        }
        StepSegment next = {};
        if (completed_motion.system_motion) {
            s_busy.store(false);
            driver::step_timer_driver_stop_from_isr();
            laser::laser_disable_from_isr();
            stepper_segment_clear_active_from_isr();
        } else if (stepper_segment_consume(next)) {
            apply_segment_direction_from_isr(next);
            MotionState next_motion = motion_from_segment(next);
            boundary_next_interval_ticks = next_motion.effective_step_interval_ticks;
            boundary_next_amass_level = next_motion.amass_level;
            record_interval_sample(next_motion.effective_step_interval_ticks);
            record_interval_delta(completed_motion, next_motion, true);
            if (completed_motion.uses_block_phase && next_motion.uses_block_phase &&
                completed_motion.execution_id == next_motion.execution_id &&
                completed_motion.phase_denominator == next_motion.phase_denominator) {
                for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
                    next_motion.axis_error[axis] = completed_motion.axis_error[axis];
                }
            } else {
                initialize_block_phase(next_motion);
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
    const std::uint32_t hot_path_end_cycles = esp_cpu_get_cycle_count();
    record_timer_hot_path(hot_path_start_cycles, pulse_front_cycles, hot_path_end_cycles, finished,
                          boundary_previous_interval_ticks, boundary_next_interval_ticks,
                          boundary_previous_amass_level, boundary_next_amass_level,
                          boundary_completes_block);
    const std::uint64_t isr_end_counter_ticks =
        driver::step_timer_driver_lateness_ticks_from_isr();
    record_timer_lateness(lateness_ticks, finished, preceding_step_mask);
    record_timer_isr_body(lateness_ticks, isr_end_counter_ticks, finished);
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
    record_interval_sample(motion.effective_step_interval_ticks);
    initialize_block_phase(motion);
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
    s_last_emitted_step_mask = 0;
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

void timer_stepper_executor_stop() {
    if (driver::step_timer_driver_is_initialized()) driver::step_timer_driver_stop();
    driver::rmt_step_pulse_driver_stop();
    laser::laser_disable();
    stepper_segment_clear_active_from_isr();
    MotionState motion = {};
    portENTER_CRITICAL(&s_motion_lock);
    motion = s_motion;
    s_motion = {};
    s_last_emitted_step_mask = 0;
    s_applied_direction_valid = false;
    s_busy.store(false);
    portEXIT_CRITICAL(&s_motion_lock);
    disable_active_axes(motion);
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
    s_last_emitted_step_mask = 0;
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
    diagnostics.timer_init_core = driver::step_timer_driver_init_core();
    diagnostics.timer_isr_core = driver::step_timer_driver_isr_core();
    diagnostics.scheduled_timer_events = s_scheduled_timer_events.load();
    diagnostics.scheduled_timer_ticks = s_scheduled_timer_ticks.load();
    diagnostics.direction_writes = s_direction_writes.load();
    diagnostics.interval_min_ticks = s_interval_min_ticks.load();
    diagnostics.interval_max_ticks = s_interval_max_ticks.load();
    diagnostics.interval_max_delta_ticks = s_interval_max_delta_ticks.load();
    portENTER_CRITICAL(&s_motion_lock);
    diagnostics.timer_lateness_samples = s_timer_lateness.samples;
    diagnostics.timer_lateness_max_ticks = s_timer_lateness.max_ticks;
    diagnostics.timer_lateness_boundary_samples = s_timer_lateness.boundary_samples;
    diagnostics.timer_lateness_boundary_max_ticks = s_timer_lateness.boundary_max_ticks;
    diagnostics.timer_lateness_over_2us = s_timer_lateness.over_2us;
    diagnostics.timer_lateness_over_5us = s_timer_lateness.over_5us;
    diagnostics.timer_lateness_over_10us = s_timer_lateness.over_10us;
    for (std::size_t mask = 0; mask < 4; ++mask) {
        diagnostics.timer_lateness_mask_samples[mask] = s_timer_lateness.mask_samples[mask];
        diagnostics.timer_lateness_over_10us_masks[mask] = s_timer_lateness.over_10us_masks[mask];
    }
    diagnostics.timer_isr_body_samples = s_timer_lateness.body_samples;
    diagnostics.timer_isr_body_max_ticks = s_timer_lateness.body_max_ticks;
    diagnostics.timer_isr_body_boundary_samples = s_timer_lateness.body_boundary_samples;
    diagnostics.timer_isr_body_boundary_max_ticks = s_timer_lateness.body_boundary_max_ticks;
    diagnostics.timer_isr_body_over_10us = s_timer_lateness.body_over_10us;
    diagnostics.timer_isr_body_counter_wrapped = s_timer_lateness.body_counter_wrapped;
    diagnostics.timer_isr_cycles_per_microsecond = kTimerIsrCyclesPerMicrosecond;
    diagnostics.timer_isr_hot_path_samples = s_timer_hot_path.samples;
    diagnostics.timer_isr_pulse_front_max_cycles = s_timer_hot_path.pulse_front_max_cycles;
    diagnostics.timer_isr_normal_samples = s_timer_hot_path.normal_samples;
    diagnostics.timer_isr_normal_max_cycles = s_timer_hot_path.normal_max_cycles;
    diagnostics.timer_isr_normal_over_10us = s_timer_hot_path.normal_over_10us;
    diagnostics.timer_isr_boundary_hot_path_samples = s_timer_hot_path.boundary_samples;
    diagnostics.timer_isr_boundary_hot_path_max_cycles = s_timer_hot_path.boundary_max_cycles;
    diagnostics.timer_isr_boundary_hot_path_over_10us = s_timer_hot_path.boundary_over_10us;
    diagnostics.timer_isr_boundary_hot_path_peak_previous_interval_ticks =
        s_timer_hot_path.boundary_peak_previous_interval_ticks;
    diagnostics.timer_isr_boundary_hot_path_peak_next_interval_ticks =
        s_timer_hot_path.boundary_peak_next_interval_ticks;
    diagnostics.timer_isr_boundary_hot_path_peak_previous_amass_level =
        s_timer_hot_path.boundary_peak_previous_amass_level;
    diagnostics.timer_isr_boundary_hot_path_peak_next_amass_level =
        s_timer_hot_path.boundary_peak_next_amass_level;
    diagnostics.timer_isr_boundary_hot_path_peak_completes_block =
        s_timer_hot_path.boundary_peak_completes_block;
    portEXIT_CRITICAL(&s_motion_lock);
    diagnostics.boundary_max_delta_ticks = s_boundary_max_delta_ticks.load();
    diagnostics.boundary_samples = s_boundary_samples.load();
    diagnostics.amass_level_changes = s_amass_level_changes.load();
    for (std::size_t from = 0; from < 4; ++from) {
        for (std::size_t to = 0; to < 4; ++to) {
            diagnostics.amass_transitions[from][to] = s_amass_transitions[from][to].load();
        }
    }
    diagnostics.amass_3_to_0 = s_amass_3_to_0.load();
    diagnostics.amass_0_to_3 = s_amass_0_to_3.load();
    diagnostics.boundary_prev_execution_id = s_boundary_prev_execution_id.load();
    diagnostics.boundary_next_execution_id = s_boundary_next_execution_id.load();
    diagnostics.boundary_prev_interval_ticks = s_boundary_prev_interval_ticks.load();
    diagnostics.boundary_next_interval_ticks = s_boundary_next_interval_ticks.load();
    diagnostics.boundary_prev_base_interval_ticks = s_boundary_prev_base_interval_ticks.load();
    diagnostics.boundary_next_base_interval_ticks = s_boundary_next_base_interval_ticks.load();
    diagnostics.boundary_prev_duration_us = s_boundary_prev_duration_us.load();
    diagnostics.boundary_next_duration_us = s_boundary_next_duration_us.load();
    diagnostics.boundary_prev_start_speed_tenths_mm_per_min =
        s_boundary_prev_start_speed_tenths_mm_per_min.load();
    diagnostics.boundary_prev_end_speed_tenths_mm_per_min =
        s_boundary_prev_end_speed_tenths_mm_per_min.load();
    diagnostics.boundary_next_start_speed_tenths_mm_per_min =
        s_boundary_next_start_speed_tenths_mm_per_min.load();
    diagnostics.boundary_next_end_speed_tenths_mm_per_min =
        s_boundary_next_end_speed_tenths_mm_per_min.load();
    diagnostics.boundary_prev_phase = s_boundary_prev_phase.load();
    diagnostics.boundary_next_phase = s_boundary_next_phase.load();
    diagnostics.boundary_prev_amass_level = s_boundary_prev_amass_level.load();
    diagnostics.boundary_next_amass_level = s_boundary_next_amass_level.load();
    diagnostics.boundary_prev_major_steps = s_boundary_prev_major_steps.load();
    diagnostics.boundary_next_major_steps = s_boundary_next_major_steps.load();
    return diagnostics;
}

}  // namespace ray2::stepper
