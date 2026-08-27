#include "stepper_prep.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

#include "raycube_machine.hpp"

namespace ray2::stepper {
namespace {

constexpr std::uint32_t kMinimumStepIntervalUs = 4;

std::array<StepSegment, kStepSegmentBufferLength> s_segment_buffer = {};
std::atomic<std::uint32_t> s_segment_head = 0;
std::atomic<std::uint32_t> s_segment_tail = 0;
std::atomic<std::size_t> s_segment_high_water = 0;
std::atomic<std::uint64_t> s_segment_underflow_count = 0;
std::atomic<std::uint64_t> s_active_execution_id = 0;
std::atomic<std::uint32_t> s_active_step_interval_us = 0;
std::atomic<std::uint32_t> s_active_amass_timer_interval_us = 0;
std::atomic<std::uint8_t> s_active_amass_level = 0;
std::atomic<std::uint32_t> s_active_start_speed_tenths_mm_per_min = 0;
std::atomic<std::uint32_t> s_active_end_speed_tenths_mm_per_min = 0;

std::array<SegmentCompletion, kStepCompletionBufferLength> s_completion_buffer = {};
std::atomic<std::uint32_t> s_completion_head = 0;
std::atomic<std::uint32_t> s_completion_tail = 0;
std::atomic<TaskHandle_t> s_completion_notification_task = nullptr;

StepperProfile s_profile = {};
planner::PlannerBlock s_source_block = {};
std::atomic<std::uint64_t> s_context_execution_id = 0;
std::atomic<std::int32_t> s_context_total_major_steps = 0;
std::atomic<std::int32_t> s_context_executed_major_steps = 0;
std::array<std::atomic<std::int32_t>, machine::kAxisCount> s_context_executed_axis_steps = {};
std::atomic<std::uint32_t> s_context_current_speed_tenths_mm_per_min = 0;
std::atomic<bool> s_context_generation_complete = false;
std::atomic<bool> s_context_hold_boundary = false;
std::atomic<bool> s_context_feed_hold_active = false;
std::atomic<bool> s_context_hold_complete = false;
std::atomic<std::uint32_t> s_context_hold_carry_speed = 0;

static_assert(kStepSegmentBufferLength > 0, "segment ring requires storage");
static_assert(kStepCompletionBufferLength > 0, "completion ring requires storage");

std::size_t segment_count_snapshot() {
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_acquire);
    return std::min<std::size_t>(head - tail, kStepSegmentBufferLength);
}

std::size_t completion_count_snapshot() {
    const std::uint32_t head = s_completion_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_acquire);
    return std::min<std::size_t>(head - tail, kStepCompletionBufferLength);
}

void update_segment_high_water(const std::size_t count) {
    std::size_t observed = s_segment_high_water.load(std::memory_order_relaxed);
    while (count > observed &&
           !s_segment_high_water.compare_exchange_weak(observed, count,
                                                       std::memory_order_relaxed,
                                                       std::memory_order_relaxed)) {
    }
}

std::uint32_t speed_to_tenths_mm_per_min(const double speed) {
    if (!std::isfinite(speed) || speed <= 0.0) return 0;
    return static_cast<std::uint32_t>(std::min(
        std::llround(speed * 10.0), static_cast<long long>(std::numeric_limits<std::uint32_t>::max())));
}

bool block_is_valid(const planner::PlannerBlock& block) {
    return block.type == planner::MotionBlockType::Motion && block.step_event_count > 0 &&
           std::isfinite(block.millimeters) && block.millimeters > 0.0 &&
           std::isfinite(block.programmed_rate_mm_per_min) &&
           block.programmed_rate_mm_per_min > 0.0 &&
           std::isfinite(block.acceleration_mm_per_sec2) &&
           block.acceleration_mm_per_sec2 > 0.0;
}

double speed_at_distance(const double distance_mm) {
    const double acceleration = s_profile.acceleration_mm_per_min2;
    if (distance_mm < s_profile.accelerate_distance_mm) {
        return std::sqrt(s_profile.entry_speed_mm_per_min * s_profile.entry_speed_mm_per_min +
                         2.0 * acceleration * distance_mm);
    }
    if (distance_mm < s_profile.decelerate_after_mm) {
        return s_profile.nominal_speed_mm_per_min;
    }
    const double distance_to_end = s_profile.length_mm - distance_mm;
    return std::sqrt(std::max(s_profile.exit_speed_mm_per_min * s_profile.exit_speed_mm_per_min,
                              s_profile.exit_speed_mm_per_min * s_profile.exit_speed_mm_per_min +
                                  2.0 * acceleration * distance_to_end));
}

StepSegmentPhase phase_at_distance(const double distance_mm) {
    if (distance_mm < s_profile.accelerate_distance_mm) {
        return StepSegmentPhase::Accelerate;
    }
    if (distance_mm < s_profile.decelerate_after_mm) {
        return StepSegmentPhase::Cruise;
    }
    return StepSegmentPhase::Decelerate;
}

double time_at_distance_us(const double distance_mm) {
    const double acceleration = s_profile.acceleration_mm_per_min2;
    const double clamped_distance = std::clamp(distance_mm, 0.0, s_profile.length_mm);
    const double accelerate_time_us =
        (s_profile.nominal_speed_mm_per_min - s_profile.entry_speed_mm_per_min) *
        60000000.0 / acceleration;
    if (clamped_distance <= s_profile.accelerate_distance_mm) {
        return (speed_at_distance(clamped_distance) - s_profile.entry_speed_mm_per_min) *
            60000000.0 / acceleration;
    }
    if (clamped_distance <= s_profile.decelerate_after_mm) {
        return accelerate_time_us +
            (clamped_distance - s_profile.accelerate_distance_mm) * 60000000.0 /
                s_profile.nominal_speed_mm_per_min;
    }
    const double cruise_time_us =
        (s_profile.decelerate_after_mm - s_profile.accelerate_distance_mm) * 60000000.0 /
        s_profile.nominal_speed_mm_per_min;
    return accelerate_time_us + cruise_time_us +
        (s_profile.nominal_speed_mm_per_min - speed_at_distance(clamped_distance)) *
            60000000.0 / acceleration;
}

StepSegment build_segment(const std::uint32_t segment_index) {
    const std::int32_t total_steps = s_profile.total_major_steps;
    const std::int32_t first_step = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(total_steps) * segment_index) / s_profile.total_segments);
    const std::int32_t final_step = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(total_steps) * (segment_index + 1)) / s_profile.total_segments);
    const double start_distance = s_profile.length_mm * first_step / total_steps;
    const double final_distance = s_profile.length_mm * final_step / total_steps;
    const double start_speed = speed_at_distance(start_distance);
    const double final_speed = speed_at_distance(final_distance);
    const double duration_us = time_at_distance_us(final_distance) - time_at_distance_us(start_distance);

    StepSegment segment = {};
    segment.execution_id = s_profile.execution_id;
    segment.major_steps = final_step - first_step;
    segment.duration_us = static_cast<std::uint32_t>(std::max(1.0, std::round(duration_us)));
    segment.step_interval_us = std::max(
        kMinimumStepIntervalUs,
        static_cast<std::uint32_t>(std::max(1.0, std::round(duration_us / segment.major_steps))));
    segment.start_speed_mm_per_min = start_speed;
    segment.end_speed_mm_per_min = final_speed;
    segment.start_speed_tenths_mm_per_min = speed_to_tenths_mm_per_min(start_speed);
    segment.end_speed_tenths_mm_per_min = speed_to_tenths_mm_per_min(final_speed);
    const bool laser_enabled =
        s_source_block.request.motion_mode == motion::MotionType::Linear &&
        s_source_block.request.spindle_mode != motion::SpindleMode::Off;
    if (laser_enabled &&
        s_source_block.request.spindle_mode == motion::SpindleMode::CounterClockwise) {
        const double actual_speed =
            (final_distance - start_distance) * 60000000.0 / std::max(1.0, duration_us);
        segment.laser_power = s_source_block.request.spindle_power * std::clamp(
            actual_speed / s_source_block.programmed_rate_mm_per_min, 0.0, 1.0);
    } else if (laser_enabled) {
        segment.laser_power = s_source_block.request.spindle_power;
    }
    const double max_power = static_cast<double>(machine::raycube_laser().max_power);
    segment.laser_power_per_mille = static_cast<std::uint16_t>(std::clamp(
        std::lround(segment.laser_power * 1000.0 / max_power), 0L, 1000L));
    segment.motion_mode = s_source_block.request.motion_mode;
    segment.spindle_mode = s_source_block.request.spindle_mode;
    segment.phase = phase_at_distance(start_distance);
    const bool finishes_profile = segment_index + 1 == s_profile.total_segments;
    segment.completes_block = finishes_profile && s_profile.completes_source_block;
    segment.feed_hold = s_profile.feed_hold;
    segment.ends_feed_hold = s_profile.ends_feed_hold && finishes_profile;
    if (segment.feed_hold) {
        segment.laser_power = 0.0;
        segment.laser_power_per_mille = 0;
    }
    for (std::size_t axis = 0; axis < segment.axis_steps.size(); ++axis) {
        const std::int32_t axis_total = s_source_block.plan.steps[axis];
        const std::int32_t axis_first = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * first_step) / total_steps);
        const std::int32_t axis_final = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * final_step) / total_steps);
        segment.axis_steps[axis] = axis_final - axis_first;
        segment.positive_direction[axis] = s_source_block.plan.positive_direction[axis];
    }
    // AMASS is selected from the source block, not this short segment. A valid
    // diagonal may contain a segment with zero minor-axis steps after integer
    // partitioning, but it still belongs to a multi-axis Bresenham stream.
    const std::array<std::int32_t, machine::kAxisCount> source_axis_steps = {
        s_source_block.plan.steps[0],
        s_source_block.plan.steps[1],
    };
    segment.amass_level = stepper_select_amass_level(segment.step_interval_us, source_axis_steps);
    return segment;
}

}  // namespace

std::uint8_t stepper_select_amass_level(
    const std::uint32_t step_interval_us,
    const std::array<std::int32_t, 2>& axis_steps) {
    if (step_interval_us == 0 || axis_steps[0] == 0 || axis_steps[1] == 0) return 0;
    constexpr std::uint32_t kAmassThresholdUs = 1000000U / kAmassBaseStepRateHz;
    std::uint32_t interval = step_interval_us;
    std::uint8_t level = 0;
    while (level < kAmassMaximumLevel && interval >= kAmassThresholdUs) {
        ++level;
        interval /= 2U;
    }
    return level;
}

void stepper_prep_reset() {
    // Reset is only valid after the executor has stopped its ISR consumer.
    s_segment_head.store(0, std::memory_order_relaxed);
    s_segment_tail.store(0, std::memory_order_relaxed);
    s_segment_high_water.store(0, std::memory_order_relaxed);
    s_segment_underflow_count.store(0, std::memory_order_relaxed);
    s_active_execution_id.store(0, std::memory_order_relaxed);
    s_active_step_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_level.store(0, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_completion_head.store(0, std::memory_order_relaxed);
    s_completion_tail.store(0, std::memory_order_relaxed);
    s_context_execution_id.store(0, std::memory_order_relaxed);
    s_context_total_major_steps.store(0, std::memory_order_relaxed);
    s_context_executed_major_steps.store(0, std::memory_order_relaxed);
    for (auto& steps : s_context_executed_axis_steps) {
        steps.store(0, std::memory_order_relaxed);
    }
    s_context_current_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_context_generation_complete.store(false, std::memory_order_relaxed);
    s_context_hold_boundary.store(false, std::memory_order_relaxed);
    s_context_feed_hold_active.store(false, std::memory_order_relaxed);
    s_context_hold_complete.store(false, std::memory_order_relaxed);
    s_context_hold_carry_speed.store(0, std::memory_order_relaxed);
    s_profile = {};
    s_source_block = {};
}

bool stepper_prep_start_block(const planner::PlannerBlock& block) {
    if (!block_is_valid(block) || s_profile.active) {
        return false;
    }

    const double entry_speed = std::sqrt(std::max(0.0, block.entry_speed_sqr));
    const double exit_speed = std::sqrt(std::max(0.0, block.exit_speed_sqr));
    const double nominal_speed = block.programmed_rate_mm_per_min;
    const double acceleration = block.acceleration_mm_per_sec2 * 60.0 * 60.0;
    double accelerate_distance = std::max(0.0,
        (nominal_speed * nominal_speed - entry_speed * entry_speed) / (2.0 * acceleration));
    double decelerate_distance = std::max(0.0,
        (nominal_speed * nominal_speed - exit_speed * exit_speed) / (2.0 * acceleration));
    double peak_speed = nominal_speed;
    if (accelerate_distance + decelerate_distance > block.millimeters) {
        peak_speed = std::sqrt(std::max(0.0,
            (2.0 * acceleration * block.millimeters + entry_speed * entry_speed +
             exit_speed * exit_speed) / 2.0));
        accelerate_distance = std::max(0.0,
            (peak_speed * peak_speed - entry_speed * entry_speed) / (2.0 * acceleration));
        decelerate_distance = std::max(0.0,
            (peak_speed * peak_speed - exit_speed * exit_speed) / (2.0 * acceleration));
    }
    const double accelerate_time_us =
        (peak_speed - entry_speed) * 60000000.0 / acceleration;
    const double cruise_distance = std::max(0.0, block.millimeters - accelerate_distance - decelerate_distance);
    const double cruise_time_us = cruise_distance * 60000000.0 / peak_speed;
    const double decelerate_time_us =
        (peak_speed - exit_speed) * 60000000.0 / acceleration;
    const double total_time_us = accelerate_time_us + cruise_time_us + decelerate_time_us;
    const std::uint32_t target_segments = static_cast<std::uint32_t>(std::max(
        1.0, std::ceil(total_time_us / kStepSegmentTargetDurationUs)));

    s_source_block = block;
    s_profile = {
        .execution_id = block.execution_id,
        .total_major_steps = block.step_event_count,
        .total_segments = std::min(target_segments, static_cast<std::uint32_t>(block.step_event_count)),
        .next_segment_index = 0,
        .length_mm = block.millimeters,
        .entry_speed_mm_per_min = entry_speed,
        .nominal_speed_mm_per_min = peak_speed,
        .exit_speed_mm_per_min = exit_speed,
        .acceleration_mm_per_min2 = acceleration,
        .accelerate_distance_mm = accelerate_distance,
        .decelerate_after_mm = block.millimeters - decelerate_distance,
        .active = true,
        .generation_complete = false,
        .feed_hold = false,
        .ends_feed_hold = false,
        .completes_source_block = true,
    };
    s_context_total_major_steps.store(block.step_event_count, std::memory_order_relaxed);
    s_context_executed_major_steps.store(0, std::memory_order_relaxed);
    for (auto& steps : s_context_executed_axis_steps) {
        steps.store(0, std::memory_order_relaxed);
    }
    s_context_current_speed_tenths_mm_per_min.store(
        speed_to_tenths_mm_per_min(entry_speed), std::memory_order_relaxed);
    s_context_generation_complete.store(false, std::memory_order_relaxed);
    s_context_hold_boundary.store(false, std::memory_order_relaxed);
    s_context_feed_hold_active.store(false, std::memory_order_relaxed);
    s_context_hold_complete.store(false, std::memory_order_relaxed);
    s_context_execution_id.store(block.execution_id, std::memory_order_release);
    return true;
}

void stepper_prep_cancel_block() {
    s_profile = {};
    s_source_block = {};
    s_context_execution_id.store(0, std::memory_order_release);
    s_context_total_major_steps.store(0, std::memory_order_relaxed);
    s_context_executed_major_steps.store(0, std::memory_order_relaxed);
    for (auto& steps : s_context_executed_axis_steps) {
        steps.store(0, std::memory_order_relaxed);
    }
    s_context_current_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_context_generation_complete.store(false, std::memory_order_relaxed);
    s_context_hold_boundary.store(false, std::memory_order_relaxed);
}

std::size_t stepper_prep_fill() {
    std::size_t generated = 0;
    while (s_profile.active && s_profile.next_segment_index < s_profile.total_segments) {
        const StepSegment segment = build_segment(s_profile.next_segment_index);
        if (!stepper_segment_publish(segment)) {
            break;
        }
        ++s_profile.next_segment_index;
        ++generated;
    }
    if (s_profile.next_segment_index == s_profile.total_segments) {
        // The final segment being published does not mean it has executed.
        s_profile.generation_complete = true;
        s_context_generation_complete.store(true, std::memory_order_release);
    }
    return generated;
}

bool stepper_prep_complete_block(const std::uint64_t execution_id) {
    if (!s_profile.active || s_profile.execution_id != execution_id ||
        !s_profile.generation_complete ||
        s_context_executed_major_steps.load(std::memory_order_acquire) !=
            s_context_total_major_steps.load(std::memory_order_acquire)) {
        return false;
    }
    const bool hold_ended = s_profile.feed_hold && s_profile.ends_feed_hold;
    const bool continue_hold = s_profile.feed_hold && !s_profile.ends_feed_hold;
    const std::uint32_t carry_speed =
        s_context_current_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    stepper_prep_cancel_block();
    if (hold_ended) {
        s_context_hold_complete.store(true, std::memory_order_release);
    }
    if (continue_hold) {
        s_context_hold_carry_speed.store(carry_speed, std::memory_order_release);
        s_context_feed_hold_active.store(true, std::memory_order_release);
    }
    return true;
}

bool stepper_prep_begin_feed_hold() {
    if (!s_profile.active) return false;
    const std::int32_t total_steps = s_context_total_major_steps.load(std::memory_order_acquire);
    const std::int32_t executed_steps = s_context_executed_major_steps.load(std::memory_order_acquire);
    const std::int32_t remaining_steps = total_steps - executed_steps;
    const double speed = static_cast<double>(
        s_context_current_speed_tenths_mm_per_min.load(std::memory_order_acquire)) / 10.0;
    if (remaining_steps <= 0 || speed <= 0.0) {
        s_context_hold_complete.store(true, std::memory_order_release);
        return false;
    }
    const double remaining_mm = s_profile.length_mm * remaining_steps / s_profile.total_major_steps;
    const double acceleration = s_profile.acceleration_mm_per_min2;
    const double stop_mm = speed * speed / (2.0 * acceleration);
    const bool stops_here = stop_mm <= remaining_mm;
    const double travel_mm = stops_here ? stop_mm : remaining_mm;
    const std::int32_t travel_steps = std::clamp<std::int32_t>(
        static_cast<std::int32_t>(std::ceil(remaining_steps * travel_mm / remaining_mm)), 1, remaining_steps);

    planner::PlannerBlock residual = s_source_block;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const std::int32_t remaining_axis = std::max<std::int32_t>(
            0, s_source_block.plan.steps[axis] -
                   s_context_executed_axis_steps[axis].load(std::memory_order_acquire));
        residual.plan.steps[axis] = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(remaining_axis) * travel_steps) / remaining_steps);
    }
    residual.step_event_count = travel_steps;
    residual.millimeters = remaining_mm * travel_steps / remaining_steps;
    residual.programmed_rate_mm_per_min = speed;
    s_source_block = residual;
    const double exit_speed = stops_here ? 0.0 :
        std::sqrt(std::max(0.0, speed * speed - 2.0 * acceleration * residual.millimeters));
    const double total_time_us = (speed - exit_speed) * 60000000.0 / acceleration;
    s_profile = {
        .execution_id = residual.execution_id,
        .total_major_steps = travel_steps,
        .total_segments = std::min<std::uint32_t>(
            static_cast<std::uint32_t>(travel_steps),
            static_cast<std::uint32_t>(std::max(1.0, std::ceil(total_time_us / kStepSegmentTargetDurationUs)))),
        .next_segment_index = 0,
        .length_mm = residual.millimeters,
        .entry_speed_mm_per_min = speed,
        .nominal_speed_mm_per_min = speed,
        .exit_speed_mm_per_min = exit_speed,
        .acceleration_mm_per_min2 = acceleration,
        .accelerate_distance_mm = 0.0,
        .decelerate_after_mm = 0.0,
        .active = true,
        .generation_complete = false,
        .feed_hold = true,
        .ends_feed_hold = stops_here,
        .completes_source_block = travel_steps == remaining_steps,
    };
    s_context_hold_boundary.store(false, std::memory_order_release);
    s_context_feed_hold_active.store(true, std::memory_order_release);
    return true;
}

bool stepper_prep_continue_feed_hold(const planner::PlannerBlock& block) {
    const std::uint32_t carry = s_context_hold_carry_speed.exchange(0, std::memory_order_acq_rel);
    if (carry == 0 || !stepper_prep_start_block(block)) return false;
    s_context_current_speed_tenths_mm_per_min.store(carry, std::memory_order_release);
    return stepper_prep_begin_feed_hold();
}

bool stepper_prep_hold_boundary_pending() {
    return s_context_hold_boundary.load(std::memory_order_acquire) &&
           !s_context_feed_hold_active.load(std::memory_order_acquire);
}

bool stepper_prep_hold_complete() {
    return s_context_hold_complete.load(std::memory_order_acquire);
}

void IRAM_ATTR stepper_prep_record_segment_completed_from_isr(const StepSegment& segment) {
    if (segment.execution_id == 0 ||
        s_context_execution_id.load(std::memory_order_relaxed) != segment.execution_id) {
        return;
    }
    s_context_executed_major_steps.fetch_add(segment.major_steps, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        s_context_executed_axis_steps[axis].fetch_add(segment.axis_steps[axis],
                                                       std::memory_order_relaxed);
    }
    s_context_current_speed_tenths_mm_per_min.store(
        segment.end_speed_tenths_mm_per_min, std::memory_order_relaxed);
}

void IRAM_ATTR stepper_prep_record_hold_boundary_from_isr() {
    s_context_hold_boundary.store(true, std::memory_order_release);
}

void IRAM_ATTR stepper_prep_record_hold_complete_from_isr() {
    s_context_hold_boundary.store(false, std::memory_order_release);
    s_context_hold_complete.store(true, std::memory_order_release);
}

void stepper_prep_clear_hold_boundary() {
    s_context_hold_boundary.store(false, std::memory_order_release);
}

bool stepper_segment_publish(const StepSegment& segment) {
    const std::uint32_t head = s_segment_head.load(std::memory_order_relaxed);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_acquire);
    if (head - tail >= kStepSegmentBufferLength) {
        return false;
    }
    s_segment_buffer[head % kStepSegmentBufferLength] = segment;
    s_segment_head.store(head + 1, std::memory_order_release);
    update_segment_high_water(head + 1 - tail);
    return true;
}

bool stepper_segment_consume(StepSegment& segment) {
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_relaxed);
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    if (tail == head) {
        return false;
    }
    segment = s_segment_buffer[tail % kStepSegmentBufferLength];
    s_segment_tail.store(tail + 1, std::memory_order_release);
    return true;
}

void stepper_segment_discard_pending() {
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    s_segment_tail.store(head, std::memory_order_release);
}

std::size_t stepper_segment_count() {
    return segment_count_snapshot();
}

std::size_t stepper_segment_free_slots() {
    return kStepSegmentBufferLength - segment_count_snapshot();
}

void stepper_segment_record_underflow_from_isr() {
    s_segment_underflow_count.fetch_add(1, std::memory_order_relaxed);
}

bool stepper_segment_needs_refill_from_isr() {
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_relaxed);
    return head - tail == kStepSegmentLowWater;
}

void stepper_segment_request_refill_from_isr(
    BaseType_t* const higher_priority_task_woken) {
    const TaskHandle_t task = s_completion_notification_task.load(std::memory_order_acquire);
    if (task != nullptr && higher_priority_task_woken != nullptr) {
        vTaskNotifyGiveFromISR(task, higher_priority_task_woken);
    }
}

void IRAM_ATTR stepper_segment_set_active(const StepSegment& segment) {
    s_active_step_interval_us.store(segment.step_interval_us, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(
        std::max<std::uint32_t>(4U,
                                segment.step_interval_us / (1U << segment.amass_level)),
        std::memory_order_relaxed);
    s_active_amass_level.store(segment.amass_level, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(
        segment.start_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(
        segment.end_speed_tenths_mm_per_min, std::memory_order_relaxed);
    s_active_execution_id.store(segment.execution_id, std::memory_order_release);
}

void stepper_segment_clear_active_from_isr() {
    s_active_execution_id.store(0, std::memory_order_release);
    s_active_step_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_level.store(0, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
}

void stepper_completion_set_notification_task(const TaskHandle_t task) {
    s_completion_notification_task.store(task, std::memory_order_release);
}

bool stepper_completion_publish_from_isr(
    const SegmentCompletion& completion, BaseType_t* const higher_priority_task_woken) {
    const std::uint32_t head = s_completion_head.load(std::memory_order_relaxed);
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_acquire);
    if (head - tail >= kStepCompletionBufferLength) {
        return false;
    }
    s_completion_buffer[head % kStepCompletionBufferLength] = completion;
    s_completion_head.store(head + 1, std::memory_order_release);

    const TaskHandle_t task = s_completion_notification_task.load(std::memory_order_acquire);
    if (task != nullptr && higher_priority_task_woken != nullptr) {
        vTaskNotifyGiveFromISR(task, higher_priority_task_woken);
    }
    return true;
}

bool stepper_completion_consume(SegmentCompletion& completion) {
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_relaxed);
    const std::uint32_t head = s_completion_head.load(std::memory_order_acquire);
    if (tail == head) {
        return false;
    }
    completion = s_completion_buffer[tail % kStepCompletionBufferLength];
    s_completion_tail.store(tail + 1, std::memory_order_release);
    return true;
}

SegmentExecutionDiagnostics stepper_segment_diagnostics() {
    SegmentExecutionDiagnostics diagnostics = {};
    diagnostics.queued_segments = segment_count_snapshot();
    diagnostics.high_water_segments = s_segment_high_water.load(std::memory_order_relaxed);
    diagnostics.underflow_count = s_segment_underflow_count.load(std::memory_order_relaxed);
    diagnostics.active_execution_id = s_active_execution_id.load(std::memory_order_acquire);
    diagnostics.active = diagnostics.active_execution_id != 0;
    diagnostics.active_step_interval_us = s_active_step_interval_us.load(std::memory_order_relaxed);
    diagnostics.active_amass_timer_interval_us =
        s_active_amass_timer_interval_us.load(std::memory_order_relaxed);
    diagnostics.active_amass_level = s_active_amass_level.load(std::memory_order_relaxed);
    diagnostics.active_start_speed_tenths_mm_per_min =
        s_active_start_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    diagnostics.active_end_speed_tenths_mm_per_min =
        s_active_end_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    diagnostics.pending_completions = completion_count_snapshot();
    return diagnostics;
}

const StepperProfile& stepper_prep_profile() {
    return s_profile;
}

StepperPrepDiagnostics stepper_prep_diagnostics() {
    StepperPrepDiagnostics diagnostics = {};
    diagnostics.execution_id = s_context_execution_id.load(std::memory_order_acquire);
    diagnostics.active = diagnostics.execution_id != 0;
    diagnostics.generation_complete =
        s_context_generation_complete.load(std::memory_order_acquire);
    diagnostics.hold_boundary = stepper_prep_hold_boundary_pending();
    diagnostics.total_major_steps = s_context_total_major_steps.load(std::memory_order_relaxed);
    diagnostics.executed_major_steps = s_context_executed_major_steps.load(std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        diagnostics.executed_axis_steps[axis] =
            s_context_executed_axis_steps[axis].load(std::memory_order_relaxed);
    }
    diagnostics.current_speed_tenths_mm_per_min =
        s_context_current_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    return diagnostics;
}

}  // namespace ray2::stepper
