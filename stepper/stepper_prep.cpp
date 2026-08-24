#include "stepper_prep.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "freertos/FreeRTOS.h"

namespace ray2::stepper {
namespace {

std::array<StepSegment, kStepSegmentBufferLength> s_segment_buffer = {};
std::size_t s_segment_head = 0;
std::size_t s_segment_tail = 0;
std::size_t s_segment_count = 0;
portMUX_TYPE s_segment_lock = portMUX_INITIALIZER_UNLOCKED;
StepperProfile s_profile = {};
planner::PlannerBlock s_source_block = {};

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
    segment.start_speed_mm_per_min = start_speed;
    segment.end_speed_mm_per_min = final_speed;
    if (s_source_block.request.spindle_mode == motion::SpindleMode::CounterClockwise) {
        const double actual_speed =
            (final_distance - start_distance) * 60000000.0 / std::max(1.0, duration_us);
        segment.laser_power = s_source_block.request.spindle_power * std::clamp(
            actual_speed / s_source_block.programmed_rate_mm_per_min, 0.0, 1.0);
    } else {
        segment.laser_power = s_source_block.request.spindle_power;
    }
    segment.phase = phase_at_distance(start_distance);
    segment.completes_block = segment_index + 1 == s_profile.total_segments;
    for (std::size_t axis = 0; axis < segment.axis_steps.size(); ++axis) {
        const std::int32_t axis_total = s_source_block.plan.steps[axis];
        const std::int32_t axis_first = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * first_step) / total_steps);
        const std::int32_t axis_final = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * final_step) / total_steps);
        segment.axis_steps[axis] = axis_final - axis_first;
    }
    return segment;
}

}  // namespace

void stepper_prep_reset() {
    portENTER_CRITICAL(&s_segment_lock);
    s_segment_head = 0;
    s_segment_tail = 0;
    s_segment_count = 0;
    s_profile = {};
    s_source_block = {};
    portEXIT_CRITICAL(&s_segment_lock);
}

bool stepper_prep_start_block(const planner::PlannerBlock& block) {
    if (!block_is_valid(block)) {
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

    portENTER_CRITICAL(&s_segment_lock);
    if (s_profile.active || s_segment_count != 0) {
        portEXIT_CRITICAL(&s_segment_lock);
        return false;
    }
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
    };
    portEXIT_CRITICAL(&s_segment_lock);
    return true;
}

std::size_t stepper_prep_fill() {
    portENTER_CRITICAL(&s_segment_lock);
    const std::size_t initial_count = s_segment_count;
    while (s_profile.active && s_segment_count < kStepSegmentBufferLength &&
           s_profile.next_segment_index < s_profile.total_segments) {
        s_segment_buffer[s_segment_head] = build_segment(s_profile.next_segment_index);
        s_segment_head = (s_segment_head + 1) % kStepSegmentBufferLength;
        ++s_segment_count;
        ++s_profile.next_segment_index;
    }
    if (s_profile.next_segment_index == s_profile.total_segments) {
        s_profile.active = false;
    }
    const std::size_t generated = s_segment_count - initial_count;
    portEXIT_CRITICAL(&s_segment_lock);
    return generated;
}

bool stepper_segment_pop(StepSegment& segment) {
    portENTER_CRITICAL(&s_segment_lock);
    if (s_segment_count == 0) {
        portEXIT_CRITICAL(&s_segment_lock);
        return false;
    }
    segment = s_segment_buffer[s_segment_tail];
    s_segment_tail = (s_segment_tail + 1) % kStepSegmentBufferLength;
    --s_segment_count;
    portEXIT_CRITICAL(&s_segment_lock);
    return true;
}

std::size_t stepper_segment_count() {
    portENTER_CRITICAL(&s_segment_lock);
    const std::size_t count = s_segment_count;
    portEXIT_CRITICAL(&s_segment_lock);
    return count;
}

const StepperProfile& stepper_prep_profile() {
    return s_profile;
}

}  // namespace ray2::stepper
