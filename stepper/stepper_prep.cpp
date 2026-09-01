#include "stepper_prep.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

#include "raycube_machine.hpp"

namespace ray2::stepper {
namespace {

constexpr std::uint32_t kMinimumStepIntervalUs = 4;
constexpr std::uint32_t kMinimumStepIntervalTicks =
    kMinimumStepIntervalUs * kStepTimerTicksPerMicrosecond;

std::array<StepSegment, kStepSegmentBufferLength> s_segment_buffer = {};
std::atomic<std::uint32_t> s_segment_head = 0;
std::atomic<std::uint32_t> s_segment_tail = 0;
std::atomic<std::size_t> s_segment_high_water = 0;
std::atomic<std::uint64_t> s_segment_underflow_count = 0;
std::atomic<std::uint64_t> s_block_boundary_stop_count = 0;
std::atomic<std::uint64_t> s_block_boundary_restart_count = 0;
std::atomic<bool> s_block_boundary_restart_pending = false;
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

// Context slots own profile geometry until the executor confirms completion.
// Stage one intentionally allows only one occupied slot; later stages will
// permit the generation side to reserve later slots before execution catches up.
struct TrapezoidPrepContext {
    std::atomic<bool> occupied = false;
    StepperProfile profile = {};
    planner::PlannerBlock source_block = {};
    planner::PlannerBlock original_source_block = {};
    std::atomic<std::int32_t> executed_major_steps = 0;
    std::array<std::atomic<std::int32_t>, machine::kAxisCount> executed_axis_steps = {};
    std::atomic<std::uint32_t> current_speed_tenths_mm_per_min = 0;
    std::atomic<bool> generation_complete = false;
};

std::array<TrapezoidPrepContext, kTrapezoidPrepContextCapacity> s_context_fifo = {};
std::size_t s_context_head = 0;
std::size_t s_context_tail = 0;
std::atomic<std::size_t> s_context_count = 0;
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
static_assert(kTrapezoidPrepContextCapacity > 0, "prep context FIFO requires storage");

TrapezoidPrepContext& active_context() {
    return s_context_fifo[s_context_head];
}

StepperProfile& active_profile() {
    return active_context().profile;
}

planner::PlannerBlock& active_source_block() {
    return active_context().source_block;
}

planner::PlannerBlock& active_original_source_block() {
    return active_context().original_source_block;
}

void reset_context_fifo() {
    for (auto& context : s_context_fifo) {
        context.occupied.store(false, std::memory_order_relaxed);
        context.profile = {};
        context.source_block = {};
        context.original_source_block = {};
        context.executed_major_steps.store(0, std::memory_order_relaxed);
        for (auto& steps : context.executed_axis_steps) {
            steps.store(0, std::memory_order_relaxed);
        }
        context.current_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
        context.generation_complete.store(false, std::memory_order_relaxed);
    }
    s_context_head = 0;
    s_context_tail = 0;
    s_context_count.store(0, std::memory_order_relaxed);
}

TrapezoidPrepContext* reserve_context(const planner::PlannerBlock& block) {
    if (s_context_count.load(std::memory_order_acquire) == kTrapezoidPrepContextCapacity) {
        return nullptr;
    }
    TrapezoidPrepContext& context = s_context_fifo[s_context_tail];
    context.profile = {};
    context.source_block = block;
    context.original_source_block = block;
    context.executed_major_steps.store(0, std::memory_order_relaxed);
    for (auto& steps : context.executed_axis_steps) {
        steps.store(0, std::memory_order_relaxed);
    }
    context.current_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    context.generation_complete.store(false, std::memory_order_relaxed);
    context.occupied.store(true, std::memory_order_release);
    s_context_count.fetch_add(1, std::memory_order_release);
    s_context_tail = (s_context_tail + 1) % kTrapezoidPrepContextCapacity;
    return &context;
}

void release_active_context() {
    if (s_context_count.load(std::memory_order_acquire) == 0) return;
    TrapezoidPrepContext& context = active_context();
    context.occupied.store(false, std::memory_order_release);
    context.profile = {};
    context.source_block = {};
    context.original_source_block = {};
    s_context_head = (s_context_head + 1) % kTrapezoidPrepContextCapacity;
    s_context_count.fetch_sub(1, std::memory_order_release);
}

void sync_active_context_state() {
    if (s_context_count.load(std::memory_order_acquire) == 0) {
        s_context_execution_id.store(0, std::memory_order_release);
        s_context_total_major_steps.store(0, std::memory_order_relaxed);
        s_context_executed_major_steps.store(0, std::memory_order_relaxed);
        s_context_current_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
        s_context_generation_complete.store(false, std::memory_order_release);
        return;
    }
    const TrapezoidPrepContext& context = active_context();
    s_context_execution_id.store(context.profile.execution_id, std::memory_order_release);
    s_context_total_major_steps.store(context.profile.total_major_steps, std::memory_order_relaxed);
    s_context_executed_major_steps.store(
        context.executed_major_steps.load(std::memory_order_relaxed), std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        s_context_executed_axis_steps[axis].store(
            context.executed_axis_steps[axis].load(std::memory_order_relaxed),
            std::memory_order_relaxed);
    }
    s_context_current_speed_tenths_mm_per_min.store(
        context.current_speed_tenths_mm_per_min.load(std::memory_order_relaxed),
        std::memory_order_relaxed);
    s_context_generation_complete.store(
        context.generation_complete.load(std::memory_order_relaxed), std::memory_order_release);
}

TrapezoidPrepContext* generation_context() {
    const std::size_t count = s_context_count.load(std::memory_order_acquire);
    for (std::size_t offset = 0; offset < count; ++offset) {
        TrapezoidPrepContext& context =
            s_context_fifo[(s_context_head + offset) % kTrapezoidPrepContextCapacity];
        if (context.occupied.load(std::memory_order_acquire) && context.profile.active &&
            !context.generation_complete.load(std::memory_order_acquire)) {
            return &context;
        }
    }
    return nullptr;
}

TrapezoidPrepContext* IRAM_ATTR context_for_execution_id(const std::uint64_t execution_id) {
    if (execution_id == 0) return nullptr;
    for (auto& context : s_context_fifo) {
        if (context.occupied.load(std::memory_order_acquire) &&
            context.profile.execution_id == execution_id) {
            return &context;
        }
    }
    return nullptr;
}

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

bool apply_trapezoid_profile(const MotionProfileInput& input, StepperProfile& profile) {
    MotionProfile motion_profile = {};
    if (!motion_profile_build_trapezoid(input, motion_profile)) return false;
    profile.motion_profile = motion_profile;
    profile.length_mm = motion_profile.length_mm;
    profile.entry_speed_mm_per_min = motion_profile.entry_speed_mm_per_min;
    profile.nominal_speed_mm_per_min = motion_profile.nominal_speed_mm_per_min;
    profile.exit_speed_mm_per_min = motion_profile.exit_speed_mm_per_min;
    profile.acceleration_mm_per_min2 = motion_profile.acceleration_mm_per_min2;
    profile.accelerate_distance_mm = motion_profile.accelerate_distance_mm;
    profile.decelerate_after_mm = motion_profile.decelerate_after_mm;
    return true;
}

StepSegmentPhase phase_at_distance(const StepperProfile& profile, const double distance_mm) {
    switch (motion_profile_phase_at_distance(profile.motion_profile, distance_mm)) {
        case MotionProfilePhase::Accelerate: return StepSegmentPhase::Accelerate;
        case MotionProfilePhase::Cruise: return StepSegmentPhase::Cruise;
        case MotionProfilePhase::Decelerate: return StepSegmentPhase::Decelerate;
    }
    return StepSegmentPhase::Cruise;
}

StepSegment build_segment(const TrapezoidPrepContext& context,
                          const std::uint32_t segment_index) {
    const StepperProfile& profile = context.profile;
    const planner::PlannerBlock& source_block = context.source_block;
    const std::int32_t total_steps = profile.total_major_steps;
    const std::int32_t first_step = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(total_steps) * segment_index) / profile.total_segments);
    const std::int32_t final_step = static_cast<std::int32_t>(
        (static_cast<std::int64_t>(total_steps) * (segment_index + 1)) / profile.total_segments);
    const double start_distance = profile.length_mm * first_step / total_steps;
    const double final_distance = profile.length_mm * final_step / total_steps;
    const double start_speed = motion_profile_speed_at_distance(profile.motion_profile, start_distance);
    const double final_speed = motion_profile_speed_at_distance(profile.motion_profile, final_distance);
    const double duration_us = motion_profile_time_at_distance_us(profile.motion_profile, final_distance) -
                               motion_profile_time_at_distance_us(profile.motion_profile, start_distance);

    StepSegment segment = {};
    segment.execution_id = profile.execution_id;
    segment.major_steps = final_step - first_step;
    segment.block_major_steps = total_steps;
    segment.duration_us = static_cast<std::uint32_t>(std::max(1.0, std::round(duration_us)));
    segment.step_interval_ticks = std::max(
        kMinimumStepIntervalTicks,
        static_cast<std::uint32_t>(std::max(
            1.0, std::round(duration_us * kStepTimerTicksPerMicrosecond / segment.major_steps))));
    segment.step_interval_us = static_cast<std::uint32_t>(std::max(
        1.0, std::round(static_cast<double>(segment.step_interval_ticks) /
                            kStepTimerTicksPerMicrosecond)));
    segment.start_speed_mm_per_min = start_speed;
    segment.end_speed_mm_per_min = final_speed;
    segment.start_speed_tenths_mm_per_min = speed_to_tenths_mm_per_min(start_speed);
    segment.end_speed_tenths_mm_per_min = speed_to_tenths_mm_per_min(final_speed);
    const bool laser_enabled =
        source_block.request.motion_mode == motion::MotionType::Linear &&
        source_block.request.spindle_mode != motion::SpindleMode::Off;
    if (laser_enabled &&
        source_block.request.spindle_mode == motion::SpindleMode::CounterClockwise) {
        const double actual_speed =
            (final_distance - start_distance) * 60000000.0 / std::max(1.0, duration_us);
        segment.laser_power = source_block.request.spindle_power * std::clamp(
            actual_speed / source_block.programmed_rate_mm_per_min, 0.0, 1.0);
    } else if (laser_enabled) {
        segment.laser_power = source_block.request.spindle_power;
    }
    const double max_power = static_cast<double>(machine::raycube_laser().max_power);
    segment.laser_power_per_mille = static_cast<std::uint16_t>(std::clamp(
        std::lround(segment.laser_power * 1000.0 / max_power), 0L, 1000L));
    segment.motion_mode = source_block.request.motion_mode;
    segment.spindle_mode = source_block.request.spindle_mode;
    segment.phase = phase_at_distance(profile, start_distance);
    const bool finishes_profile = segment_index + 1 == profile.total_segments;
    segment.completes_block = finishes_profile && profile.completes_source_block;
    segment.feed_hold = profile.feed_hold;
    segment.ends_feed_hold = profile.ends_feed_hold && finishes_profile;
    if (segment.feed_hold) {
        segment.laser_power = 0.0;
        segment.laser_power_per_mille = 0;
    }
    for (std::size_t axis = 0; axis < segment.axis_steps.size(); ++axis) {
        const std::int32_t axis_total = source_block.plan.steps[axis];
        const std::int32_t axis_first = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * first_step) / total_steps);
        const std::int32_t axis_final = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(axis_total) * final_step) / total_steps);
        segment.axis_steps[axis] = axis_final - axis_first;
        segment.block_axis_steps[axis] = axis_total;
        segment.positive_direction[axis] = source_block.plan.positive_direction[axis];
    }
    // AMASS is selected from the source block, not this short segment. A valid
    // diagonal may contain a segment with zero minor-axis steps after integer
    // partitioning, but it still belongs to a multi-axis Bresenham stream.
    const std::array<std::int32_t, machine::kAxisCount> source_axis_steps = {
        source_block.plan.steps[0],
        source_block.plan.steps[1],
    };
    segment.amass_level = stepper_select_amass_level(segment.step_interval_ticks, source_axis_steps);
    return segment;
}

}  // namespace

std::uint8_t stepper_select_amass_level(
    const std::uint32_t step_interval_ticks,
    const std::array<std::int32_t, 2>& axis_steps) {
    if (step_interval_ticks == 0 || axis_steps[0] == 0 || axis_steps[1] == 0) return 0;
    constexpr std::uint32_t kAmassThresholdTicks = kStepTimerFrequencyHz / kAmassBaseStepRateHz;
    std::uint32_t interval = step_interval_ticks;
    std::uint8_t level = 0;
    while (level < kAmassMaximumLevel && interval >= kAmassThresholdTicks) {
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
    s_block_boundary_stop_count.store(0, std::memory_order_relaxed);
    s_block_boundary_restart_count.store(0, std::memory_order_relaxed);
    s_block_boundary_restart_pending.store(false, std::memory_order_relaxed);
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
    reset_context_fifo();
}

void stepper_prep_abort() {
    // Keep the fault path explicit: no stale segment or completion may cross
    // an execution-generation boundary.
    stepper_prep_reset();
}

bool stepper_prep_start_block(const planner::PlannerBlock& block) {
    if (!block_is_valid(block)) {
        return false;
    }
    TrapezoidPrepContext* context = reserve_context(block);
    if (context == nullptr) return false;

    const double entry_speed = std::sqrt(std::max(0.0, block.entry_speed_sqr));
    const double exit_speed = std::sqrt(std::max(0.0, block.exit_speed_sqr));
    StepperProfile profile = {};
    if (!apply_trapezoid_profile({
                                      .length_mm = block.millimeters,
                                      .entry_speed_mm_per_min = entry_speed,
                                      .nominal_speed_mm_per_min = block.programmed_rate_mm_per_min,
                                      .exit_speed_mm_per_min = exit_speed,
                                      .acceleration_mm_per_min2 = block.acceleration_mm_per_sec2 * 60.0 * 60.0,
                                  }, profile)) {
        context->occupied.store(false, std::memory_order_release);
        s_context_count.fetch_sub(1, std::memory_order_release);
        return false;
    }
    const std::uint32_t target_segments = static_cast<std::uint32_t>(std::max(
        1.0, std::ceil(profile.motion_profile.total_time_us / kStepSegmentTargetDurationUs)));

    context->profile = {
        .execution_id = block.execution_id,
        .total_major_steps = block.step_event_count,
        .total_segments = std::min(target_segments, static_cast<std::uint32_t>(block.step_event_count)),
        .next_segment_index = 0,
        .length_mm = profile.length_mm,
        .entry_speed_mm_per_min = profile.entry_speed_mm_per_min,
        .nominal_speed_mm_per_min = profile.nominal_speed_mm_per_min,
        .exit_speed_mm_per_min = profile.exit_speed_mm_per_min,
        .acceleration_mm_per_min2 = profile.acceleration_mm_per_min2,
        .accelerate_distance_mm = profile.accelerate_distance_mm,
        .decelerate_after_mm = profile.decelerate_after_mm,
        .motion_profile = profile.motion_profile,
        .active = true,
        .generation_complete = false,
        .feed_hold = false,
        .ends_feed_hold = false,
        .completes_source_block = true,
    };
    context->current_speed_tenths_mm_per_min.store(
        speed_to_tenths_mm_per_min(entry_speed), std::memory_order_relaxed);
    if (context == &active_context()) {
        s_context_total_major_steps.store(block.step_event_count, std::memory_order_relaxed);
        s_context_executed_major_steps.store(0, std::memory_order_relaxed);
        for (auto& steps : s_context_executed_axis_steps) {
            steps.store(0, std::memory_order_relaxed);
        }
        s_context_current_speed_tenths_mm_per_min.store(
            speed_to_tenths_mm_per_min(entry_speed), std::memory_order_relaxed);
        s_context_generation_complete.store(false, std::memory_order_relaxed);
        s_context_execution_id.store(block.execution_id, std::memory_order_release);
    }
    return true;
}

void stepper_prep_cancel_block() {
    release_active_context();
    sync_active_context_state();
    s_context_hold_boundary.store(false, std::memory_order_relaxed);
}

std::size_t stepper_prep_fill() {
    std::size_t generated = 0;
    TrapezoidPrepContext* context = generation_context();
    if (context == nullptr) return 0;
    StepperProfile& profile = context->profile;
    while (profile.active && profile.next_segment_index < profile.total_segments) {
        const StepSegment segment = build_segment(*context, profile.next_segment_index);
        if (!stepper_segment_publish(segment)) {
            break;
        }
        ++profile.next_segment_index;
        ++generated;
    }
    if (profile.next_segment_index == profile.total_segments) {
        // The final segment being published does not mean it has executed.
        profile.generation_complete = true;
        context->generation_complete.store(true, std::memory_order_release);
        if (context == &active_context()) {
            s_context_generation_complete.store(true, std::memory_order_release);
        }
    }
    return generated;
}

bool stepper_prep_complete_block(const std::uint64_t execution_id) {
    TrapezoidPrepContext* context = context_for_execution_id(execution_id);
    if (context == nullptr || context != &active_context() || !context->profile.active ||
        !context->generation_complete.load(std::memory_order_acquire) ||
        context->executed_major_steps.load(std::memory_order_acquire) !=
            context->profile.total_major_steps) {
        return false;
    }
    const StepperProfile& profile = context->profile;
    const bool hold_ended = profile.feed_hold && profile.ends_feed_hold;
    const bool continue_hold = profile.feed_hold && !profile.ends_feed_hold;
    const std::uint32_t carry_speed =
        context->current_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
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
    StepperProfile& active = active_profile();
    if (!active.active) return false;
    const std::int32_t total_steps = s_context_total_major_steps.load(std::memory_order_acquire);
    const std::int32_t executed_steps = s_context_executed_major_steps.load(std::memory_order_acquire);
    const std::int32_t remaining_steps = total_steps - executed_steps;
    const double speed = static_cast<double>(
        s_context_current_speed_tenths_mm_per_min.load(std::memory_order_acquire)) / 10.0;
    if (remaining_steps <= 0 || speed <= 0.0) {
        s_context_hold_complete.store(true, std::memory_order_release);
        return false;
    }
    const double remaining_mm = active.length_mm * remaining_steps / active.total_major_steps;
    const double acceleration = active.acceleration_mm_per_min2;
    const double stop_mm = speed * speed / (2.0 * acceleration);
    const bool stops_here = stop_mm <= remaining_mm;
    const double travel_mm = stops_here ? stop_mm : remaining_mm;
    const std::int32_t travel_steps = std::clamp<std::int32_t>(
        static_cast<std::int32_t>(std::ceil(remaining_steps * travel_mm / remaining_mm)), 1, remaining_steps);

    planner::PlannerBlock residual = active_source_block();
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const std::int32_t remaining_axis = std::max<std::int32_t>(
            0, active_source_block().plan.steps[axis] -
                   s_context_executed_axis_steps[axis].load(std::memory_order_acquire));
        residual.plan.steps[axis] = static_cast<std::int32_t>(
            (static_cast<std::int64_t>(remaining_axis) * travel_steps) / remaining_steps);
    }
    residual.step_event_count = travel_steps;
    residual.millimeters = remaining_mm * travel_steps / remaining_steps;
    residual.programmed_rate_mm_per_min = speed;
    active_source_block() = residual;
    const double exit_speed = stops_here ? 0.0 :
        std::sqrt(std::max(0.0, speed * speed - 2.0 * acceleration * residual.millimeters));
    StepperProfile profile = {};
    if (!apply_trapezoid_profile({
                                      .length_mm = residual.millimeters,
                                      .entry_speed_mm_per_min = speed,
                                      .nominal_speed_mm_per_min = speed,
                                      .exit_speed_mm_per_min = exit_speed,
                                      .acceleration_mm_per_min2 = acceleration,
                                  }, profile)) {
        return false;
    }
    active_profile() = {
        .execution_id = residual.execution_id,
        .total_major_steps = travel_steps,
        .total_segments = std::min<std::uint32_t>(
            static_cast<std::uint32_t>(travel_steps),
            static_cast<std::uint32_t>(std::max(1.0, std::ceil(profile.motion_profile.total_time_us / kStepSegmentTargetDurationUs)))),
        .next_segment_index = 0,
        .length_mm = profile.length_mm,
        .entry_speed_mm_per_min = profile.entry_speed_mm_per_min,
        .nominal_speed_mm_per_min = profile.nominal_speed_mm_per_min,
        .exit_speed_mm_per_min = profile.exit_speed_mm_per_min,
        .acceleration_mm_per_min2 = profile.acceleration_mm_per_min2,
        .accelerate_distance_mm = profile.accelerate_distance_mm,
        .decelerate_after_mm = profile.decelerate_after_mm,
        .motion_profile = profile.motion_profile,
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

bool stepper_prep_resume_from_hold() {
    if (!stepper_prep_hold_complete() || !active_profile().active ||
        s_context_execution_id.load(std::memory_order_acquire) == 0) {
        return false;
    }
    const std::int32_t total_steps = s_context_total_major_steps.load(std::memory_order_acquire);
    const std::int32_t executed_steps = s_context_executed_major_steps.load(std::memory_order_acquire);
    const std::int32_t remaining_steps = total_steps - executed_steps;
    if (!block_is_valid(active_original_source_block()) || remaining_steps <= 0) {
        return false;
    }

    planner::PlannerBlock residual = active_original_source_block();
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        residual.plan.steps[axis] = std::max<std::int32_t>(
            0, active_original_source_block().plan.steps[axis] -
                   s_context_executed_axis_steps[axis].load(std::memory_order_acquire));
    }
    residual.step_event_count = remaining_steps;
    residual.millimeters = active_original_source_block().millimeters * remaining_steps /
                           active_original_source_block().step_event_count;
    const double entry_speed = 0.0;
    const double exit_speed = std::sqrt(std::max(0.0, residual.exit_speed_sqr));
    StepperProfile profile = {};
    if (!apply_trapezoid_profile({
                                      .length_mm = residual.millimeters,
                                      .entry_speed_mm_per_min = entry_speed,
                                      .nominal_speed_mm_per_min = residual.programmed_rate_mm_per_min,
                                      .exit_speed_mm_per_min = exit_speed,
                                      .acceleration_mm_per_min2 = residual.acceleration_mm_per_sec2 * 60.0 * 60.0,
                                  }, profile)) {
        return false;
    }
    active_source_block() = residual;
    active_profile() = {
        .execution_id = residual.execution_id,
        .total_major_steps = remaining_steps,
        .total_segments = std::min<std::uint32_t>(
            static_cast<std::uint32_t>(remaining_steps),
            static_cast<std::uint32_t>(std::max(1.0, std::ceil(profile.motion_profile.total_time_us / kStepSegmentTargetDurationUs)))),
        .next_segment_index = 0,
        .length_mm = profile.length_mm,
        .entry_speed_mm_per_min = profile.entry_speed_mm_per_min,
        .nominal_speed_mm_per_min = profile.nominal_speed_mm_per_min,
        .exit_speed_mm_per_min = profile.exit_speed_mm_per_min,
        .acceleration_mm_per_min2 = profile.acceleration_mm_per_min2,
        .accelerate_distance_mm = profile.accelerate_distance_mm,
        .decelerate_after_mm = profile.decelerate_after_mm,
        .motion_profile = profile.motion_profile,
        .active = true,
        .generation_complete = false,
        .feed_hold = false,
        .ends_feed_hold = false,
        .completes_source_block = true,
    };
    s_context_current_speed_tenths_mm_per_min.store(0, std::memory_order_release);
    s_context_generation_complete.store(false, std::memory_order_release);
    s_context_hold_boundary.store(false, std::memory_order_release);
    s_context_feed_hold_active.store(false, std::memory_order_release);
    s_context_hold_complete.store(false, std::memory_order_release);
    return true;
}

void IRAM_ATTR stepper_prep_record_segment_completed_from_isr(const StepSegment& segment) {
    TrapezoidPrepContext* context = context_for_execution_id(segment.execution_id);
    if (context == nullptr) return;
    context->executed_major_steps.fetch_add(segment.major_steps, std::memory_order_relaxed);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        context->executed_axis_steps[axis].fetch_add(segment.axis_steps[axis],
                                                      std::memory_order_relaxed);
    }
    context->current_speed_tenths_mm_per_min.store(
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

void IRAM_ATTR stepper_segment_record_block_boundary_stop_from_isr() {
    s_block_boundary_stop_count.fetch_add(1, std::memory_order_relaxed);
    s_block_boundary_restart_pending.store(true, std::memory_order_release);
}

void stepper_segment_record_block_boundary_restart() {
    if (s_block_boundary_restart_pending.exchange(false, std::memory_order_acq_rel)) {
        s_block_boundary_restart_count.fetch_add(1, std::memory_order_relaxed);
    }
}

void stepper_segment_clear_block_boundary_pending() {
    s_block_boundary_restart_pending.store(false, std::memory_order_release);
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
    diagnostics.block_boundary_stop_count =
        s_block_boundary_stop_count.load(std::memory_order_relaxed);
    diagnostics.block_boundary_restart_count =
        s_block_boundary_restart_count.load(std::memory_order_relaxed);
    diagnostics.block_boundary_restart_pending =
        s_block_boundary_restart_pending.load(std::memory_order_acquire);
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
    return active_profile();
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
    diagnostics.profile_mode = active_profile().motion_profile.mode;
    diagnostics.context_used = s_context_count.load(std::memory_order_acquire);
    if (diagnostics.context_used != 0) {
        diagnostics.generation_context_execution_id = active_profile().execution_id;
        diagnostics.completion_context_execution_id = diagnostics.generation_complete
                                                          ? diagnostics.generation_context_execution_id
                                                          : 0;
    }
    diagnostics.execution_context_execution_id =
        s_active_execution_id.load(std::memory_order_acquire);
    return diagnostics;
}

}  // namespace ray2::stepper
