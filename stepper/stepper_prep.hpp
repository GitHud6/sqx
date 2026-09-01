#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "planner.hpp"
#include "trapezoid_execution_plan.hpp"

namespace ray2::stepper {

constexpr std::size_t kStepSegmentBufferLength = 64;
constexpr std::size_t kStepCompletionBufferLength = 32;
constexpr std::size_t kTrapezoidPrepContextCapacity = 4;
// Match the classic Grbl acceleration update cadence used by ray2-main.
constexpr std::uint32_t kStepSegmentTargetDurationUs = 1000;
constexpr std::size_t kStepSegmentLowWater = 16;
constexpr std::uint32_t kStepTimerFrequencyHz = 20000000;
constexpr std::uint32_t kStepTimerTicksPerMicrosecond = kStepTimerFrequencyHz / 1000000U;
constexpr std::uint32_t kAmassBaseStepRateHz = 8000;
constexpr std::uint8_t kAmassMaximumLevel = 3;

enum class StepSegmentPhase : std::uint8_t {
    Accelerate,
    Cruise,
    Decelerate,
};

struct StepSegment {
    std::uint64_t execution_id = 0;
    std::array<std::int32_t, 2> axis_steps = {};
    std::array<bool, 2> positive_direction = {};
    std::int32_t major_steps = 0;
    // Profile segments retain the source block geometry so the executor can
    // preserve Bresenham phase across segment boundaries. System motions keep
    // these zero and use their local DDA state.
    std::array<std::int32_t, 2> block_axis_steps = {};
    std::int32_t block_major_steps = 0;
    std::uint32_t duration_us = 0;
    std::uint32_t step_interval_us = 0;
    std::uint32_t step_interval_ticks = 0;
    std::uint8_t amass_level = 0;
    double start_speed_mm_per_min = 0.0;
    double end_speed_mm_per_min = 0.0;
    // Prepared outside the ISR for status reporting while this segment is active.
    std::uint32_t start_speed_tenths_mm_per_min = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    double laser_power = 0.0;
    std::uint16_t laser_power_per_mille = 0;
    motion::MotionType motion_mode = motion::MotionType::Rapid;
    motion::SpindleMode spindle_mode = motion::SpindleMode::Off;
    StepSegmentPhase phase = StepSegmentPhase::Cruise;
    bool completes_block = false;
    bool feed_hold = false;
    bool ends_feed_hold = false;
    // System motions use the timer/RMT pulse backend without planner completion.
    bool system_motion = false;
};

enum class SegmentCompletionStatus : std::uint8_t {
    Completed,
    Faulted,
};

struct SegmentCompletion {
    std::uint64_t execution_id = 0;
    SegmentCompletionStatus status = SegmentCompletionStatus::Completed;
};

struct SegmentExecutionDiagnostics {
    std::size_t queued_segments = 0;
    std::size_t segment_capacity = kStepSegmentBufferLength;
    std::size_t high_water_segments = 0;
    std::uint64_t underflow_count = 0;
    bool active = false;
    std::uint64_t active_execution_id = 0;
    std::uint32_t active_step_interval_us = 0;
    std::uint32_t active_amass_timer_interval_us = 0;
    std::uint8_t active_amass_level = 0;
    std::uint32_t active_start_speed_tenths_mm_per_min = 0;
    std::uint32_t active_end_speed_tenths_mm_per_min = 0;
    std::size_t pending_completions = 0;
    // A completed source block with no ready successor stops the timer cleanly.
    // This is distinct from a segment underflow inside a block.
    std::uint64_t block_boundary_stop_count = 0;
    std::uint64_t block_boundary_restart_count = 0;
    bool block_boundary_restart_pending = false;
};

struct StepperProfile {
    std::uint32_t next_segment_index = 0;
    TrapezoidExecutionPlan execution_plan = {};
    bool active = false;
    bool generation_complete = false;
    bool feed_hold = false;
    bool ends_feed_hold = false;
    bool completes_source_block = true;
};

// Segment entries are disposable cache. This persistent context is retained
// until the executor confirms that the entire source block was emitted.
struct StepperPrepDiagnostics {
    bool active = false;
    bool generation_complete = false;
    bool hold_boundary = false;
    std::uint64_t execution_id = 0;
    std::int32_t total_major_steps = 0;
    std::int32_t executed_major_steps = 0;
    std::array<std::int32_t, 2> executed_axis_steps = {};
    std::uint32_t current_speed_tenths_mm_per_min = 0;
    MotionProfileMode profile_mode = MotionProfileMode::Trapezoid;
    std::size_t context_used = 0;
    std::size_t context_capacity = kTrapezoidPrepContextCapacity;
    std::uint64_t generation_context_execution_id = 0;
    std::uint64_t execution_context_execution_id = 0;
    std::uint64_t completion_context_execution_id = 0;
};

void stepper_prep_reset();
// Drops all prepared contexts, segments, and completion notifications after
// an execution fault. The timer consumer must already be stopped.
void stepper_prep_abort();
bool stepper_prep_start_block(const planner::PlannerBlock& block);
void stepper_prep_cancel_block();
std::size_t stepper_prep_fill();
bool stepper_prep_complete_block(std::uint64_t execution_id);
bool stepper_prep_begin_feed_hold();
bool stepper_prep_continue_feed_hold(const planner::PlannerBlock& block);
bool stepper_prep_hold_boundary_pending();
bool stepper_prep_hold_complete();
bool stepper_prep_resume_from_hold();
void IRAM_ATTR stepper_prep_record_segment_completed_from_isr(const StepSegment& segment);
void IRAM_ATTR stepper_prep_record_hold_boundary_from_isr();
void IRAM_ATTR stepper_prep_record_hold_complete_from_isr();
void stepper_prep_clear_hold_boundary();
// Pure AMASS selector. It has no dependency on GPIO, RMT, FreeRTOS, or motion state.
std::uint8_t stepper_select_amass_level(std::uint32_t step_interval_ticks,
                                        const std::array<std::int32_t, 2>& axis_steps);
bool stepper_segment_publish(const StepSegment& segment);
bool IRAM_ATTR stepper_segment_consume(StepSegment& segment);
void stepper_segment_discard_pending();
std::size_t stepper_segment_count();
std::size_t stepper_segment_free_slots();
void IRAM_ATTR stepper_segment_record_underflow_from_isr();
void IRAM_ATTR stepper_segment_record_block_boundary_stop_from_isr();
void stepper_segment_record_block_boundary_restart();
void stepper_segment_clear_block_boundary_pending();
bool IRAM_ATTR stepper_segment_needs_refill_from_isr();
void IRAM_ATTR stepper_segment_request_refill_from_isr(
    BaseType_t* higher_priority_task_woken);
void IRAM_ATTR stepper_segment_set_active(const StepSegment& segment);
void IRAM_ATTR stepper_segment_clear_active_from_isr();

void stepper_completion_set_notification_task(TaskHandle_t task);
bool IRAM_ATTR stepper_completion_publish_from_isr(const SegmentCompletion& completion,
                                                   BaseType_t* higher_priority_task_woken);
bool stepper_completion_consume(SegmentCompletion& completion);
SegmentExecutionDiagnostics stepper_segment_diagnostics();

const StepperProfile& stepper_prep_profile();
StepperPrepDiagnostics stepper_prep_diagnostics();

}  // namespace ray2::stepper
