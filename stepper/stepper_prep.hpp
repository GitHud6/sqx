#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "planner.hpp"

namespace ray2::stepper {

constexpr std::size_t kStepSegmentBufferLength = 64;
constexpr std::size_t kStepCompletionBufferLength = 32;
constexpr std::uint32_t kStepSegmentTargetDurationUs = 750;
constexpr std::size_t kStepSegmentLowWater = 16;

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
    std::uint32_t duration_us = 0;
    std::uint32_t step_interval_us = 0;
    double start_speed_mm_per_min = 0.0;
    double end_speed_mm_per_min = 0.0;
    double laser_power = 0.0;
    motion::MotionType motion_mode = motion::MotionType::Rapid;
    motion::SpindleMode spindle_mode = motion::SpindleMode::Off;
    StepSegmentPhase phase = StepSegmentPhase::Cruise;
    bool completes_block = false;
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
    std::size_t pending_completions = 0;
};

struct StepperProfile {
    std::uint64_t execution_id = 0;
    std::int32_t total_major_steps = 0;
    std::uint32_t total_segments = 0;
    std::uint32_t next_segment_index = 0;
    double length_mm = 0.0;
    double entry_speed_mm_per_min = 0.0;
    double nominal_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double acceleration_mm_per_min2 = 0.0;
    double accelerate_distance_mm = 0.0;
    double decelerate_after_mm = 0.0;
    bool active = false;
};

void stepper_prep_reset();
bool stepper_prep_start_block(const planner::PlannerBlock& block);
void stepper_prep_cancel_block();
std::size_t stepper_prep_fill();
bool stepper_segment_publish(const StepSegment& segment);
bool IRAM_ATTR stepper_segment_consume(StepSegment& segment);
std::size_t stepper_segment_count();
std::size_t stepper_segment_free_slots();
void IRAM_ATTR stepper_segment_record_underflow_from_isr();
bool IRAM_ATTR stepper_segment_needs_refill_from_isr();
void IRAM_ATTR stepper_segment_request_refill_from_isr(
    BaseType_t* higher_priority_task_woken);
void stepper_segment_set_active(std::uint64_t execution_id);
void IRAM_ATTR stepper_segment_clear_active_from_isr();

void stepper_completion_set_notification_task(TaskHandle_t task);
bool IRAM_ATTR stepper_completion_publish_from_isr(const SegmentCompletion& completion,
                                                   BaseType_t* higher_priority_task_woken);
bool stepper_completion_consume(SegmentCompletion& completion);
SegmentExecutionDiagnostics stepper_segment_diagnostics();

const StepperProfile& stepper_prep_profile();

}  // namespace ray2::stepper
