#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motion_request.hpp"

namespace laserctl::stepper {

constexpr std::size_t kStepSegmentBufferLength = 64;
constexpr std::size_t kStepCompletionBufferLength = 32;
constexpr std::size_t kStepSegmentLowWater = 16;

enum class StepSegmentPhase : std::uint8_t { Accelerate, Cruise, Decelerate };

struct StepSegment {
    std::uint64_t execution_id = 0;
    std::array<std::int32_t, 2> axis_steps = {};
    std::array<bool, 2> positive_direction = {};
    std::int32_t major_steps = 0;
    // Required by the Timer DDA to preserve the source block phase across
    // segment boundaries. These are not planner or profile state.
    std::array<std::int32_t, 2> block_axis_steps = {};
    std::int32_t block_major_steps = 0;
    std::uint32_t duration_us = 0;
    std::uint32_t step_interval_us = 0;
    std::uint32_t step_interval_ticks = 0;
    std::uint8_t amass_level = 0;
    double start_speed_mm_per_min = 0.0;
    double end_speed_mm_per_min = 0.0;
    std::uint32_t start_speed_tenths_mm_per_min = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    double laser_power = 0.0;
    std::uint16_t laser_power_per_mille = 0;
    motion::MotionType motion_mode = motion::MotionType::Rapid;
    motion::SpindleMode spindle_mode = motion::SpindleMode::Off;
    StepSegmentPhase phase = StepSegmentPhase::Cruise;
    bool completes_block = false;
    bool system_motion = false;
};

enum class SegmentCompletionStatus : std::uint8_t { Completed, Faulted };
struct SegmentCompletion {
    std::uint64_t execution_id = 0;
    SegmentCompletionStatus status = SegmentCompletionStatus::Completed;
    std::array<std::int32_t, 2> axis_steps = {};
    std::int32_t major_steps = 0;
    std::uint32_t end_speed_tenths_mm_per_min = 0;
    bool completes_block = false;
};

enum StepperTransportEvent : std::uint32_t {
    StepperTransportEventNone = 0,
    StepperTransportEventRefillRequested = 1U << 0U,
    StepperTransportEventCompletionAvailable = 1U << 1U,
    StepperTransportEventHoldReached = 1U << 2U,
};

struct SegmentExecutionDiagnostics {
    std::size_t queued_segments = 0;
    std::size_t segment_capacity = kStepSegmentBufferLength;
    bool active = false;
    std::uint64_t active_execution_id = 0;
    std::uint32_t active_step_interval_us = 0;
    std::uint32_t active_amass_timer_interval_us = 0;
    std::uint8_t active_amass_level = 0;
    std::uint32_t active_start_speed_tenths_mm_per_min = 0;
    std::uint32_t active_end_speed_tenths_mm_per_min = 0;
    std::size_t pending_completions = 0;
    bool block_boundary_restart_pending = false;
};

void stepper_segment_ring_reset();
bool stepper_segment_publish(const StepSegment& segment);
bool IRAM_ATTR stepper_segment_consume(StepSegment& segment);
void stepper_segment_discard_pending();
std::size_t stepper_segment_count();
std::size_t stepper_segment_free_slots();
void IRAM_ATTR stepper_segment_record_block_boundary_stop_from_isr();
void stepper_segment_record_block_boundary_restart();
void stepper_segment_clear_block_boundary_pending();
bool IRAM_ATTR stepper_segment_needs_refill_from_isr();
void IRAM_ATTR stepper_segment_request_refill_from_isr(BaseType_t* higher_priority_task_woken);
// Task-context refill request, used when a held executor resumes.
void stepper_segment_request_refill();
void IRAM_ATTR stepper_segment_record_hold_reached_from_isr(
    BaseType_t* higher_priority_task_woken);
void IRAM_ATTR stepper_segment_set_active(const StepSegment& segment);
void IRAM_ATTR stepper_segment_clear_active_from_isr();

void stepper_completion_set_notification_task(TaskHandle_t task);
std::uint32_t stepper_transport_take_events();
bool IRAM_ATTR stepper_completion_publish_from_isr(const SegmentCompletion& completion,
                                                   BaseType_t* higher_priority_task_woken);
bool stepper_completion_consume(SegmentCompletion& completion);
SegmentExecutionDiagnostics stepper_segment_diagnostics();

}  // namespace laserctl::stepper
