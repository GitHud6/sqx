#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "planner.hpp"
#include "segment_ring.hpp"
#include "trapezoid_execution_plan.hpp"

namespace ray2::stepper {

constexpr std::size_t kTrapezoidPrepContextCapacity = 4;
// Match the classic Grbl acceleration update cadence used by ray2-main.
constexpr std::uint32_t kStepSegmentTargetDurationUs = 1000;
constexpr std::uint32_t kStepTimerFrequencyHz = 20000000;
constexpr std::uint32_t kStepTimerTicksPerMicrosecond = kStepTimerFrequencyHz / 1000000U;
constexpr std::uint32_t kAmassBaseStepRateHz = 8000;
constexpr std::uint8_t kAmassMaximumLevel = 3;

struct StepperProfile {
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
// Generates one complete segment from the active immutable execution plan.
// It does not access the segment ring or start the timer.
bool stepper_prep_next_segment(StepSegment& segment);
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
const StepperProfile& stepper_prep_profile();
StepperPrepDiagnostics stepper_prep_diagnostics();

}  // namespace ray2::stepper
