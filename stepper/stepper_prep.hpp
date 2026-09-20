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

namespace laserctl::stepper {

constexpr std::size_t kTrapezoidPrepContextCapacity = 4;
// Match the classic Grbl acceleration update cadence used by legacy-upstream.
constexpr std::uint32_t kStepSegmentTargetDurationUs = 1000;
constexpr std::uint32_t kStepTimerFrequencyHz = 20000000;
constexpr std::uint32_t kStepTimerTicksPerMicrosecond = kStepTimerFrequencyHz / 1000000U;
constexpr std::uint32_t kAmassBaseStepRateHz = 8000;
constexpr std::uint8_t kAmassMaximumLevel = 3;

struct StepperProfile {
    TrapezoidExecutionPlan execution_plan = {};
    bool active = false;
    bool generation_complete = false;
    bool completes_source_block = true;
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
bool stepper_prep_record_segment_completed(const SegmentCompletion& completion);
// Pure AMASS selector. It has no dependency on GPIO, RMT, FreeRTOS, or motion state.
std::uint8_t stepper_select_amass_level(std::uint32_t step_interval_ticks,
                                        const std::array<std::int32_t, 2>& axis_steps);
const StepperProfile& stepper_prep_profile();
bool stepper_prep_can_accept_block();
bool stepper_prep_has_active_context();

}  // namespace laserctl::stepper
