#pragma once

#include <cstdint>

#include "motion_profile.hpp"
#include "trapezoid_segment_generator.hpp"

namespace ray2::stepper {

struct TrapezoidExecutionPlanInput {
    std::uint64_t execution_id = 0;
    std::uint32_t total_major_steps = 0;
    MotionProfileInput profile_input = {};
    std::uint32_t segment_duration_us = 0;
    std::uint32_t timer_ticks_per_us = 0;
};

// Immutable profile boundaries plus the producer-owned time-slice cursor.
// It has no dependency on planner queues, GPIO, RMT, or FreeRTOS.
struct TrapezoidExecutionPlan {
    std::uint64_t execution_id = 0;
    std::uint32_t total_major_steps = 0;
    MotionProfile profile = {};
    TrapezoidSegmentGenerator generator = {};
};

bool trapezoid_execution_plan_build(const TrapezoidExecutionPlanInput& input,
                                    TrapezoidExecutionPlan& plan);
bool trapezoid_execution_plan_next(TrapezoidExecutionPlan& plan,
                                   TrapezoidTimeSlice& slice);
bool trapezoid_execution_plan_complete(const TrapezoidExecutionPlan& plan);

}  // namespace ray2::stepper
