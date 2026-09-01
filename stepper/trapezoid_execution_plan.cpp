#include "trapezoid_execution_plan.hpp"

namespace ray2::stepper {

bool trapezoid_execution_plan_build(const TrapezoidExecutionPlanInput& input,
                                    TrapezoidExecutionPlan& plan) {
    if (input.execution_id == 0 || input.total_major_steps == 0 ||
        input.segment_duration_us == 0 || input.timer_ticks_per_us == 0) {
        return false;
    }

    MotionProfile profile = {};
    if (!motion_profile_build_trapezoid(input.profile_input, profile)) return false;

    TrapezoidSegmentGenerator generator = {};
    if (!trapezoid_segment_generator_init(generator, profile, input.total_major_steps,
                                          input.segment_duration_us,
                                          input.timer_ticks_per_us)) {
        return false;
    }

    plan = {
        .execution_id = input.execution_id,
        .total_major_steps = input.total_major_steps,
        .profile = profile,
        .generator = generator,
    };
    return true;
}

bool trapezoid_execution_plan_next(TrapezoidExecutionPlan& plan,
                                   TrapezoidTimeSlice& slice) {
    return trapezoid_segment_generator_next(plan.generator, slice);
}

bool trapezoid_execution_plan_complete(const TrapezoidExecutionPlan& plan) {
    return plan.generator.complete;
}

}  // namespace ray2::stepper
