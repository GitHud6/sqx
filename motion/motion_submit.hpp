#pragma once

#include <cstddef>
#include <cstdint>

#include "motion_request.hpp"

namespace ray2::motion {

enum class MotionSubmitResult {
    Accepted,
    Busy,
    Rejected,
};

MotionSubmitResult motion_submit(const MotionRequest& request);
MotionSubmitResult motion_submit_batch(const MotionRequest* requests, std::size_t count);
bool build_execution_plan(const MotionRequest& request, MotionExecutionPlan& plan);
const MotionRequest& last_motion_request();
const MotionExecutionPlan& last_motion_plan();
std::uint64_t submitted_motion_count();
const char* submit_result_text(MotionSubmitResult result);

}  // namespace ray2::motion
