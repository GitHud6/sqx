#pragma once

#include <cstdint>

#include "motion_request.hpp"

namespace ray2::motion {

enum class MotionSubmitResult {
    Accepted,
    Busy,
    Rejected,
};

MotionSubmitResult motion_submit(const MotionRequest& request);
const MotionRequest& last_motion_request();
std::uint64_t submitted_motion_count();
const char* submit_result_text(MotionSubmitResult result);

}  // namespace ray2::motion
