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
bool build_motion_geometry(const MotionRequest& request, MotionGeometry& geometry);
const MotionRequest& last_motion_request();
const MotionGeometry& last_motion_geometry();
std::uint64_t submitted_motion_count();
const char* submit_result_text(MotionSubmitResult result);

}  // namespace ray2::motion
