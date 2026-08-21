#include "motion_submit.hpp"

#include <cmath>

namespace ray2::motion {
namespace {

MotionRequest s_last_request = {};
std::uint64_t s_submitted_count = 0;

bool valid_request(const MotionRequest& request) {
    if (!request.has_motion || !std::isfinite(request.feed_rate_mm_per_min) ||
        request.feed_rate_mm_per_min < 0.0 || !std::isfinite(request.spindle_power) ||
        request.spindle_power < 0.0 || request.spindle_power > 1000.0) {
        return false;
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(request.start_mm[axis]) || !std::isfinite(request.target_mm[axis])) {
            return false;
        }
    }
    return true;
}

}  // namespace

MotionSubmitResult motion_submit(const MotionRequest& request) {
    if (!valid_request(request)) {
        return MotionSubmitResult::Rejected;
    }

    s_last_request = request;
    ++s_submitted_count;
    return MotionSubmitResult::Accepted;
}

const MotionRequest& last_motion_request() {
    return s_last_request;
}

std::uint64_t submitted_motion_count() {
    return s_submitted_count;
}

const char* submit_result_text(MotionSubmitResult result) {
    switch (result) {
        case MotionSubmitResult::Accepted: return "accepted";
        case MotionSubmitResult::Busy: return "busy";
        case MotionSubmitResult::Rejected: return "rejected";
    }
    return "unknown";
}

}  // namespace ray2::motion
