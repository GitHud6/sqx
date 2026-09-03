#include "motion_submit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "esp_log.h"
#include "raycube_machine.hpp"
#include "planner.hpp"

namespace ray2::motion {
namespace {

MotionRequest s_last_request = {};
MotionGeometry s_last_geometry = {};
std::uint64_t s_submitted_count = 0;
// Command dispatch serializes G-code submission. Keep the arc batch workspace
// out of the protocol task stack, where 32 full PlannerBlock instances would
// exceed the task's available headroom.
std::array<planner::PlannerBlock, planner::kPlannerQueueLength> s_batch_blocks = {};

constexpr char kLogTag[] = "motion";

void log_rejection(const char* reason,
                   const MotionRequest& request,
                   const MotionGeometry* geometry = nullptr) {
    ESP_LOGE(kLogTag,
             "MOTION-REJECT reason=%s mode=%d start=(%.6f,%.6f,%.6f) "
             "target=(%.6f,%.6f,%.6f) feed=%.3f power=%.3f steps=(%ld,%ld)",
             reason,
             static_cast<int>(request.motion_mode),
             request.start_mm[0], request.start_mm[1], request.start_mm[2],
             request.target_mm[0], request.target_mm[1], request.target_mm[2],
             request.feed_rate_mm_per_min,
             request.spindle_power,
             geometry == nullptr ? 0L : static_cast<long>(geometry->steps[0]),
             geometry == nullptr ? 0L : static_cast<long>(geometry->steps[1]));
}

bool valid_coordinate(double value, double max_travel) {
    return std::isfinite(value) && value >= 0.0 && value <= max_travel;
}

bool quantize_absolute_steps(const double position_mm,
                             const float steps_per_mm,
                             std::int64_t& steps) {
    const double scaled = position_mm * static_cast<double>(steps_per_mm);
    if (!std::isfinite(scaled) ||
        scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
        scaled > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        return false;
    }
    steps = std::llround(scaled);
    return true;
}

bool build_motion_geometry_impl(const MotionRequest& request, MotionGeometry& geometry) {
    if (!request.has_motion || !std::isfinite(request.feed_rate_mm_per_min) ||
        request.feed_rate_mm_per_min < 0.0 || !std::isfinite(request.spindle_power) ||
        request.spindle_power < 0.0 || request.spindle_power > 1000.0) {
        log_rejection("invalid-request", request);
        return false;
    }

    MotionGeometry candidate = {};
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis));
        if ((request.motion_mode == MotionType::Linear &&
             request.feed_rate_mm_per_min > hardware.max_rate_mm_per_min) ||
            !valid_coordinate(request.start_mm[axis], hardware.max_travel_mm) ||
            !valid_coordinate(request.target_mm[axis], hardware.max_travel_mm) ||
            !std::isfinite(hardware.steps_per_mm) || hardware.steps_per_mm <= 0.0F) {
            log_rejection(axis == 0 ? "x-coordinate-or-rate" : "y-coordinate-or-rate",
                          request, &candidate);
            return false;
        }

        std::int64_t start_steps = 0;
        std::int64_t target_steps = 0;
        if (!quantize_absolute_steps(request.start_mm[axis], hardware.steps_per_mm, start_steps) ||
            !quantize_absolute_steps(request.target_mm[axis], hardware.steps_per_mm, target_steps)) {
            log_rejection(axis == 0 ? "x-step-count" : "y-step-count", request, &candidate);
            return false;
        }
        const std::int64_t delta_steps = target_steps - start_steps;
        if (delta_steps < -std::numeric_limits<std::int32_t>::max() ||
            delta_steps > std::numeric_limits<std::int32_t>::max()) {
            log_rejection(axis == 0 ? "x-step-count" : "y-step-count", request, &candidate);
            return false;
        }
        candidate.steps[axis] = static_cast<std::int32_t>(std::abs(delta_steps));
        candidate.positive_direction[axis] = delta_steps >= 0;
    }

    if (request.start_mm[2] != 0.0 || request.target_mm[2] != 0.0) {
        log_rejection("z-axis-not-supported", request, &candidate);
        return false;
    }

    geometry = candidate;
    return true;
}

double rapid_rate_for_direction(const planner::PlannerBlock& block) {
    double rapid_rate_mm_per_min = std::numeric_limits<double>::infinity();
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (block.geometry.steps[axis] == 0) continue;

        const double axis_component = std::abs(block.unit_vector[axis]);
        const double axis_max_rate =
            machine::raycube_axis(static_cast<machine::Axis>(axis)).max_rate_mm_per_min;
        if (!std::isfinite(axis_component) || axis_component <= 0.0 ||
            !std::isfinite(axis_max_rate) || axis_max_rate <= 0.0) {
            return 0.0;
        }
        rapid_rate_mm_per_min = std::min(rapid_rate_mm_per_min, axis_max_rate / axis_component);
    }
    return std::isfinite(rapid_rate_mm_per_min) ? rapid_rate_mm_per_min : 0.0;
}

bool make_planner_block(const MotionRequest& request,
                        const MotionGeometry& geometry,
                        planner::PlannerBlock& block) {
    block = {.request = request, .geometry = geometry};
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        block.machine_start_mm[axis] = request.start_mm[axis];
        block.machine_target_mm[axis] = request.target_mm[axis];
    }
    block.step_event_count = std::max(geometry.steps[0], geometry.steps[1]);
    const double delta_x = request.target_mm[0] - request.start_mm[0];
    const double delta_y = request.target_mm[1] - request.start_mm[1];
    block.millimeters = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (block.millimeters <= 0.0) return false;
    block.unit_vector[0] = delta_x / block.millimeters;
    block.unit_vector[1] = delta_y / block.millimeters;
    block.programmed_rate_mm_per_min = request.motion_mode == MotionType::Rapid
                                            ? rapid_rate_for_direction(block)
                                            : request.feed_rate_mm_per_min;
    if (!std::isfinite(block.programmed_rate_mm_per_min) ||
        block.programmed_rate_mm_per_min <= 0.0) {
        return false;
    }
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (geometry.steps[axis] == 0) continue;
        const double axis_component = std::abs(block.unit_vector[axis]);
        if (axis_component == 0.0) continue;
        const double axis_limited_acceleration =
            machine::raycube_axis(static_cast<machine::Axis>(axis)).acceleration_mm_per_sec2 /
            axis_component;
        if (block.acceleration_mm_per_sec2 == 0.0 ||
            axis_limited_acceleration < block.acceleration_mm_per_sec2) {
            block.acceleration_mm_per_sec2 = axis_limited_acceleration;
        }
    }
    return true;
}

}  // namespace

bool build_motion_geometry(const MotionRequest& request, MotionGeometry& geometry) {
    return build_motion_geometry_impl(request, geometry);
}

MotionSubmitResult motion_submit(const MotionRequest& request) {
    MotionGeometry geometry = {};
    if (!build_motion_geometry_impl(request, geometry)) {
        return MotionSubmitResult::Rejected;
    }

    planner::PlannerBlock block = {};
    if (!make_planner_block(request, geometry, block)) return MotionSubmitResult::Rejected;
    switch (planner::planner_submit(block)) {
        case planner::SubmitResult::Accepted:
            break;
        case planner::SubmitResult::Full:
            return MotionSubmitResult::Busy;
        case planner::SubmitResult::Rejected:
            log_rejection("planner-rejected", request, &geometry);
            return MotionSubmitResult::Rejected;
    }

    s_last_request = request;
    s_last_geometry = geometry;
    ++s_submitted_count;
    return MotionSubmitResult::Accepted;
}

MotionSubmitResult motion_submit_batch(const MotionRequest* const requests, const std::size_t count) {
    if (requests == nullptr || count == 0 || count > planner::kPlannerQueueLength) {
        return MotionSubmitResult::Rejected;
    }
    MotionGeometry final_geometry = {};
    for (std::size_t index = 0; index < count; ++index) {
        MotionGeometry geometry = {};
        if (!build_motion_geometry_impl(requests[index], geometry) ||
            !make_planner_block(requests[index], geometry, s_batch_blocks[index])) {
            return MotionSubmitResult::Rejected;
        }
        if (index + 1 == count) final_geometry = geometry;
    }
    switch (planner::planner_submit_batch(s_batch_blocks.data(), count)) {
        case planner::SubmitResult::Accepted:
            s_last_request = requests[count - 1];
            s_last_geometry = final_geometry;
            s_submitted_count += count;
            return MotionSubmitResult::Accepted;
        case planner::SubmitResult::Full: return MotionSubmitResult::Busy;
        case planner::SubmitResult::Rejected: return MotionSubmitResult::Rejected;
    }
    return MotionSubmitResult::Rejected;
}

const MotionGeometry& last_motion_geometry() {
    return s_last_geometry;
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
