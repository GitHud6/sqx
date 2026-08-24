#include "motion_submit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "esp_log.h"
#include "raycube_machine.hpp"
#include "planner.hpp"

namespace ray2::motion {
namespace {

MotionRequest s_last_request = {};
MotionExecutionPlan s_last_plan = {};
std::uint64_t s_submitted_count = 0;

constexpr std::uint32_t kMinStepFrequencyHz = 10;
constexpr std::uint32_t kMaxStepFrequencyHz = 10000;
constexpr double kRapidRateMmPerMin = 6000.0;
constexpr char kLogTag[] = "motion";

void log_rejection(const char* reason,
                   const MotionRequest& request,
                   const MotionExecutionPlan* plan = nullptr) {
    ESP_LOGE(kLogTag,
             "MOTION-REJECT reason=%s mode=%d start=(%.6f,%.6f,%.6f) "
             "target=(%.6f,%.6f,%.6f) feed=%.3f power=%.3f steps=(%ld,%ld) interval=%lu",
             reason,
             static_cast<int>(request.motion_mode),
             request.start_mm[0], request.start_mm[1], request.start_mm[2],
             request.target_mm[0], request.target_mm[1], request.target_mm[2],
             request.feed_rate_mm_per_min,
             request.spindle_power,
             plan == nullptr ? 0L : static_cast<long>(plan->steps[0]),
             plan == nullptr ? 0L : static_cast<long>(plan->steps[1]),
             plan == nullptr ? 0UL : static_cast<unsigned long>(plan->step_interval_us));
}

bool valid_coordinate(double value, double max_travel) {
    return std::isfinite(value) && value >= 0.0 && value <= max_travel;
}

std::uint32_t step_interval_us(const MotionRequest& request,
                               const MotionExecutionPlan& plan) {
    const auto& x_hardware = machine::raycube_axis(machine::Axis::X);
    const auto& y_hardware = machine::raycube_axis(machine::Axis::Y);
    const double feed_rate = request.motion_mode == MotionType::Rapid
                                 ? std::fmin(kRapidRateMmPerMin,
                                             std::min(static_cast<double>(x_hardware.max_rate_mm_per_min),
                                                      static_cast<double>(y_hardware.max_rate_mm_per_min)))
                                 : request.feed_rate_mm_per_min;
    const double path_length_mm = std::hypot(
        request.target_mm[0] - request.start_mm[0],
        request.target_mm[1] - request.start_mm[1]);
    const std::int32_t major_steps = std::max(plan.steps[0], plan.steps[1]);
    if (major_steps <= 0 || !std::isfinite(path_length_mm) || path_length_mm <= 0.0) {
        return 0;
    }

    const double requested_frequency =
        feed_rate * major_steps / (path_length_mm * 60.0);
    if (!std::isfinite(requested_frequency) || requested_frequency <= 0.0) {
        return 0;
    }

    const double limited_frequency = std::fmin(
        std::fmax(requested_frequency, static_cast<double>(kMinStepFrequencyHz)),
        static_cast<double>(kMaxStepFrequencyHz));
    const double interval = 1000000.0 / limited_frequency;
    if (!std::isfinite(interval) || interval < 1.0 || interval > std::numeric_limits<std::uint32_t>::max()) {
        return 0;
    }
    return static_cast<std::uint32_t>(std::lround(interval));
}

bool build_execution_plan_impl(const MotionRequest& request, MotionExecutionPlan& plan) {
    if (!request.has_motion || !std::isfinite(request.feed_rate_mm_per_min) ||
        request.feed_rate_mm_per_min < 0.0 || !std::isfinite(request.spindle_power) ||
        request.spindle_power < 0.0 || request.spindle_power > 1000.0) {
        log_rejection("invalid-request", request);
        return false;
    }

    MotionExecutionPlan candidate = {};
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis));
        if (request.feed_rate_mm_per_min > hardware.max_rate_mm_per_min ||
            !valid_coordinate(request.start_mm[axis], hardware.max_travel_mm) ||
            !valid_coordinate(request.target_mm[axis], hardware.max_travel_mm) ||
            !std::isfinite(hardware.steps_per_mm) || hardware.steps_per_mm <= 0.0F) {
            log_rejection(axis == 0 ? "x-coordinate-or-rate" : "y-coordinate-or-rate",
                          request, &candidate);
            return false;
        }

        const double delta_mm = request.target_mm[axis] - request.start_mm[axis];
        const double step_count = std::abs(delta_mm) * hardware.steps_per_mm;
        if (!std::isfinite(step_count) || step_count > std::numeric_limits<std::int32_t>::max()) {
            log_rejection(axis == 0 ? "x-step-count" : "y-step-count", request, &candidate);
            return false;
        }
        candidate.steps[axis] = static_cast<std::int32_t>(std::lround(step_count));
        candidate.positive_direction[axis] = delta_mm >= 0.0;
    }

    if (request.start_mm[2] != 0.0 || request.target_mm[2] != 0.0) {
        log_rejection("z-axis-not-supported", request, &candidate);
        return false;
    }

    std::size_t active_axis_count = 0;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (candidate.steps[axis] > 0) {
            ++active_axis_count;
        }
    }
    if (active_axis_count > 0) {
        candidate.step_interval_us = step_interval_us(request, candidate);
        if (candidate.step_interval_us == 0) {
            log_rejection("step-interval-zero", request, &candidate);
            return false;
        }
    }

    plan = candidate;
    return true;
}

}  // namespace

bool build_execution_plan(const MotionRequest& request, MotionExecutionPlan& plan) {
    return build_execution_plan_impl(request, plan);
}

MotionSubmitResult motion_submit(const MotionRequest& request) {
    MotionExecutionPlan plan = {};
    if (!build_execution_plan_impl(request, plan)) {
        return MotionSubmitResult::Rejected;
    }

    planner::PlannerBlock block = {
        .request = request,
        .plan = plan,
    };
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        block.machine_start_mm[axis] = request.start_mm[axis];
        block.machine_target_mm[axis] = request.target_mm[axis];
    }
    block.step_event_count = std::max(plan.steps[0], plan.steps[1]);
    const double delta_x = request.target_mm[0] - request.start_mm[0];
    const double delta_y = request.target_mm[1] - request.start_mm[1];
    block.millimeters = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    block.programmed_rate_mm_per_min = request.motion_mode == MotionType::Rapid
                                            ? kRapidRateMmPerMin
                                            : request.feed_rate_mm_per_min;
    if (block.millimeters > 0.0) {
        block.unit_vector[0] = delta_x / block.millimeters;
        block.unit_vector[1] = delta_y / block.millimeters;
    }

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (plan.steps[axis] == 0) {
            continue;
        }
        const double axis_component = std::abs(block.unit_vector[axis]);
        if (axis_component == 0.0) {
            continue;
        }
        const double axis_limited_acceleration =
            machine::raycube_axis(static_cast<machine::Axis>(axis)).acceleration_mm_per_sec2 /
            axis_component;
        if (block.acceleration_mm_per_sec2 == 0.0 ||
            axis_limited_acceleration < block.acceleration_mm_per_sec2) {
            block.acceleration_mm_per_sec2 = axis_limited_acceleration;
        }
    }
    switch (planner::planner_submit(block)) {
        case planner::SubmitResult::Accepted:
            break;
        case planner::SubmitResult::Full:
            return MotionSubmitResult::Busy;
        case planner::SubmitResult::Rejected:
            log_rejection("planner-rejected", request, &plan);
            return MotionSubmitResult::Rejected;
    }

    s_last_request = request;
    s_last_plan = plan;
    ++s_submitted_count;
    return MotionSubmitResult::Accepted;
}

const MotionExecutionPlan& last_motion_plan() {
    return s_last_plan;
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
