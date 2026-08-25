#include "laser_planner_adapter.hpp"

#include <atomic>

#include "laser_driver.hpp"
#include "motion_request.hpp"
#include "planner.hpp"

namespace ray2::laser {
namespace {

std::atomic<std::uint64_t> s_active_execution_id = 0;

void disable_laser() {
    laser_disable();
    s_active_execution_id.store(0);
}

void planner_execution_callback(const planner::ExecutionEvent& event, void*) {
    switch (event.type) {
        case planner::ExecutionEventType::BlockStarted:
            if (event.request.motion_mode != motion::MotionType::Linear ||
                event.request.spindle_mode == motion::SpindleMode::Off) {
                disable_laser();
            }
            return;
        case planner::ExecutionEventType::SegmentBatchStarted:
            if (event.request.motion_mode == motion::MotionType::Linear &&
                event.request.spindle_mode != motion::SpindleMode::Off &&
                event.segment_power > 0.0 &&
                laser_set_power(event.segment_power) == ESP_OK) {
                s_active_execution_id.store(event.execution_id);
            } else {
                disable_laser();
            }
            return;
        case planner::ExecutionEventType::SegmentBatchCompleted:
            return;
        case planner::ExecutionEventType::Resumed:
            return;
        case planner::ExecutionEventType::FeedHoldRequested:
        case planner::ExecutionEventType::BlockCompleted:
        case planner::ExecutionEventType::BlockCancelled:
        case planner::ExecutionEventType::BlockFailed:
        case planner::ExecutionEventType::QueueDrained:
            disable_laser();
            return;
    }
}

}  // namespace

esp_err_t laser_attach_planner() {
    if (!laser_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    planner::planner_set_execution_callback(planner_execution_callback, nullptr);
    return ESP_OK;
}

std::uint64_t laser_active_execution_id() {
    return s_active_execution_id.load();
}

}  // namespace ray2::laser
