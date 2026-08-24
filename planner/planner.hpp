#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "motion_request.hpp"

namespace ray2::planner {

constexpr std::size_t kPlannerQueueLength = 32;

struct MotionBlock {
    motion::MotionRequest request = {};
    motion::MotionExecutionPlan plan = {};
    std::uint32_t reset_generation = 0;
    std::uint64_t execution_id = 0;
};

enum class SubmitResult {
    Accepted,
    Full,
    Rejected,
};

enum class ExecutionEventType {
    BlockStarted,
    FeedHoldRequested,
    Resumed,
    BlockCompleted,
    BlockCancelled,
    BlockFailed,
    QueueDrained,
};

struct ExecutionEvent {
    ExecutionEventType type = ExecutionEventType::QueueDrained;
    std::uint64_t execution_id = 0;
    motion::MotionRequest request = {};
    motion::MotionExecutionPlan plan = {};
};

// Callbacks run in the caller's task and must return quickly without calling planner APIs.
using ExecutionCallback = void (*)(const ExecutionEvent& event, void* user_data);

esp_err_t planner_start();
SubmitResult planner_submit(const MotionBlock& block);
void planner_set_execution_callback(ExecutionCallback callback, void* user_data);
void planner_request_feed_hold();
void planner_resume();
void planner_reset();
std::size_t planner_free_blocks();
std::uint64_t planner_completed_block_count();

}  // namespace ray2::planner
