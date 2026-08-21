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
};

enum class SubmitResult {
    Accepted,
    Full,
    Rejected,
};

esp_err_t planner_start();
SubmitResult planner_submit(const MotionBlock& block);
void planner_request_feed_hold();
void planner_resume();
void planner_reset();
std::size_t planner_free_blocks();
std::uint64_t planner_completed_block_count();

}  // namespace ray2::planner
