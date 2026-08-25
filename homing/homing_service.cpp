#include "homing_service.hpp"

#include <atomic>

#include "planner.hpp"
#include "sd_job_service.hpp"

namespace ray2::homing {
namespace {

std::atomic<HomingState> s_state = HomingState::Idle;

}  // namespace

StartResult homing_start() {
    if (homing_is_active() || !planner::planner_is_idle() || sd::sd_job_is_active()) {
        return StartResult::Busy;
    }

    // The limit driver and system-motion backend are introduced in later phases.
    // Do not claim a coordinate reference until they can complete successfully.
    return StartResult::Unavailable;
}

HomingState homing_state() {
    return s_state.load(std::memory_order_relaxed);
}

bool homing_is_active() {
    return homing_state() == HomingState::Running;
}

const char* homing_state_text(const HomingState state) {
    switch (state) {
        case HomingState::Idle: return "idle";
        case HomingState::Running: return "running";
        case HomingState::Completed: return "completed";
        case HomingState::Faulted: return "faulted";
    }
    return "unknown";
}

}  // namespace ray2::homing
