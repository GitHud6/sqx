#pragma once

namespace ray2::homing {

enum class HomingState {
    Idle,
    Running,
    Completed,
    Faulted,
};

enum class StartResult {
    Accepted,
    Busy,
    Unavailable,
};

// Starts the system homing workflow when its execution backend is available.
StartResult homing_start();
HomingState homing_state();
bool homing_is_active();
const char* homing_state_text(HomingState state);

}  // namespace ray2::homing
