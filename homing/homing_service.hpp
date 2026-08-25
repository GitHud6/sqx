#pragma once

#include <cstdint>

#include "esp_err.h"

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

enum class TestAxis : std::uint8_t {
    X = 0,
    Y = 1,
};

// Starts the system homing workflow when its execution backend is available.
esp_err_t homing_init();
StartResult homing_start();
HomingState homing_state();
bool homing_is_active();
const char* homing_state_text(HomingState state);
StartResult homing_start_seek_test(TestAxis axis);

}  // namespace ray2::homing
