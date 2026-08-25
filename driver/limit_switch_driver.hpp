#pragma once

#include <cstdint>

#include "esp_err.h"

namespace ray2::driver {

struct LimitSwitchState {
    bool triggered[2] = {};
    std::uint32_t debounce_events = 0;
};

// Owns RayCube X/Y limit GPIO input, ISR wakeup, and debounced state snapshots.
esp_err_t limit_switch_driver_init();
bool limit_switch_driver_is_initialized();
LimitSwitchState limit_switch_driver_state();

}  // namespace ray2::driver
