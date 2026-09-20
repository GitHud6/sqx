#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace laserctl::driver {

using LimitSwitchIsrCallback = void (*)(std::size_t axis, void* user_data);

struct LimitSwitchState {
    bool triggered[2] = {};
    std::uint32_t debounce_events = 0;
};

using LimitSwitchStateCallback = void (*)(const LimitSwitchState& state, void* user_data);

// Owns LaserController X/Y limit GPIO input, ISR wakeup, and debounced state snapshots.
esp_err_t limit_switch_driver_init();
bool limit_switch_driver_is_initialized();
LimitSwitchState limit_switch_driver_state();
void limit_switch_driver_set_isr_callback(LimitSwitchIsrCallback callback, void* user_data);
// Invoked from the debounce task after a stable state is sampled. It is never
// called from the GPIO ISR and may request task-context safety handling.
void limit_switch_driver_set_state_callback(LimitSwitchStateCallback callback, void* user_data);

}  // namespace laserctl::driver
