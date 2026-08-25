#pragma once

#include <cstdint>

#include "esp_err.h"

namespace ray2::stepper {

struct TimerStepperDiagnostics {
    bool initialized = false;
    bool owns_step_pins = false;
    bool timer_running = false;
    std::uint64_t timer_interrupts = 0;
    std::uint64_t segment_underflows = 0;
};

// Stage one only establishes the executor boundary. It does not allocate a
// hardware timer, claim STEP pins, or emit pulses.
esp_err_t timer_stepper_executor_init();
TimerStepperDiagnostics timer_stepper_executor_diagnostics();

}  // namespace ray2::stepper
