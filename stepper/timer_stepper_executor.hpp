#pragma once

#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

#include "raycube_machine.hpp"

namespace ray2::stepper {

struct StepSegment;

struct TimerStepperDiagnostics {
    bool initialized = false;
    bool owns_step_pins = false;
    bool timer_running = false;
    std::uint64_t timer_interrupts = 0;
    std::uint64_t segment_underflows = 0;
};

// Stage two initialization remains passive until the new-driver-compatible
// one-shot RMT backend is ready. It never emits a STEP pulse.
esp_err_t timer_stepper_executor_init();
esp_err_t timer_stepper_executor_start_segment(const StepSegment& segment);
esp_err_t timer_stepper_executor_start_from_segment_ring();
bool timer_stepper_executor_is_busy();
void timer_stepper_executor_stop();
void IRAM_ATTR timer_stepper_executor_stop_from_isr();
TimerStepperDiagnostics timer_stepper_executor_diagnostics();

}  // namespace ray2::stepper
