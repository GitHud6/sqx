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
// Requests a feed hold at the next completed segment boundary. Queued segments
// remain intact so the planner can resume them without changing their profile.
void timer_stepper_executor_request_feed_hold();
void timer_stepper_executor_resume();
void timer_stepper_executor_stop();
void IRAM_ATTR timer_stepper_executor_stop_from_isr();
// Stops one active system-motion axis. Returns true when no system-motion axis remains.
bool IRAM_ATTR timer_stepper_executor_stop_axis_from_isr(std::size_t axis_index);
TimerStepperDiagnostics timer_stepper_executor_diagnostics();

}  // namespace ray2::stepper
