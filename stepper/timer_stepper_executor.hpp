#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

#include "raycube_machine.hpp"
#include "segment_ring.hpp"

namespace ray2::stepper {

// Stage two initialization remains passive until the new-driver-compatible
// one-shot RMT backend is ready. It never emits a STEP pulse.
esp_err_t timer_stepper_executor_init();
esp_err_t timer_stepper_executor_start_segment(const StepSegment& segment);
esp_err_t timer_stepper_executor_start_from_segment_ring();
bool timer_stepper_executor_is_busy();
// Requests a non-system motion hold at the next completed segment boundary.
// This is task-context only; it never emits or truncates a STEP pulse.
bool timer_stepper_executor_request_hold();
bool timer_stepper_executor_is_held();
// Restarts consumption from the preserved segment ring after a boundary hold.
esp_err_t timer_stepper_executor_resume_from_hold();
void timer_stepper_executor_stop();
void IRAM_ATTR timer_stepper_executor_stop_from_isr();
// Stops one active system-motion axis. Returns true when no system-motion axis remains.
bool IRAM_ATTR timer_stepper_executor_stop_axis_from_isr(std::size_t axis_index);

}  // namespace ray2::stepper
