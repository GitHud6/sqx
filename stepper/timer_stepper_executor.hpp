#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

#include "raycube_machine.hpp"
#include "segment_ring.hpp"

namespace ray2::stepper {

struct TimerStepperDiagnostics {
    bool initialized = false;
    bool owns_step_pins = false;
    bool timer_running = false;
    int timer_init_core = -1;
    int timer_isr_core = -1;
    // These counters feed a one-second trace window. Keep them 32-bit so
    // segment-boundary diagnostics do not require 64-bit atomic operations
    // in the Timer ISR. Unsigned subtraction preserves deltas across wrap.
    std::uint32_t scheduled_timer_events = 0;
    std::uint32_t scheduled_timer_ticks = 0;
    std::uint32_t segment_handoffs = 0;
    std::uint32_t block_phase_continuations = 0;
    std::uint32_t block_phase_initializations = 0;
    std::uint32_t pulse_callbacks = 0;
    std::uint32_t rmt_triggers[machine::kAxisCount] = {};
    std::uint32_t direction_writes = 0;
    std::uint32_t interval_min_ticks = 0;
    std::uint32_t interval_max_ticks = 0;
    std::uint32_t interval_max_delta_ticks = 0;
    std::uint32_t timer_lateness_samples = 0;
    std::uint32_t timer_lateness_max_ticks = 0;
    std::uint32_t timer_lateness_boundary_samples = 0;
    std::uint32_t timer_lateness_boundary_max_ticks = 0;
    std::uint32_t timer_lateness_over_2us = 0;
    std::uint32_t timer_lateness_over_5us = 0;
    std::uint32_t timer_lateness_over_10us = 0;
    std::uint32_t timer_lateness_mask_samples[4] = {};
    std::uint32_t timer_lateness_over_10us_masks[4] = {};
    std::uint32_t timer_isr_body_samples = 0;
    std::uint32_t timer_isr_body_max_ticks = 0;
    std::uint32_t timer_isr_body_boundary_samples = 0;
    std::uint32_t timer_isr_body_boundary_max_ticks = 0;
    std::uint32_t timer_isr_body_over_10us = 0;
    std::uint32_t timer_isr_body_counter_wrapped = 0;
    std::uint32_t boundary_max_delta_ticks = 0;
    std::uint32_t boundary_samples = 0;
    std::uint32_t amass_level_changes = 0;
    std::uint32_t amass_transitions[4][4] = {};
    std::uint32_t amass_3_to_0 = 0;
    std::uint32_t amass_0_to_3 = 0;
    std::uint64_t boundary_prev_execution_id = 0;
    std::uint64_t boundary_next_execution_id = 0;
    std::uint32_t boundary_prev_interval_ticks = 0;
    std::uint32_t boundary_next_interval_ticks = 0;
    std::uint32_t boundary_prev_base_interval_ticks = 0;
    std::uint32_t boundary_next_base_interval_ticks = 0;
    std::uint32_t boundary_prev_duration_us = 0;
    std::uint32_t boundary_next_duration_us = 0;
    std::uint32_t boundary_prev_start_speed_tenths_mm_per_min = 0;
    std::uint32_t boundary_prev_end_speed_tenths_mm_per_min = 0;
    std::uint32_t boundary_next_start_speed_tenths_mm_per_min = 0;
    std::uint32_t boundary_next_end_speed_tenths_mm_per_min = 0;
    std::uint8_t boundary_prev_phase = 0;
    std::uint8_t boundary_next_phase = 0;
    std::uint8_t boundary_prev_amass_level = 0;
    std::uint8_t boundary_next_amass_level = 0;
    std::int32_t boundary_prev_major_steps = 0;
    std::int32_t boundary_next_major_steps = 0;
};

// Stage two initialization remains passive until the new-driver-compatible
// one-shot RMT backend is ready. It never emits a STEP pulse.
esp_err_t timer_stepper_executor_init();
esp_err_t timer_stepper_executor_start_segment(const StepSegment& segment);
esp_err_t timer_stepper_executor_start_from_segment_ring();
bool timer_stepper_executor_is_busy();
void timer_stepper_executor_stop();
void IRAM_ATTR timer_stepper_executor_stop_from_isr();
// Stops one active system-motion axis. Returns true when no system-motion axis remains.
bool IRAM_ATTR timer_stepper_executor_stop_axis_from_isr(std::size_t axis_index);
TimerStepperDiagnostics timer_stepper_executor_diagnostics();

}  // namespace ray2::stepper
