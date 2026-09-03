#pragma once

#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace ray2::driver {

using StepTimerAlarmCallback = bool (*)(void* user_data);

// Owns the GPTimer lifecycle used as the STEP event timebase. Intervals are
// expressed in 20 MHz timer ticks to match the legacy Grbl executor.
esp_err_t step_timer_driver_init(StepTimerAlarmCallback callback, void* user_data);
esp_err_t step_timer_driver_start(std::uint32_t interval_ticks);
esp_err_t step_timer_driver_stop();
bool step_timer_driver_is_initialized();
bool step_timer_driver_is_running();
int step_timer_driver_init_core();
int step_timer_driver_isr_core();

// ISR-safe terminal stop for the callback registered through init().
void IRAM_ATTR step_timer_driver_stop_from_isr();
void IRAM_ATTR step_timer_driver_update_interval_from_isr(std::uint32_t interval_ticks);
// Returns timer ticks elapsed from the hardware alarm edge to the ISR read.
// This is a latency observation only; it does not acknowledge or alter the alarm.
std::uint64_t IRAM_ATTR step_timer_driver_lateness_ticks_from_isr();

}  // namespace ray2::driver
