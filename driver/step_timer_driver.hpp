#pragma once

#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace ray2::driver {

using StepTimerAlarmCallback = bool (*)(void* user_data);

// Owns the GPTimer lifecycle used as the STEP event timebase.
esp_err_t step_timer_driver_init(StepTimerAlarmCallback callback, void* user_data);
esp_err_t step_timer_driver_start(std::uint32_t interval_us);
esp_err_t step_timer_driver_stop();
bool step_timer_driver_is_initialized();
bool step_timer_driver_is_running();

// ISR-safe terminal stop for the callback registered through init().
void IRAM_ATTR step_timer_driver_stop_from_isr();
void IRAM_ATTR step_timer_driver_update_interval_from_isr(std::uint32_t interval_us);

}  // namespace ray2::driver
