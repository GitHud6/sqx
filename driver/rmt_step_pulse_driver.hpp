#pragma once

#include <cstddef>

#include "esp_attr.h"
#include "esp_err.h"

namespace ray2::driver {

// Owns the ESP32-S3 RMT channels and GPIO matrix used for STEP pulse output.
esp_err_t rmt_step_pulse_driver_init();
void rmt_step_pulse_driver_stop();
bool rmt_step_pulse_driver_is_initialized();

// ISR-safe. The caller must pass an initialized axis index in range.
void IRAM_ATTR rmt_step_pulse_driver_trigger(std::size_t axis_index);

}  // namespace ray2::driver
