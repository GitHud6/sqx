#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace laserctl::driver {

// Owns the ESP32-S3 RMT channels and GPIO matrix used for STEP pulse output.
esp_err_t rmt_step_pulse_driver_init();
void rmt_step_pulse_driver_stop();
void rmt_step_pulse_driver_stop_from_isr();
void rmt_step_pulse_driver_stop_axis_from_isr(std::size_t axis_index);
bool rmt_step_pulse_driver_is_initialized();

// ISR-safe. The caller must pass an initialized axis index in range.
void rmt_step_pulse_driver_trigger(std::size_t axis_index);

}  // namespace laserctl::driver
