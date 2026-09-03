#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace ray2::driver {

struct RmtStepPulseDiagnostics {
    bool invalid_trigger_seen = false;
};

// Owns the ESP32-S3 RMT channels and GPIO matrix used for STEP pulse output.
esp_err_t rmt_step_pulse_driver_init();
void rmt_step_pulse_driver_stop();
void IRAM_ATTR rmt_step_pulse_driver_stop_from_isr();
void IRAM_ATTR rmt_step_pulse_driver_stop_axis_from_isr(std::size_t axis_index);
bool rmt_step_pulse_driver_is_initialized();

// ISR-safe. The caller must pass an initialized axis index in range.
void IRAM_ATTR rmt_step_pulse_driver_trigger(std::size_t axis_index);
RmtStepPulseDiagnostics rmt_step_pulse_driver_diagnostics();

}  // namespace ray2::driver
