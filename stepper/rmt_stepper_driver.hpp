#pragma once

#include <cstddef>

#include "esp_err.h"

namespace ray2::stepper {

constexpr unsigned int kRmtStepResolutionHz = 1000000;
constexpr std::size_t kRmtStepMemorySymbols = 64;

// This only allocates and enables RMT TX channels. It never transmits STEP pulses.
esp_err_t rmt_stepper_init();
esp_err_t rmt_stepper_deinit();
bool rmt_stepper_is_initialized();

}  // namespace ray2::stepper
