#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "raycube_machine.hpp"

namespace ray2::stepper {

constexpr unsigned int kRmtStepResolutionHz = 1000000;
constexpr std::size_t kRmtStepMemorySymbols = 64;
constexpr std::uint32_t kRmtStepMaxIntervalUs = 32767;

// This only allocates and enables RMT TX channels. It never transmits STEP pulses.
esp_err_t rmt_stepper_init();
esp_err_t rmt_stepper_deinit();
bool rmt_stepper_is_initialized();
bool rmt_stepper_can_execute(std::int32_t step_count, std::uint32_t step_interval_us);
bool rmt_stepper_can_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                std::uint32_t step_interval_us);
esp_err_t rmt_stepper_execute(machine::Axis axis,
                               std::int32_t step_count,
                               std::uint32_t step_interval_us);
esp_err_t rmt_stepper_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                 std::uint32_t step_interval_us);

}  // namespace ray2::stepper
