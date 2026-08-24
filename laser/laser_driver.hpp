#pragma once

#include <cstdint>

#include "esp_err.h"

namespace ray2::laser {

// Initializes the PWM peripheral in its safe, disabled state without enabling laser output.
esp_err_t laser_init();
esp_err_t laser_deinit();
bool laser_is_initialized();
esp_err_t laser_set_power(double power);
esp_err_t laser_disable();
bool laser_is_output_enabled();
std::uint32_t laser_current_duty();

}  // namespace ray2::laser
