#pragma once

#include "esp_err.h"

namespace ray2::laser {

// Initializes the PWM peripheral in its safe, disabled state without enabling laser output.
esp_err_t laser_init();
esp_err_t laser_deinit();
bool laser_is_initialized();

}  // namespace ray2::laser
