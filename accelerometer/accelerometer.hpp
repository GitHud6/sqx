#pragma once

#include "esp_err.h"

namespace ray2::accelerometer {

esp_err_t init();
bool is_initialized();

}  // namespace ray2::accelerometer
