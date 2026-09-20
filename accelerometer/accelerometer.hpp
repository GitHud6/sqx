#pragma once

#include "esp_err.h"

namespace laserctl::accelerometer {

esp_err_t init();
bool is_initialized();

}  // namespace laserctl::accelerometer
