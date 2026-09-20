#pragma once

#include <cstdint>

#include "esp_err.h"

namespace laserctl::laser {

esp_err_t laser_attach_planner();
std::uint64_t laser_active_execution_id();

}  // namespace laserctl::laser
