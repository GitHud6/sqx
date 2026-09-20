#pragma once

#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"

namespace laserctl::laser {

// Initializes the PWM peripheral in its safe, disabled state without enabling laser output.
esp_err_t laser_init();
esp_err_t laser_deinit();
bool laser_is_initialized();
esp_err_t laser_set_power(double power);
esp_err_t laser_disable();
void IRAM_ATTR laser_apply_segment_power_from_isr(std::uint16_t power_per_mille,
                                                  std::uint64_t execution_id);
void IRAM_ATTR laser_disable_from_isr();
void laser_set_motion_output_permitted(bool permitted);
bool laser_is_output_enabled();
std::uint32_t laser_current_duty();

}  // namespace laserctl::laser
