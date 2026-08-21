#pragma once

#include "esp_err.h"

#include "raycube_machine.hpp"

namespace ray2::stepper {

enum class Direction : bool {
    Negative = false,
    Positive = true,
};

esp_err_t stepper_init();
esp_err_t stepper_set_enabled(machine::Axis axis, bool enabled);
esp_err_t stepper_set_direction(machine::Axis axis, Direction direction);
esp_err_t stepper_pulse(machine::Axis axis);

}  // namespace ray2::stepper
