#include "raycube_machine.hpp"

namespace ray2::machine {
namespace {

constexpr AxisHardware kAxes[kAxisCount] = {
    {
        .step_pin = 15,
        .direction_pin = 7,
        .disable_pin = 16,
        .direction_inverted = true,
        .homing_seek_positive_direction = false,
        .disable_active_high = true,
        .steps_per_mm = 80.0F,
        .max_rate_mm_per_min = 12000.0F,
        .acceleration_mm_per_sec2 = 500.0F,
        .max_travel_mm = 130.0F,
        .limit_pin = 42,
        .limit_active_low = true,
    },
    {
        .step_pin = 8,
        .direction_pin = 17,
        .disable_pin = 18,
        .direction_inverted = true,
        .homing_seek_positive_direction = false,
        .disable_active_high = true,
        .steps_per_mm = 80.0F,
        .max_rate_mm_per_min = 12000.0F,
        .acceleration_mm_per_sec2 = 500.0F,
        .max_travel_mm = 110.0F,
        .limit_pin = 2,
        .limit_active_low = true,
    },
};

constexpr LaserHardware kLaser = {
    .pwm_pin = 4,
    .enable_pin = 5,
    .pwm_inverted = false,
    .enable_active_high = true,
    .pwm_frequency_hz = 2000,
    .max_power = 1000,
};

}  // namespace

const AxisHardware& raycube_axis(Axis axis) {
    return kAxes[static_cast<std::size_t>(axis)];
}

const LaserHardware& raycube_laser() {
    return kLaser;
}

}  // namespace ray2::machine
