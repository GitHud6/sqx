#include "raycube_machine.hpp"

namespace ray2::machine {
namespace {

constexpr AxisHardware kAxes[kAxisCount] = {
    {
        .step_pin = 15,
        .direction_pin = 7,
        .disable_pin = 16,
        .direction_inverted = false,
        .disable_active_high = true,
        .steps_per_mm = 80.0F,
        .max_rate_mm_per_min = 12000.0F,
        .acceleration_mm_per_sec2 = 500.0F,
    },
    {
        .step_pin = 8,
        .direction_pin = 17,
        .disable_pin = 18,
        .direction_inverted = false,
        .disable_active_high = true,
        .steps_per_mm = 80.0F,
        .max_rate_mm_per_min = 12000.0F,
        .acceleration_mm_per_sec2 = 500.0F,
    },
};

}  // namespace

const AxisHardware& raycube_axis(Axis axis) {
    return kAxes[static_cast<std::size_t>(axis)];
}

}  // namespace ray2::machine
