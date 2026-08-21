#pragma once

#include <cstddef>

namespace ray2::machine {

enum class Axis : std::size_t {
    X = 0,
    Y = 1,
};

struct AxisHardware {
    int step_pin;
    int direction_pin;
    int disable_pin;
    bool direction_inverted;
    bool disable_active_high;
    float steps_per_mm;
    float max_rate_mm_per_min;
    float acceleration_mm_per_sec2;
};

constexpr std::size_t kAxisCount = 2;
constexpr unsigned int kStepPulseMicroseconds = 3;

const AxisHardware& raycube_axis(Axis axis);

}  // namespace ray2::machine
