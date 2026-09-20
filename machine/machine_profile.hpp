#pragma once

#include <cstddef>
#include <cstdint>

namespace laserctl::machine {

enum class Axis : std::size_t {
    X = 0,
    Y = 1,
};

struct AxisHardware {
    int step_pin;
    int direction_pin;
    int disable_pin;
    bool direction_inverted;
    // Physical direction that moves this axis toward its homing switch.
    bool homing_seek_positive_direction;
    bool disable_active_high;
    float steps_per_mm;
    float max_rate_mm_per_min;
    float acceleration_mm_per_sec2;
    float max_travel_mm;
    int limit_pin;
    bool limit_active_low;
};

struct LaserHardware {
    int pwm_pin;
    int enable_pin;
    bool pwm_inverted;
    bool enable_active_high;
    std::uint32_t pwm_frequency_hz;
    std::uint32_t max_power;
};

constexpr std::size_t kAxisCount = 2;
constexpr unsigned int kStepPulseMicroseconds = 3;

const AxisHardware& machine_axis(Axis axis);
const LaserHardware& machine_laser();

}  // namespace laserctl::machine
