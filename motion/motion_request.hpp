#pragma once

#include <cstdint>

namespace laserctl::motion {

enum class MotionType {
    Rapid,
    Linear,
};

enum class SpindleMode {
    Off,
    Clockwise,
    CounterClockwise,
};

struct MotionRequest {
    MotionType motion_mode = MotionType::Rapid;
    double target_mm[3] = {};
    double start_mm[3] = {};
    double feed_rate_mm_per_min = 0.0;
    double spindle_power = 0.0;
    SpindleMode spindle_mode = SpindleMode::Off;
    bool has_motion = false;
};

// Geometry quantized by the motion boundary. Timing belongs to the profile
// and stepper-prep layers, not to the G-code submission path.
struct MotionGeometry {
    std::int32_t steps[3] = {};
    bool positive_direction[3] = {};
};

}  // namespace laserctl::motion
