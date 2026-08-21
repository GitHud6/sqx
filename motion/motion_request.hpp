#pragma once

#include <cstdint>

namespace ray2::motion {

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

struct MotionExecutionPlan {
    std::int32_t steps[3] = {};
    bool positive_direction[3] = {};
    std::uint32_t step_interval_us = 0;
};

}  // namespace ray2::motion
