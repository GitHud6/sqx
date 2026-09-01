#pragma once

#include <cstdint>

#include "motion_profile.hpp"

namespace ray2::stepper {

// Pure reference implementation of ray2-main Stepper.cpp::st_prep_buffer()
// normal-motion segment math. It owns no queue, GPIO, timer, or RMT state.
enum class GrblTrapezoidRamp : std::uint8_t {
    Accelerate,
    Cruise,
    Decelerate,
};

struct GrblTrapezoidPrepSlice {
    std::uint32_t major_steps = 0;
    std::uint32_t timer_ticks = 0;
    std::uint8_t amass_level = 0;
    double duration_us = 0.0;
    double dt_remainder_us = 0.0;
    bool completes_profile = false;
};

struct GrblTrapezoidPrep {
    MotionProfileInput input = {};
    std::uint32_t total_major_steps = 0;
    std::uint32_t emitted_major_steps = 0;
    std::uint32_t segment_duration_us = 1000;
    std::uint32_t timer_frequency_hz = 20000000;
    std::uint32_t amass_threshold_ticks = 2500;
    std::uint8_t max_amass_level = 3;
    double steps_remaining = 0.0;
    double step_per_mm = 0.0;
    double required_mm_increment = 0.0;
    double dt_remainder_minutes = 0.0;
    double mm_remaining = 0.0;
    double current_speed_mm_per_min = 0.0;
    double maximum_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double accelerate_until_mm_remaining = 0.0;
    double decelerate_after_mm_remaining = 0.0;
    GrblTrapezoidRamp ramp = GrblTrapezoidRamp::Accelerate;
    bool complete = false;
};

struct GrblTrapezoidPrepTestCase {
    std::uint32_t expected_steps = 0;
    std::uint32_t emitted_steps = 0;
    std::uint32_t slice_count = 0;
    std::uint32_t zero_step_slice_count = 0;
    double total_duration_us = 0.0;
    double maximum_remainder_us = 0.0;
    bool passed = false;
};

struct GrblTrapezoidPrepSelfTestReport {
    GrblTrapezoidPrepTestCase cases[3] = {};
    bool passed = false;
};

bool grbl_trapezoid_prep_init(GrblTrapezoidPrep& prep,
                              const MotionProfileInput& input,
                              std::uint32_t total_major_steps,
                              std::uint32_t segment_duration_us = 1000,
                              std::uint32_t timer_frequency_hz = 20000000);
bool grbl_trapezoid_prep_next(GrblTrapezoidPrep& prep, GrblTrapezoidPrepSlice& slice);
bool grbl_trapezoid_prep_self_test(GrblTrapezoidPrepSelfTestReport& report);

}  // namespace ray2::stepper
