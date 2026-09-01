#pragma once

#include <cstddef>
#include <cstdint>

#include "motion_profile.hpp"

namespace ray2::stepper {

// Pure fixed-time segment generator. It owns no ring, GPIO, FreeRTOS state,
// or executor state. The caller maps its major-axis output to X/Y steps.
struct TrapezoidTimeSlice {
    std::uint32_t major_steps = 0;
    std::uint32_t duration_us = 0;
    std::uint32_t step_interval_ticks = 0;
    double start_time_us = 0.0;
    double end_time_us = 0.0;
    double start_distance_mm = 0.0;
    double end_distance_mm = 0.0;
    double start_speed_mm_per_min = 0.0;
    double end_speed_mm_per_min = 0.0;
    MotionProfilePhase phase = MotionProfilePhase::Cruise;
    bool completes_profile = false;
};

struct TrapezoidSegmentGenerator {
    MotionProfile profile = {};
    std::uint32_t total_major_steps = 0;
    std::uint32_t emitted_major_steps = 0;
    double elapsed_time_us = 0.0;
    double emitted_distance_mm = 0.0;
    double dt_remainder_us = 0.0;
    std::uint32_t segment_duration_us = 1000;
    std::uint32_t timer_ticks_per_us = 20;
    bool complete = false;
};

struct TrapezoidGeneratorTestCase {
    std::uint32_t expected_steps = 0;
    std::uint32_t emitted_steps = 0;
    std::uint32_t slice_count = 0;
    double final_time_us = 0.0;
    double expected_time_us = 0.0;
    double emitted_pulse_time_us = 0.0;
    double pulse_time_error_us = 0.0;
    double maximum_remainder_us = 0.0;
    bool passed = false;
};

struct TrapezoidGeneratorSelfTestReport {
    TrapezoidGeneratorTestCase cases[3] = {};
    bool passed = false;
};

bool trapezoid_segment_generator_init(TrapezoidSegmentGenerator& generator,
                                      const MotionProfile& profile,
                                      std::uint32_t total_major_steps,
                                      std::uint32_t segment_duration_us,
                                      std::uint32_t timer_ticks_per_us);

bool trapezoid_segment_generator_next(TrapezoidSegmentGenerator& generator,
                                      TrapezoidTimeSlice& slice);

bool trapezoid_segment_generator_self_test_report(TrapezoidGeneratorSelfTestReport& report);
bool trapezoid_segment_generator_self_test();

// Formats a compact, protocol-safe diagnostic response without performing I/O.
bool trapezoid_segment_generator_format_self_test(char* buffer, std::size_t buffer_size);

}  // namespace ray2::stepper
