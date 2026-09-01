#include "trapezoid_segment_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ray2::stepper {
namespace {

bool valid_profile(const MotionProfile& profile) {
    return profile.mode == MotionProfileMode::Trapezoid &&
           std::isfinite(profile.length_mm) && profile.length_mm > 0.0 &&
           std::isfinite(profile.total_time_us) && profile.total_time_us > 0.0;
}

}  // namespace

bool trapezoid_segment_generator_init(TrapezoidSegmentGenerator& generator,
                                      const MotionProfile& profile,
                                      const std::uint32_t total_major_steps,
                                      const std::uint32_t segment_duration_us,
                                      const std::uint32_t timer_ticks_per_us) {
    if (!valid_profile(profile) || total_major_steps == 0 || segment_duration_us == 0 ||
        timer_ticks_per_us == 0) {
        return false;
    }
    generator = {
        .profile = profile,
        .total_major_steps = total_major_steps,
        .segment_duration_us = segment_duration_us,
        .timer_ticks_per_us = timer_ticks_per_us,
    };
    return true;
}

bool trapezoid_segment_generator_next(TrapezoidSegmentGenerator& generator,
                                      TrapezoidTimeSlice& slice) {
    if (generator.complete || generator.emitted_major_steps >= generator.total_major_steps) {
        generator.complete = true;
        return false;
    }

    const double start_time_us = generator.elapsed_time_us;
    double end_time_us = std::min(generator.profile.total_time_us,
                                  start_time_us + generator.segment_duration_us);
    std::uint32_t major_steps = 0;
    double end_distance_mm = generator.emitted_distance_mm;
    double continuous_steps_remaining = 0.0;

    // Low speeds may not contain a full step in one nominal time slice. Grow
    // the time window rather than publishing an invalid zero-step segment.
    while (true) {
        end_distance_mm = motion_profile_distance_at_time_us(generator.profile, end_time_us);
        const double ideal_total_steps = end_distance_mm * generator.total_major_steps /
                                         generator.profile.length_mm;
        continuous_steps_remaining = std::max(0.0, generator.total_major_steps - ideal_total_steps);
        const std::uint32_t last_quantized_remaining =
            generator.total_major_steps - generator.emitted_major_steps;
        const std::uint32_t quantized_remaining = end_time_us >= generator.profile.total_time_us
                                                      ? 0
                                                      : static_cast<std::uint32_t>(std::ceil(
                                                            continuous_steps_remaining - 1e-9));
        major_steps = last_quantized_remaining - quantized_remaining;
        if (major_steps != 0 || end_time_us >= generator.profile.total_time_us) break;
        end_time_us = std::min(generator.profile.total_time_us,
                               end_time_us + generator.segment_duration_us);
    }

    if (major_steps == 0) {
        generator.complete = true;
        return false;
    }

    const double physical_duration_us = end_time_us - start_time_us;
    const double corrected_duration_us = physical_duration_us + generator.dt_remainder_us;
    const double quantized_step_span =
        static_cast<double>(generator.total_major_steps - generator.emitted_major_steps) -
        continuous_steps_remaining;
    const double interval_ticks_exact = corrected_duration_us * generator.timer_ticks_per_us /
                                        quantized_step_span;
    if (!std::isfinite(interval_ticks_exact) || interval_ticks_exact <= 0.0 ||
        interval_ticks_exact > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }
    const std::uint32_t interval_ticks = static_cast<std::uint32_t>(
        std::ceil(interval_ticks_exact));
    slice = {
        .major_steps = major_steps,
        .duration_us = static_cast<std::uint32_t>(std::lround(physical_duration_us)),
        .step_interval_ticks = interval_ticks,
        .start_time_us = start_time_us,
        .end_time_us = end_time_us,
        .start_distance_mm = generator.emitted_distance_mm,
        .end_distance_mm = end_distance_mm,
        .start_speed_mm_per_min = motion_profile_speed_at_time_us(generator.profile, start_time_us),
        .end_speed_mm_per_min = motion_profile_speed_at_time_us(generator.profile, end_time_us),
        .phase = motion_profile_phase_at_time_us(generator.profile, start_time_us),
        .completes_profile = end_time_us >= generator.profile.total_time_us,
    };

    generator.elapsed_time_us = end_time_us;
    generator.emitted_distance_mm = end_distance_mm;
    generator.emitted_major_steps += major_steps;
    // Match the classic Grbl recurrence. The segment interval is calculated
    // over the integer steps plus this terminal fractional step, then that
    // fractional step's time is carried to the next segment.
    const std::uint32_t quantized_remaining = slice.completes_profile
                                                  ? 0
                                                  : static_cast<std::uint32_t>(std::ceil(
                                                        continuous_steps_remaining - 1e-9));
    const double pending_fractional_steps =
        static_cast<double>(quantized_remaining) - continuous_steps_remaining;
    const double ideal_step_duration_us = corrected_duration_us / quantized_step_span;
    generator.dt_remainder_us = slice.completes_profile ? 0.0 :
        std::max(0.0, pending_fractional_steps) * ideal_step_duration_us;
    generator.complete = slice.completes_profile;
    return true;
}

}  // namespace ray2::stepper
