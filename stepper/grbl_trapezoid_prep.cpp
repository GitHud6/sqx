#include "grbl_trapezoid_prep.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ray2::stepper {
namespace {

constexpr double kMinutesToMicroseconds = 60000000.0;
constexpr double kRequiredMmIncrementScalar = 1.25;
constexpr double kCompletionEpsilonMm = 1e-9;

bool valid_input(const MotionProfileInput& input, const std::uint32_t total_steps,
                 const std::uint32_t duration_us, const std::uint32_t timer_frequency_hz) {
    return total_steps != 0 && duration_us != 0 && timer_frequency_hz != 0 &&
           std::isfinite(input.length_mm) && input.length_mm > 0.0 &&
           std::isfinite(input.entry_speed_mm_per_min) && input.entry_speed_mm_per_min >= 0.0 &&
           std::isfinite(input.nominal_speed_mm_per_min) && input.nominal_speed_mm_per_min > 0.0 &&
           std::isfinite(input.exit_speed_mm_per_min) && input.exit_speed_mm_per_min >= 0.0 &&
           std::isfinite(input.acceleration_mm_per_min2) && input.acceleration_mm_per_min2 > 0.0;
}

}  // namespace

bool grbl_trapezoid_prep_init(GrblTrapezoidPrep& prep,
                              const MotionProfileInput& input,
                              const std::uint32_t total_major_steps,
                              const std::uint32_t segment_duration_us,
                              const std::uint32_t timer_frequency_hz) {
    if (!valid_input(input, total_major_steps, segment_duration_us, timer_frequency_hz)) return false;

    prep = {};
    prep.input = input;
    prep.total_major_steps = total_major_steps;
    prep.segment_duration_us = segment_duration_us;
    prep.timer_frequency_hz = timer_frequency_hz;
    prep.amass_threshold_ticks = timer_frequency_hz / 8000U;
    prep.steps_remaining = static_cast<double>(total_major_steps);
    prep.step_per_mm = prep.steps_remaining / input.length_mm;
    prep.required_mm_increment = kRequiredMmIncrementScalar / prep.step_per_mm;
    prep.mm_remaining = input.length_mm;
    prep.current_speed_mm_per_min = input.entry_speed_mm_per_min;
    prep.exit_speed_mm_per_min = input.exit_speed_mm_per_min;
    prep.accelerate_until_mm_remaining = input.length_mm;

    const double inverse_two_acceleration = 0.5 / input.acceleration_mm_per_min2;
    const double entry_sqr = input.entry_speed_mm_per_min * input.entry_speed_mm_per_min;
    const double exit_sqr = input.exit_speed_mm_per_min * input.exit_speed_mm_per_min;
    const double nominal_sqr = input.nominal_speed_mm_per_min * input.nominal_speed_mm_per_min;
    const double intersect_distance =
        0.5 * (input.length_mm + inverse_two_acceleration * (entry_sqr - exit_sqr));

    if (intersect_distance > 0.0) {
        if (intersect_distance < input.length_mm) {
            prep.decelerate_after_mm_remaining = inverse_two_acceleration * (nominal_sqr - exit_sqr);
            if (prep.decelerate_after_mm_remaining < intersect_distance) {
                prep.maximum_speed_mm_per_min = input.nominal_speed_mm_per_min;
                if (entry_sqr == nominal_sqr) {
                    prep.ramp = GrblTrapezoidRamp::Cruise;
                } else {
                    prep.accelerate_until_mm_remaining -=
                        inverse_two_acceleration * (nominal_sqr - entry_sqr);
                }
            } else {
                prep.accelerate_until_mm_remaining = intersect_distance;
                prep.decelerate_after_mm_remaining = intersect_distance;
                prep.maximum_speed_mm_per_min = std::sqrt(
                    2.0 * input.acceleration_mm_per_min2 * intersect_distance + exit_sqr);
            }
        } else {
            prep.ramp = GrblTrapezoidRamp::Decelerate;
            prep.maximum_speed_mm_per_min = input.entry_speed_mm_per_min;
        }
    } else {
        prep.accelerate_until_mm_remaining = 0.0;
        prep.maximum_speed_mm_per_min = input.exit_speed_mm_per_min;
    }
    return std::isfinite(prep.maximum_speed_mm_per_min);
}

bool grbl_trapezoid_prep_next(GrblTrapezoidPrep& prep, GrblTrapezoidPrepSlice& slice) {
    if (prep.complete || prep.emitted_major_steps >= prep.total_major_steps) {
        prep.complete = true;
        return false;
    }

    const double acceleration = prep.input.acceleration_mm_per_min2;
    const double dt_segment = static_cast<double>(prep.segment_duration_us) / kMinutesToMicroseconds;
    double dt_max = dt_segment;
    double dt = 0.0;
    double mm_remaining = prep.mm_remaining;
    const double start_mm_remaining = mm_remaining;
    const double start_speed = prep.current_speed_mm_per_min;
    const GrblTrapezoidRamp start_ramp = prep.ramp;
    const double minimum_mm = std::max(0.0, mm_remaining - prep.required_mm_increment);

    for (std::uint32_t guard = 0; guard < 4096; ++guard) {
        double time_var = dt_max - dt;
        double speed_var = 0.0;
        double mm_var = 0.0;
        switch (prep.ramp) {
            case GrblTrapezoidRamp::Accelerate:
                speed_var = acceleration * time_var;
                mm_remaining -= time_var * (prep.current_speed_mm_per_min + 0.5 * speed_var);
                if (mm_remaining < prep.accelerate_until_mm_remaining) {
                    mm_remaining = prep.accelerate_until_mm_remaining;
                    time_var = 2.0 * (prep.mm_remaining - mm_remaining) /
                               (prep.current_speed_mm_per_min + prep.maximum_speed_mm_per_min);
                    prep.ramp = mm_remaining == prep.decelerate_after_mm_remaining
                                    ? GrblTrapezoidRamp::Decelerate
                                    : GrblTrapezoidRamp::Cruise;
                    prep.current_speed_mm_per_min = prep.maximum_speed_mm_per_min;
                } else {
                    prep.current_speed_mm_per_min += speed_var;
                }
                break;
            case GrblTrapezoidRamp::Cruise:
                mm_var = mm_remaining - prep.maximum_speed_mm_per_min * time_var;
                if (mm_var < prep.decelerate_after_mm_remaining) {
                    time_var = (mm_remaining - prep.decelerate_after_mm_remaining) /
                               prep.maximum_speed_mm_per_min;
                    mm_remaining = prep.decelerate_after_mm_remaining;
                    prep.ramp = GrblTrapezoidRamp::Decelerate;
                } else {
                    mm_remaining = mm_var;
                }
                break;
            case GrblTrapezoidRamp::Decelerate:
                speed_var = acceleration * time_var;
                if (prep.current_speed_mm_per_min > speed_var) {
                    mm_var = mm_remaining -
                             time_var * (prep.current_speed_mm_per_min - 0.5 * speed_var);
                    if (mm_var > 0.0) {
                        mm_remaining = mm_var;
                        prep.current_speed_mm_per_min -= speed_var;
                        break;
                    }
                }
                time_var = 2.0 * mm_remaining /
                           (prep.current_speed_mm_per_min + prep.exit_speed_mm_per_min);
                mm_remaining = 0.0;
                prep.current_speed_mm_per_min = prep.exit_speed_mm_per_min;
                break;
        }

        dt += time_var;
        if (dt >= dt_max) {
            if (mm_remaining <= minimum_mm || mm_remaining <= kCompletionEpsilonMm) break;
            dt_max += dt_segment;
        }
        if (mm_remaining <= kCompletionEpsilonMm) break;
    }

    mm_remaining = std::clamp(mm_remaining, 0.0, prep.mm_remaining);
    const double step_distance_remaining = prep.step_per_mm * mm_remaining;
    const double last_steps_remaining = std::ceil(prep.steps_remaining);
    const double next_steps_remaining = std::ceil(step_distance_remaining);
    const double base_steps_double = last_steps_remaining - next_steps_remaining;
    if (!std::isfinite(dt) || !std::isfinite(base_steps_double) || base_steps_double < 0.0) return false;

    std::uint32_t base_steps = static_cast<std::uint32_t>(base_steps_double);
    if (mm_remaining == 0.0) {
        base_steps = prep.total_major_steps - prep.emitted_major_steps;
    }
    // st_prep_buffer() publishes ordinary zero-step time slices. At low speed
    // this is required to preserve ramp time and dt_remainder progression.
    // Feed-hold-specific zero-step termination is intentionally outside this
    // offline normal-motion reference.

    dt += prep.dt_remainder_minutes;
    const double executed_step_span = last_steps_remaining - step_distance_remaining;
    if (!std::isfinite(executed_step_span) || executed_step_span <= 0.0) return false;
    const double inverse_rate = dt / executed_step_span;
    const double timer_ticks_exact = static_cast<double>(prep.timer_frequency_hz) * 60.0 * inverse_rate;
    if (!std::isfinite(timer_ticks_exact) || timer_ticks_exact <= 0.0 ||
        timer_ticks_exact > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }

    std::uint32_t timer_ticks = static_cast<std::uint32_t>(std::ceil(timer_ticks_exact));
    std::uint8_t amass_level = 0;
    while (amass_level < prep.max_amass_level && timer_ticks >= prep.amass_threshold_ticks) {
        ++amass_level;
        timer_ticks >>= 1U;
    }

    if (base_steps > prep.total_major_steps - prep.emitted_major_steps) return false;
    prep.emitted_major_steps += base_steps;
    prep.steps_remaining = next_steps_remaining;
    prep.mm_remaining = mm_remaining;
    prep.dt_remainder_minutes = mm_remaining == 0.0
                                    ? 0.0
                                    : (next_steps_remaining - step_distance_remaining) * inverse_rate;
    prep.complete = mm_remaining == 0.0 && prep.emitted_major_steps == prep.total_major_steps;
    slice = {
        .major_steps = base_steps,
        .emitted_major_steps = prep.emitted_major_steps,
        .timer_ticks = timer_ticks,
        .amass_level = amass_level,
        .duration_us = dt * kMinutesToMicroseconds,
        .dt_remainder_us = prep.dt_remainder_minutes * kMinutesToMicroseconds,
        .start_distance_mm = prep.input.length_mm - start_mm_remaining,
        .end_distance_mm = prep.input.length_mm - mm_remaining,
        .start_speed_mm_per_min = start_speed,
        .end_speed_mm_per_min = prep.current_speed_mm_per_min,
        .ramp = start_ramp,
        .completes_profile = prep.complete,
    };
    return true;
}

}  // namespace ray2::stepper
