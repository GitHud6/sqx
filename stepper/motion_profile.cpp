#include "motion_profile.hpp"

#include <algorithm>
#include <cmath>

namespace ray2::stepper {
namespace {

constexpr double kMinutesToMicroseconds = 60000000.0;

bool valid_input(const MotionProfileInput& input) {
    return std::isfinite(input.length_mm) && input.length_mm > 0.0 &&
           std::isfinite(input.entry_speed_mm_per_min) && input.entry_speed_mm_per_min >= 0.0 &&
           std::isfinite(input.nominal_speed_mm_per_min) && input.nominal_speed_mm_per_min > 0.0 &&
           std::isfinite(input.exit_speed_mm_per_min) && input.exit_speed_mm_per_min >= 0.0 &&
           std::isfinite(input.acceleration_mm_per_min2) && input.acceleration_mm_per_min2 > 0.0;
}

}  // namespace

bool motion_profile_build_trapezoid(const MotionProfileInput& input, MotionProfile& profile) {
    if (!valid_input(input)) return false;
    const double entry = input.entry_speed_mm_per_min;
    const double exit = input.exit_speed_mm_per_min;
    const double acceleration = input.acceleration_mm_per_min2;
    double peak = input.nominal_speed_mm_per_min;
    double accelerate = std::max(0.0, (peak * peak - entry * entry) / (2.0 * acceleration));
    double decelerate = std::max(0.0, (peak * peak - exit * exit) / (2.0 * acceleration));
    if (accelerate + decelerate > input.length_mm) {
        peak = std::sqrt(std::max(0.0, (2.0 * acceleration * input.length_mm + entry * entry + exit * exit) / 2.0));
        accelerate = std::max(0.0, (peak * peak - entry * entry) / (2.0 * acceleration));
        decelerate = std::max(0.0, (peak * peak - exit * exit) / (2.0 * acceleration));
    }
    const double cruise = std::max(0.0, input.length_mm - accelerate - decelerate);
    profile = {
        .mode = MotionProfileMode::Trapezoid,
        .length_mm = input.length_mm,
        .entry_speed_mm_per_min = entry,
        .nominal_speed_mm_per_min = peak,
        .exit_speed_mm_per_min = exit,
        .acceleration_mm_per_min2 = acceleration,
        .accelerate_distance_mm = accelerate,
        .decelerate_after_mm = input.length_mm - decelerate,
        .total_time_us = ((peak - entry) + (peak - exit)) * kMinutesToMicroseconds / acceleration +
                         cruise * kMinutesToMicroseconds / peak,
    };
    return true;
}

double motion_profile_speed_at_distance(const MotionProfile& profile, const double distance_mm) {
    const double distance = std::clamp(distance_mm, 0.0, profile.length_mm);
    if (distance < profile.accelerate_distance_mm) {
        return std::sqrt(profile.entry_speed_mm_per_min * profile.entry_speed_mm_per_min +
                         2.0 * profile.acceleration_mm_per_min2 * distance);
    }
    if (distance < profile.decelerate_after_mm) return profile.nominal_speed_mm_per_min;
    const double remaining = profile.length_mm - distance;
    return std::sqrt(std::max(profile.exit_speed_mm_per_min * profile.exit_speed_mm_per_min,
                              profile.exit_speed_mm_per_min * profile.exit_speed_mm_per_min +
                                  2.0 * profile.acceleration_mm_per_min2 * remaining));
}

double motion_profile_time_at_distance_us(const MotionProfile& profile, const double distance_mm) {
    const double distance = std::clamp(distance_mm, 0.0, profile.length_mm);
    const double acceleration_time = (profile.nominal_speed_mm_per_min - profile.entry_speed_mm_per_min) *
        kMinutesToMicroseconds / profile.acceleration_mm_per_min2;
    if (distance <= profile.accelerate_distance_mm) {
        return (motion_profile_speed_at_distance(profile, distance) - profile.entry_speed_mm_per_min) *
            kMinutesToMicroseconds / profile.acceleration_mm_per_min2;
    }
    if (distance <= profile.decelerate_after_mm) {
        return acceleration_time + (distance - profile.accelerate_distance_mm) *
            kMinutesToMicroseconds / profile.nominal_speed_mm_per_min;
    }
    const double cruise_time = (profile.decelerate_after_mm - profile.accelerate_distance_mm) *
        kMinutesToMicroseconds / profile.nominal_speed_mm_per_min;
    return acceleration_time + cruise_time +
        (profile.nominal_speed_mm_per_min - motion_profile_speed_at_distance(profile, distance)) *
            kMinutesToMicroseconds / profile.acceleration_mm_per_min2;
}

MotionProfilePhase motion_profile_phase_at_distance(const MotionProfile& profile, const double distance_mm) {
    if (distance_mm < profile.accelerate_distance_mm) return MotionProfilePhase::Accelerate;
    if (distance_mm < profile.decelerate_after_mm) return MotionProfilePhase::Cruise;
    return MotionProfilePhase::Decelerate;
}

}  // namespace ray2::stepper
