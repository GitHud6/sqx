#pragma once

namespace laserctl::stepper {

enum class MotionProfileMode { Trapezoid, SCurve };
enum class MotionProfilePhase { Accelerate, Cruise, Decelerate };

struct MotionProfileInput {
    double length_mm = 0.0;
    double entry_speed_mm_per_min = 0.0;
    double nominal_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double acceleration_mm_per_min2 = 0.0;
};

// Pure profile data. It does not depend on queues, segments, GPIO, or FreeRTOS.
struct MotionProfile {
    MotionProfileMode mode = MotionProfileMode::Trapezoid;
    double length_mm = 0.0;
    double entry_speed_mm_per_min = 0.0;
    double nominal_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double acceleration_mm_per_min2 = 0.0;
    double accelerate_distance_mm = 0.0;
    double decelerate_after_mm = 0.0;
    double total_time_us = 0.0;
};

bool motion_profile_build_trapezoid(const MotionProfileInput& input, MotionProfile& profile);
double motion_profile_speed_at_distance(const MotionProfile& profile, double distance_mm);
double motion_profile_time_at_distance_us(const MotionProfile& profile, double distance_mm);
MotionProfilePhase motion_profile_phase_at_distance(const MotionProfile& profile, double distance_mm);
double motion_profile_distance_at_time_us(const MotionProfile& profile, double time_us);
double motion_profile_speed_at_time_us(const MotionProfile& profile, double time_us);
MotionProfilePhase motion_profile_phase_at_time_us(const MotionProfile& profile, double time_us);

}  // namespace laserctl::stepper
