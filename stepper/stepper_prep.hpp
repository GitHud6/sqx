#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "planner.hpp"

namespace ray2::stepper {

constexpr std::size_t kStepSegmentBufferLength = 64;
constexpr std::uint32_t kStepSegmentTargetDurationUs = 750;

enum class StepSegmentPhase : std::uint8_t {
    Accelerate,
    Cruise,
    Decelerate,
};

struct StepSegment {
    std::uint64_t execution_id = 0;
    std::array<std::int32_t, 2> axis_steps = {};
    std::int32_t major_steps = 0;
    std::uint32_t duration_us = 0;
    double start_speed_mm_per_min = 0.0;
    double end_speed_mm_per_min = 0.0;
    double laser_power = 0.0;
    StepSegmentPhase phase = StepSegmentPhase::Cruise;
    bool completes_block = false;
};

struct StepperProfile {
    std::uint64_t execution_id = 0;
    std::int32_t total_major_steps = 0;
    std::uint32_t total_segments = 0;
    std::uint32_t next_segment_index = 0;
    double length_mm = 0.0;
    double entry_speed_mm_per_min = 0.0;
    double nominal_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double acceleration_mm_per_min2 = 0.0;
    double accelerate_distance_mm = 0.0;
    double decelerate_after_mm = 0.0;
    bool active = false;
};

void stepper_prep_reset();
bool stepper_prep_start_block(const planner::PlannerBlock& block);
void stepper_prep_cancel_block();
std::size_t stepper_prep_fill();
bool stepper_segment_pop(StepSegment& segment);
std::size_t stepper_segment_count();
const StepperProfile& stepper_prep_profile();

}  // namespace ray2::stepper
