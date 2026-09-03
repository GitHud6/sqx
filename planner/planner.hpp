#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "motion_request.hpp"
#include "stepper_driver.hpp"

namespace ray2::planner {

constexpr std::size_t kPlannerQueueLength = 32;
constexpr std::size_t kPlannerDiagnosticBlockCount = 3;
constexpr double kJunctionDeviationMm = 0.01;

enum class MotionBlockType {
    Motion,
    SpindleDisable,
    ProgramPause,
    Dwell,
};

enum class PlannerBlockState : std::uint8_t {
    Executing,
    Frozen,
    Lookahead,
};

// This is the planner-owned motion representation consumed by the stepper executor.
struct PlannerBlock {
    MotionBlockType type = MotionBlockType::Motion;
    motion::MotionRequest request = {};
    motion::MotionGeometry geometry = {};
    std::array<double, 2> machine_start_mm = {};
    std::array<double, 2> machine_target_mm = {};
    std::int32_t step_event_count = 0;
    double millimeters = 0.0;
    double programmed_rate_mm_per_min = 0.0;
    double acceleration_mm_per_sec2 = 0.0;
    double unit_vector[2] = {};
    double entry_speed_sqr = 0.0;
    double exit_speed_sqr = 0.0;
    double max_entry_speed_sqr = 0.0;
    double max_junction_speed_sqr = 0.0;
    PlannerBlockState state = PlannerBlockState::Lookahead;
    std::uint64_t execution_id = 0;
    std::uint32_t dwell_milliseconds = 0;
};

// Kept while callers migrate from the original block name.
using MotionBlock = PlannerBlock;

enum class SubmitResult {
    Accepted,
    Full,
    Rejected,
};

struct PlannerBlockDiagnostics {
    bool present = false;
    PlannerBlockState state = PlannerBlockState::Lookahead;
    std::uint64_t execution_id = 0;
    double entry_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double max_junction_speed_mm_per_min = 0.0;
    double programmed_rate_mm_per_min = 0.0;
};

// Captures the strongest non-terminal entry-to-exit velocity collapse among
// blocks handed to the profile layer. It is diagnostic-only.
struct PlannerVelocityCollapseDiagnostics {
    std::uint32_t samples = 0;
    std::uint32_t severe_samples = 0;
    // Severe entry-to-exit collapses, binned by turn angle:
    // [0,5), [5,45), [45,135), [135,175), [175,180] degrees.
    std::array<std::uint32_t, 5> severe_turn_bins = {};
    std::uint64_t execution_id = 0;
    std::uint64_t next_execution_id = 0;
    double length_mm = 0.0;
    double next_length_mm = 0.0;
    double entry_speed_mm_per_min = 0.0;
    double exit_speed_mm_per_min = 0.0;
    double nominal_speed_mm_per_min = 0.0;
    double next_entry_speed_mm_per_min = 0.0;
    double next_nominal_speed_mm_per_min = 0.0;
    double next_max_entry_speed_mm_per_min = 0.0;
    double next_max_junction_speed_mm_per_min = 0.0;
    double turn_degrees = 0.0;
};

struct PlannerDiagnostics {
    int planner_task_core = -1;
    std::uint32_t recalculate_calls = 0;
    std::uint32_t recalculate_max_cycles = 0;
    std::size_t queued_blocks = 0;
    std::size_t frozen_blocks = 0;
    std::uint64_t coalesced_short_blocks = 0;
    std::array<PlannerBlockDiagnostics, kPlannerDiagnosticBlockCount> blocks = {};
    PlannerVelocityCollapseDiagnostics velocity_collapse = {};
};

enum class ExecutionEventType {
    BlockStarted,
    SegmentBatchStarted,
    SegmentBatchCompleted,
    BlockCompleted,
    BlockCancelled,
    BlockFailed,
    QueueDrained,
};

struct ExecutionEvent {
    ExecutionEventType type = ExecutionEventType::QueueDrained;
    std::uint64_t execution_id = 0;
    motion::MotionRequest request = {};
    motion::MotionGeometry geometry = {};
    double segment_power = 0.0;
    double segment_start_speed_mm_per_min = 0.0;
    double segment_end_speed_mm_per_min = 0.0;
    bool segment_completes_block = false;
    stepper::MachinePosition executed_machine_position = {};
};

// Callbacks run in the caller's task and must return quickly without calling planner APIs.
using ExecutionCallback = void (*)(const ExecutionEvent& event, void* user_data);

esp_err_t planner_start();
SubmitResult planner_submit(const MotionBlock& block);
// Atomically appends all blocks or none. Used by decomposed motion commands
// such as an arc, where a partial prefix would corrupt parser position.
SubmitResult planner_submit_batch(const MotionBlock* blocks, std::size_t count);
SubmitResult planner_submit_spindle_disable();
SubmitResult planner_submit_program_pause();
SubmitResult planner_submit_dwell(std::uint32_t milliseconds);
void planner_set_execution_callback(ExecutionCallback callback, void* user_data);
void planner_reset();
bool planner_is_idle();
std::size_t planner_free_blocks();
void planner_wait_for_free_block();
std::uint64_t planner_completed_block_count();
PlannerDiagnostics planner_diagnostics();

}  // namespace ray2::planner
