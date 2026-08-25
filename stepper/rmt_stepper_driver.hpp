#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "raycube_machine.hpp"
#include "stepper_prep.hpp"

namespace ray2::stepper {

constexpr unsigned int kRmtStepResolutionHz = 1000000;
constexpr std::size_t kRmtStepMemorySymbols = 64;
constexpr std::size_t kRmtSegmentTransactionDepth = 64;
constexpr std::uint32_t kRmtStepMaxIntervalUs = 32767;

using RmtSegmentCompletedCallback = void (*)(const StepSegment& segment,
                                             bool completed,
                                             void* user_data);
using RmtSegmentStartedCallback = void (*)(const StepSegment& segment, void* user_data);

struct RmtStreamDiagnostics {
    std::size_t pending_segments = 0;
    std::size_t inflight_segments = 0;
    std::uint64_t submitted_segments = 0;
    std::uint64_t started_segments = 0;
    std::uint64_t completed_segments = 0;
    std::uint64_t failed_segments = 0;
    std::uint64_t timeout_segments = 0;
    bool active_segment = false;
    std::uint64_t active_execution_id = 0;
    std::int32_t active_x_steps = 0;
    std::int32_t active_y_steps = 0;
    std::int32_t active_major_steps = 0;
    std::uint32_t active_duration_us = 0;
    std::uint64_t active_elapsed_us = 0;
};

// This only allocates and enables RMT TX channels. It never transmits STEP pulses.
esp_err_t rmt_stepper_init();
esp_err_t rmt_stepper_deinit();
bool rmt_stepper_is_initialized();
bool rmt_stepper_can_execute(std::int32_t step_count, std::uint32_t step_interval_us);
bool rmt_stepper_can_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                std::uint32_t step_interval_us);
esp_err_t rmt_stepper_execute(machine::Axis axis,
                               std::int32_t step_count,
                               std::uint32_t step_interval_us);
esp_err_t rmt_stepper_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                 std::uint32_t step_interval_us);
bool rmt_stepper_can_execute_segments(const StepSegment* segments, std::size_t segment_count);
esp_err_t rmt_stepper_execute_segments(const StepSegment* segments, std::size_t segment_count);
esp_err_t rmt_stepper_stream_start(RmtSegmentCompletedCallback completed_callback,
                                   RmtSegmentStartedCallback started_callback,
                                   void* user_data);
std::size_t rmt_stepper_stream_free_slots();
bool rmt_stepper_stream_is_busy();
RmtStreamDiagnostics rmt_stepper_stream_diagnostics();
esp_err_t rmt_stepper_stream_submit(const StepSegment* segments, std::size_t segment_count);
esp_err_t rmt_stepper_stream_drain();
void rmt_stepper_stream_reset();
void rmt_stepper_request_feed_hold();
void rmt_stepper_resume();
esp_err_t rmt_stepper_stop();

}  // namespace ray2::stepper
