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
esp_err_t rmt_stepper_stream_start(RmtSegmentCompletedCallback callback, void* user_data);
std::size_t rmt_stepper_stream_free_slots();
bool rmt_stepper_stream_is_busy();
esp_err_t rmt_stepper_stream_submit(const StepSegment* segments, std::size_t segment_count);
esp_err_t rmt_stepper_stream_drain();
void rmt_stepper_stream_reset();
void rmt_stepper_request_feed_hold();
void rmt_stepper_resume();
esp_err_t rmt_stepper_stop();

}  // namespace ray2::stepper
