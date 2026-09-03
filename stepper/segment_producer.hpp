#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "segment_ring.hpp"
#include "planner.hpp"
#include "stepper_prep.hpp"

namespace ray2::stepper {

struct SegmentProducerDiagnostics {
    std::uint64_t refill_requests = 0;
    std::uint64_t refill_services = 0;
    std::uint64_t refill_segments = 0;
    std::uint64_t refill_empty_services = 0;
    std::uint64_t refill_empty_ring_full = 0;
    std::uint64_t refill_empty_profile_inactive = 0;
    std::uint64_t refill_empty_generation_complete = 0;
    std::uint64_t refill_empty_unavailable = 0;
    std::uint64_t refill_service_total_cycles = 0;
    std::uint64_t refill_service_max_cycles = 0;
    std::uint64_t refill_gap_min_cycles = 0;
    std::uint64_t refill_gap_max_cycles = 0;
    std::size_t observed_low_water_segments = kStepSegmentBufferLength;
    std::size_t refill_batch_min_segments = 0;
    std::size_t refill_batch_max_segments = 0;
};

// The producer is the planner-facing transport boundary. It is intentionally
// single-task for now: prep contexts are not yet safe for concurrent ownership.
void segment_producer_reset();
void segment_producer_abort();
void segment_producer_set_notification_task(TaskHandle_t task);
std::uint32_t segment_producer_take_events();

bool segment_producer_submit_plan(const planner::PlannerBlock& block);
std::size_t segment_producer_process_events(std::uint32_t events);
bool segment_producer_start();
bool segment_producer_has_pending_segments();
// Admission is constrained by immutable plan context capacity, not disposable
// segment-ring capacity. This keeps successor plans ready before a boundary.
bool segment_producer_can_accept_plan();

bool segment_producer_consume_completion(SegmentCompletion& completion);
bool segment_producer_record_segment_completed(const SegmentCompletion& completion);
bool segment_producer_complete_plan(std::uint64_t execution_id);
void segment_producer_discard_pending();

const StepperProfile& segment_producer_profile();
StepperPrepDiagnostics segment_producer_diagnostics();
SegmentExecutionDiagnostics segment_producer_transport_diagnostics();
SegmentProducerDiagnostics segment_producer_diagnostics_transport();

}  // namespace ray2::stepper
