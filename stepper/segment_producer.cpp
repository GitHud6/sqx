#include "segment_producer.hpp"

#include <algorithm>
#include <atomic>

#include "esp_cpu.h"

#include "timer_stepper_executor.hpp"

namespace ray2::stepper {
namespace {

std::atomic<std::uint64_t> s_refill_requests = 0;
std::atomic<std::uint64_t> s_refill_services = 0;
std::atomic<std::uint64_t> s_refill_segments = 0;
std::atomic<std::uint64_t> s_refill_empty_services = 0;
std::atomic<std::uint64_t> s_refill_empty_ring_full = 0;
std::atomic<std::uint64_t> s_refill_empty_profile_inactive = 0;
std::atomic<std::uint64_t> s_refill_empty_generation_complete = 0;
std::atomic<std::uint64_t> s_refill_empty_unavailable = 0;
std::atomic<std::uint64_t> s_refill_service_total_cycles = 0;
std::atomic<std::uint64_t> s_refill_service_max_cycles = 0;
std::atomic<std::uint64_t> s_refill_gap_min_cycles = UINT64_MAX;
std::atomic<std::uint64_t> s_refill_gap_max_cycles = 0;
std::atomic<std::uint32_t> s_refill_last_service_cycles = 0;
std::atomic<std::size_t> s_observed_low_water_segments = kStepSegmentBufferLength;
std::atomic<std::size_t> s_refill_batch_min_segments = kStepSegmentBufferLength;
std::atomic<std::size_t> s_refill_batch_max_segments = 0;

template <typename T>
void record_max(std::atomic<T>& maximum, const T value) {
    T observed = maximum.load(std::memory_order_relaxed);
    while (observed < value &&
           !maximum.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
    }
}

template <typename T>
void record_min(std::atomic<T>& minimum, const T value) {
    T observed = minimum.load(std::memory_order_relaxed);
    while (observed > value &&
           !minimum.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
    }
}

void observe_ring_watermark() {
    record_min(s_observed_low_water_segments, stepper_segment_count());
}

std::size_t publish_available_segments() {
    std::size_t generated = 0;
    while (stepper_segment_free_slots() != 0) {
        StepSegment segment = {};
        if (!stepper_prep_next_segment(segment)) break;
        if (!stepper_segment_publish(segment)) break;
        ++generated;
    }
    return generated;
}

}  // namespace

void segment_producer_reset() {
    s_refill_requests.store(0, std::memory_order_relaxed);
    s_refill_services.store(0, std::memory_order_relaxed);
    s_refill_segments.store(0, std::memory_order_relaxed);
    s_refill_empty_services.store(0, std::memory_order_relaxed);
    s_refill_empty_ring_full.store(0, std::memory_order_relaxed);
    s_refill_empty_profile_inactive.store(0, std::memory_order_relaxed);
    s_refill_empty_generation_complete.store(0, std::memory_order_relaxed);
    s_refill_empty_unavailable.store(0, std::memory_order_relaxed);
    s_refill_service_total_cycles.store(0, std::memory_order_relaxed);
    s_refill_service_max_cycles.store(0, std::memory_order_relaxed);
    s_refill_gap_min_cycles.store(UINT64_MAX, std::memory_order_relaxed);
    s_refill_gap_max_cycles.store(0, std::memory_order_relaxed);
    s_refill_last_service_cycles.store(0, std::memory_order_relaxed);
    s_observed_low_water_segments.store(kStepSegmentBufferLength, std::memory_order_relaxed);
    s_refill_batch_min_segments.store(kStepSegmentBufferLength, std::memory_order_relaxed);
    s_refill_batch_max_segments.store(0, std::memory_order_relaxed);
    stepper_segment_ring_reset();
    stepper_prep_reset();
}

void segment_producer_abort() {
    stepper_segment_ring_reset();
    stepper_prep_abort();
}

void segment_producer_set_notification_task(const TaskHandle_t task) {
    stepper_completion_set_notification_task(task);
}

std::uint32_t segment_producer_take_events() {
    return stepper_transport_take_events();
}

bool segment_producer_submit_plan(const planner::PlannerBlock& block) {
    if (!stepper_prep_start_block(block)) {
        return false;
    }
    // The first publish must not depend on an ISR-generated low-water event.
    // Later refills are driven only by StepperTransportEventRefillRequested.
    publish_available_segments();
    observe_ring_watermark();
    return true;
}

std::size_t segment_producer_process_events(const std::uint32_t events) {
    if ((events & StepperTransportEventRefillRequested) == 0U) {
        return 0;
    }
    s_refill_requests.fetch_add(1, std::memory_order_relaxed);
    s_refill_services.fetch_add(1, std::memory_order_relaxed);
    const std::uint32_t service_started_cycles = esp_cpu_get_cycle_count();
    const std::uint32_t previous_service_cycles =
        s_refill_last_service_cycles.exchange(service_started_cycles, std::memory_order_relaxed);
    if (previous_service_cycles != 0) {
        const std::uint64_t gap_cycles = service_started_cycles - previous_service_cycles;
        record_min(s_refill_gap_min_cycles, gap_cycles);
        record_max(s_refill_gap_max_cycles, gap_cycles);
    }
    const std::size_t free_slots_before_refill = stepper_segment_free_slots();
    const StepperPrepDiagnostics prep_before_refill = stepper_prep_diagnostics();
    observe_ring_watermark();
    const std::size_t generated = publish_available_segments();
    const std::uint64_t service_duration_cycles =
        esp_cpu_get_cycle_count() - service_started_cycles;
    s_refill_segments.fetch_add(generated, std::memory_order_relaxed);
    s_refill_service_total_cycles.fetch_add(service_duration_cycles, std::memory_order_relaxed);
    record_max(s_refill_service_max_cycles, service_duration_cycles);
    if (generated == 0) {
        s_refill_empty_services.fetch_add(1, std::memory_order_relaxed);
        if (free_slots_before_refill == 0) {
            s_refill_empty_ring_full.fetch_add(1, std::memory_order_relaxed);
        } else if (!prep_before_refill.active) {
            s_refill_empty_profile_inactive.fetch_add(1, std::memory_order_relaxed);
        } else if (prep_before_refill.generation_complete) {
            s_refill_empty_generation_complete.fetch_add(1, std::memory_order_relaxed);
        } else {
            s_refill_empty_unavailable.fetch_add(1, std::memory_order_relaxed);
        }
    } else {
        record_min(s_refill_batch_min_segments, generated);
        record_max(s_refill_batch_max_segments, generated);
    }
    observe_ring_watermark();
    return generated;
}

bool segment_producer_start() {
    return timer_stepper_executor_start_from_segment_ring() == ESP_OK;
}

bool segment_producer_has_pending_segments() {
    return stepper_segment_count() != 0;
}

bool segment_producer_can_accept_plan() {
    const StepperPrepDiagnostics diagnostics = stepper_prep_diagnostics();
    return diagnostics.context_used < diagnostics.context_capacity;
}

bool segment_producer_consume_completion(SegmentCompletion& completion) {
    return stepper_completion_consume(completion);
}

bool segment_producer_record_segment_completed(const SegmentCompletion& completion) {
    return stepper_prep_record_segment_completed(completion);
}

bool segment_producer_complete_plan(const std::uint64_t execution_id) {
    return stepper_prep_complete_block(execution_id);
}

void segment_producer_discard_pending() {
    stepper_segment_discard_pending();
}

const StepperProfile& segment_producer_profile() {
    return stepper_prep_profile();
}

StepperPrepDiagnostics segment_producer_diagnostics() {
    return stepper_prep_diagnostics();
}

SegmentExecutionDiagnostics segment_producer_transport_diagnostics() {
    return stepper_segment_diagnostics();
}

SegmentProducerDiagnostics segment_producer_diagnostics_transport() {
    return {
        .refill_requests = s_refill_requests.load(std::memory_order_relaxed),
        .refill_services = s_refill_services.load(std::memory_order_relaxed),
        .refill_segments = s_refill_segments.load(std::memory_order_relaxed),
        .refill_empty_services = s_refill_empty_services.load(std::memory_order_relaxed),
        .refill_empty_ring_full = s_refill_empty_ring_full.load(std::memory_order_relaxed),
        .refill_empty_profile_inactive =
            s_refill_empty_profile_inactive.load(std::memory_order_relaxed),
        .refill_empty_generation_complete =
            s_refill_empty_generation_complete.load(std::memory_order_relaxed),
        .refill_empty_unavailable = s_refill_empty_unavailable.load(std::memory_order_relaxed),
        .refill_service_total_cycles =
            s_refill_service_total_cycles.load(std::memory_order_relaxed),
        .refill_service_max_cycles = s_refill_service_max_cycles.load(std::memory_order_relaxed),
        .refill_gap_min_cycles = s_refill_gap_min_cycles.load(std::memory_order_relaxed),
        .refill_gap_max_cycles = s_refill_gap_max_cycles.load(std::memory_order_relaxed),
        .observed_low_water_segments =
            s_observed_low_water_segments.load(std::memory_order_relaxed),
        .refill_batch_min_segments = s_refill_batch_min_segments.load(std::memory_order_relaxed),
        .refill_batch_max_segments = s_refill_batch_max_segments.load(std::memory_order_relaxed),
    };
}

}  // namespace ray2::stepper
