#include "segment_producer.hpp"

#include <atomic>

#include "timer_stepper_executor.hpp"

namespace ray2::stepper {
namespace {

std::atomic<std::uint64_t> s_refill_requests = 0;
std::atomic<std::uint64_t> s_refill_services = 0;
std::atomic<std::uint64_t> s_refill_segments = 0;

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
    return true;
}

std::size_t segment_producer_process_events(const std::uint32_t events) {
    if ((events & StepperTransportEventRefillRequested) == 0U) {
        return 0;
    }
    s_refill_requests.fetch_add(1, std::memory_order_relaxed);
    s_refill_services.fetch_add(1, std::memory_order_relaxed);
    const std::size_t generated = publish_available_segments();
    s_refill_segments.fetch_add(generated, std::memory_order_relaxed);
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

bool segment_producer_begin_feed_hold() {
    return stepper_prep_begin_feed_hold();
}

bool segment_producer_continue_feed_hold(const planner::PlannerBlock& block) {
    return stepper_prep_continue_feed_hold(block);
}

bool segment_producer_hold_boundary_pending() {
    return stepper_prep_hold_boundary_pending();
}

bool segment_producer_hold_complete() {
    return stepper_prep_hold_complete();
}

bool segment_producer_resume_from_hold() {
    return stepper_prep_resume_from_hold();
}

bool segment_producer_consume_completion(SegmentCompletion& completion) {
    return stepper_completion_consume(completion);
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
    };
}

}  // namespace ray2::stepper
