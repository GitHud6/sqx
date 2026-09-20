#include "segment_producer.hpp"

#include "timer_stepper_executor.hpp"

namespace laserctl::stepper {
namespace {

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
    return publish_available_segments();
}

bool segment_producer_start() {
    return timer_stepper_executor_start_from_segment_ring() == ESP_OK;
}

bool segment_producer_has_pending_segments() {
    return stepper_segment_count() != 0;
}

bool segment_producer_can_accept_plan() {
    return stepper_prep_can_accept_block();
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

}  // namespace laserctl::stepper
