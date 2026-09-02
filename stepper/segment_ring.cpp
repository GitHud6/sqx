#include "segment_ring.hpp"

#include <algorithm>
#include <atomic>

namespace ray2::stepper {
namespace {

std::array<StepSegment, kStepSegmentBufferLength> s_segment_buffer = {};
std::atomic<std::uint32_t> s_segment_head = 0;
std::atomic<std::uint32_t> s_segment_tail = 0;
std::atomic<std::size_t> s_segment_high_water = 0;
std::atomic<std::uint64_t> s_segment_underflow_count = 0;
std::atomic<std::uint64_t> s_block_boundary_stop_count = 0;
std::atomic<std::uint64_t> s_block_boundary_restart_count = 0;
std::atomic<bool> s_block_boundary_restart_pending = false;
std::atomic<std::uint64_t> s_active_execution_id = 0;
std::atomic<std::uint32_t> s_active_step_interval_us = 0;
std::atomic<std::uint32_t> s_active_amass_timer_interval_us = 0;
std::atomic<std::uint8_t> s_active_amass_level = 0;
std::atomic<std::uint32_t> s_active_start_speed_tenths_mm_per_min = 0;
std::atomic<std::uint32_t> s_active_end_speed_tenths_mm_per_min = 0;

std::array<SegmentCompletion, kStepCompletionBufferLength> s_completion_buffer = {};
std::atomic<std::uint32_t> s_completion_head = 0;
std::atomic<std::uint32_t> s_completion_tail = 0;
std::atomic<TaskHandle_t> s_completion_notification_task = nullptr;
std::atomic<std::uint32_t> s_transport_events = StepperTransportEventNone;

std::size_t segment_count_snapshot() {
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_acquire);
    return std::min<std::size_t>(head - tail, kStepSegmentBufferLength);
}

std::size_t completion_count_snapshot() {
    const std::uint32_t head = s_completion_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_acquire);
    return std::min<std::size_t>(head - tail, kStepCompletionBufferLength);
}

void update_segment_high_water(const std::size_t count) {
    std::size_t observed = s_segment_high_water.load(std::memory_order_relaxed);
    while (count > observed &&
           !s_segment_high_water.compare_exchange_weak(observed, count,
                                                       std::memory_order_relaxed,
                                                       std::memory_order_relaxed)) {
    }
}

}  // namespace

void stepper_segment_ring_reset() {
    s_segment_head.store(0, std::memory_order_relaxed);
    s_segment_tail.store(0, std::memory_order_relaxed);
    s_segment_high_water.store(0, std::memory_order_relaxed);
    s_segment_underflow_count.store(0, std::memory_order_relaxed);
    s_block_boundary_stop_count.store(0, std::memory_order_relaxed);
    s_block_boundary_restart_count.store(0, std::memory_order_relaxed);
    s_block_boundary_restart_pending.store(false, std::memory_order_relaxed);
    s_active_execution_id.store(0, std::memory_order_relaxed);
    s_active_step_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_level.store(0, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_completion_head.store(0, std::memory_order_relaxed);
    s_completion_tail.store(0, std::memory_order_relaxed);
    s_transport_events.store(StepperTransportEventNone, std::memory_order_relaxed);
}

bool stepper_segment_publish(const StepSegment& segment) {
    const std::uint32_t head = s_segment_head.load(std::memory_order_relaxed);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_acquire);
    if (head - tail >= kStepSegmentBufferLength) return false;
    s_segment_buffer[head % kStepSegmentBufferLength] = segment;
    s_segment_head.store(head + 1, std::memory_order_release);
    update_segment_high_water(head + 1 - tail);
    return true;
}

bool IRAM_ATTR stepper_segment_consume(StepSegment& segment) {
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_relaxed);
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    if (tail == head) return false;
    segment = s_segment_buffer[tail % kStepSegmentBufferLength];
    s_segment_tail.store(tail + 1, std::memory_order_release);
    return true;
}

void stepper_segment_discard_pending() {
    s_segment_tail.store(s_segment_head.load(std::memory_order_acquire),
                         std::memory_order_release);
}

std::size_t stepper_segment_count() { return segment_count_snapshot(); }
std::size_t stepper_segment_free_slots() {
    return kStepSegmentBufferLength - segment_count_snapshot();
}
void IRAM_ATTR stepper_segment_record_underflow_from_isr() {
    s_segment_underflow_count.fetch_add(1, std::memory_order_relaxed);
}
void IRAM_ATTR stepper_segment_record_block_boundary_stop_from_isr() {
    s_block_boundary_stop_count.fetch_add(1, std::memory_order_relaxed);
    s_block_boundary_restart_pending.store(true, std::memory_order_release);
}
void stepper_segment_record_block_boundary_restart() {
    if (s_block_boundary_restart_pending.exchange(false, std::memory_order_acq_rel)) {
        s_block_boundary_restart_count.fetch_add(1, std::memory_order_relaxed);
    }
}
void stepper_segment_clear_block_boundary_pending() {
    s_block_boundary_restart_pending.store(false, std::memory_order_release);
}
bool IRAM_ATTR stepper_segment_needs_refill_from_isr() {
    const std::uint32_t head = s_segment_head.load(std::memory_order_acquire);
    const std::uint32_t tail = s_segment_tail.load(std::memory_order_relaxed);
    return head - tail == kStepSegmentLowWater;
}
void IRAM_ATTR stepper_segment_request_refill_from_isr(BaseType_t* const higher_priority_task_woken) {
    s_transport_events.fetch_or(StepperTransportEventRefillRequested, std::memory_order_release);
    const TaskHandle_t task = s_completion_notification_task.load(std::memory_order_acquire);
    if (task != nullptr && higher_priority_task_woken != nullptr) {
        vTaskNotifyGiveFromISR(task, higher_priority_task_woken);
    }
}
void IRAM_ATTR stepper_segment_set_active(const StepSegment& segment) {
    s_active_step_interval_us.store(segment.step_interval_us, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(
        std::max<std::uint32_t>(4U, segment.step_interval_us / (1U << segment.amass_level)),
        std::memory_order_relaxed);
    s_active_amass_level.store(segment.amass_level, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(segment.start_speed_tenths_mm_per_min,
                                                  std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(segment.end_speed_tenths_mm_per_min,
                                                std::memory_order_relaxed);
    s_active_execution_id.store(segment.execution_id, std::memory_order_release);
}
void stepper_segment_clear_active_from_isr() {
    s_active_execution_id.store(0, std::memory_order_release);
    s_active_step_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_timer_interval_us.store(0, std::memory_order_relaxed);
    s_active_amass_level.store(0, std::memory_order_relaxed);
    s_active_start_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
    s_active_end_speed_tenths_mm_per_min.store(0, std::memory_order_relaxed);
}
void stepper_completion_set_notification_task(const TaskHandle_t task) {
    s_completion_notification_task.store(task, std::memory_order_release);
}
std::uint32_t stepper_transport_take_events() {
    return s_transport_events.exchange(StepperTransportEventNone, std::memory_order_acq_rel);
}
bool IRAM_ATTR stepper_completion_publish_from_isr(
    const SegmentCompletion& completion, BaseType_t* const higher_priority_task_woken) {
    const std::uint32_t head = s_completion_head.load(std::memory_order_relaxed);
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_acquire);
    if (head - tail >= kStepCompletionBufferLength) return false;
    s_completion_buffer[head % kStepCompletionBufferLength] = completion;
    s_completion_head.store(head + 1, std::memory_order_release);
    s_transport_events.fetch_or(StepperTransportEventCompletionAvailable, std::memory_order_release);
    const TaskHandle_t task = s_completion_notification_task.load(std::memory_order_acquire);
    if (task != nullptr && higher_priority_task_woken != nullptr) {
        vTaskNotifyGiveFromISR(task, higher_priority_task_woken);
    }
    return true;
}
bool stepper_completion_consume(SegmentCompletion& completion) {
    const std::uint32_t tail = s_completion_tail.load(std::memory_order_relaxed);
    const std::uint32_t head = s_completion_head.load(std::memory_order_acquire);
    if (tail == head) return false;
    completion = s_completion_buffer[tail % kStepCompletionBufferLength];
    s_completion_tail.store(tail + 1, std::memory_order_release);
    return true;
}
SegmentExecutionDiagnostics stepper_segment_diagnostics() {
    SegmentExecutionDiagnostics diagnostics = {};
    diagnostics.queued_segments = segment_count_snapshot();
    diagnostics.high_water_segments = s_segment_high_water.load(std::memory_order_relaxed);
    diagnostics.underflow_count = s_segment_underflow_count.load(std::memory_order_relaxed);
    diagnostics.block_boundary_stop_count = s_block_boundary_stop_count.load(std::memory_order_relaxed);
    diagnostics.block_boundary_restart_count = s_block_boundary_restart_count.load(std::memory_order_relaxed);
    diagnostics.block_boundary_restart_pending = s_block_boundary_restart_pending.load(std::memory_order_acquire);
    diagnostics.active_execution_id = s_active_execution_id.load(std::memory_order_acquire);
    diagnostics.active = diagnostics.active_execution_id != 0;
    diagnostics.active_step_interval_us = s_active_step_interval_us.load(std::memory_order_relaxed);
    diagnostics.active_amass_timer_interval_us = s_active_amass_timer_interval_us.load(std::memory_order_relaxed);
    diagnostics.active_amass_level = s_active_amass_level.load(std::memory_order_relaxed);
    diagnostics.active_start_speed_tenths_mm_per_min = s_active_start_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    diagnostics.active_end_speed_tenths_mm_per_min = s_active_end_speed_tenths_mm_per_min.load(std::memory_order_relaxed);
    diagnostics.pending_completions = completion_count_snapshot();
    return diagnostics;
}

}  // namespace ray2::stepper
