#include "planner.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "timer_stepper_executor.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"

namespace ray2::planner {
namespace {

enum class ControlState {
    Running,
    Hold,
};

TaskHandle_t s_planner_task = nullptr;
std::array<MotionBlock, kPlannerQueueLength> s_block_buffer = {};
std::size_t s_block_head = 0;
std::size_t s_block_tail = 0;
std::size_t s_block_planned = 0;
std::size_t s_block_count = 0;
// Once the preceding motion block leaves the ring, preserve its exit speed as
// the immutable entry boundary for the newly exposed queue head.
double s_tail_entry_speed_sqr = 0.0;
portMUX_TYPE s_block_buffer_lock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_capacity_waiter = nullptr;
std::atomic<ControlState> s_control_state = ControlState::Running;
std::atomic<std::uint32_t> s_reset_generation = 0;
std::atomic<std::uint64_t> s_completed_block_count = 0;
std::atomic<std::uint64_t> s_coalesced_short_block_count = 0;
std::atomic<std::uint64_t> s_next_execution_id = 1;
std::atomic<bool> s_queue_drain_reported = false;
portMUX_TYPE s_execution_event_lock = portMUX_INITIALIZER_UNLOCKED;
ExecutionCallback s_execution_callback = nullptr;
void* s_execution_callback_user_data = nullptr;
MotionBlock s_active_block = {};
bool s_has_active_block = false;
std::atomic<std::size_t> s_segment_prepared_blocks = 0;

struct FeedHoldCheckpoint {
    bool valid = false;
    MotionBlock block = {};
    stepper::MachinePosition position = {};
    std::size_t block_tail = 0;
};

struct FeedHoldSnapshot {
    bool valid = false;
    FeedHoldState state = FeedHoldState::Running;
    FeedHoldCheckpoint checkpoint = {};
    std::uint64_t active_execution_id = 0;
    stepper::MachinePosition stopped_position = {};
};

portMUX_TYPE s_feed_hold_lock = portMUX_INITIALIZER_UNLOCKED;
FeedHoldCheckpoint s_latest_checkpoint = {};
FeedHoldSnapshot s_feed_hold_snapshot = {};

constexpr double kMinimumJunctionSpeedMmPerMin = 1.0;
constexpr double kCoalescingDirectionCosine = 0.999999;
constexpr double kCoalescingMaximumDeviationMm = 0.0125;
constexpr TickType_t kStepperIdleLockTime = pdMS_TO_TICKS(250);
constexpr TickType_t kHoldWaitTime = pdMS_TO_TICKS(10);
constexpr char kLogTag[] = "planner";

std::size_t block_index_after(const std::size_t index) {
    return (index + 1) % kPlannerQueueLength;
}

bool is_plannable_motion(const MotionBlock& block) {
    return block.type == MotionBlockType::Motion && block.step_event_count > 0 &&
           std::isfinite(block.machine_start_mm[0]) &&
           std::isfinite(block.machine_start_mm[1]) &&
           std::isfinite(block.machine_target_mm[0]) &&
           std::isfinite(block.machine_target_mm[1]) &&
           std::isfinite(block.millimeters) && block.millimeters > 0.0 &&
           std::isfinite(block.programmed_rate_mm_per_min) &&
           block.programmed_rate_mm_per_min > 0.0 &&
           std::isfinite(block.acceleration_mm_per_sec2) &&
           block.acceleration_mm_per_sec2 > 0.0;
}

double nominal_speed_sqr(const MotionBlock& block) {
    return block.programmed_rate_mm_per_min * block.programmed_rate_mm_per_min;
}

bool coalesce_tail_block_locked(const MotionBlock& candidate) {
    const std::size_t frozen_blocks = std::min(
        s_segment_prepared_blocks.load(std::memory_order_acquire), s_block_count);
    if (s_block_count - frozen_blocks < 2 ||
        candidate.type != MotionBlockType::Motion ||
        candidate.request.motion_mode != motion::MotionType::Linear) {
        return false;
    }

    MotionBlock& tail = s_block_buffer[(s_block_head + kPlannerQueueLength - 1) % kPlannerQueueLength];
    if (tail.type != MotionBlockType::Motion ||
        tail.request.motion_mode != motion::MotionType::Linear ||
        tail.request.spindle_mode != candidate.request.spindle_mode ||
        tail.request.spindle_power != candidate.request.spindle_power ||
        tail.request.feed_rate_mm_per_min != candidate.request.feed_rate_mm_per_min ||
        std::hypot(tail.machine_target_mm[0] - candidate.machine_start_mm[0],
                   tail.machine_target_mm[1] - candidate.machine_start_mm[1]) > 0.000001) {
        return false;
    }

    const double direction_dot = tail.unit_vector[0] * candidate.unit_vector[0] +
                                 tail.unit_vector[1] * candidate.unit_vector[1];
    if (direction_dot < kCoalescingDirectionCosine) return false;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (tail.plan.steps[axis] != 0 && candidate.plan.steps[axis] != 0 &&
            tail.plan.positive_direction[axis] != candidate.plan.positive_direction[axis]) {
            return false;
        }
    }

    const double merged_dx = candidate.machine_target_mm[0] - tail.machine_start_mm[0];
    const double merged_dy = candidate.machine_target_mm[1] - tail.machine_start_mm[1];
    const double merged_length = std::hypot(merged_dx, merged_dy);
    if (!std::isfinite(merged_length) || merged_length <= 0.0) return false;
    const double deviation = std::abs(
        merged_dx * (tail.machine_target_mm[1] - tail.machine_start_mm[1]) -
        merged_dy * (tail.machine_target_mm[0] - tail.machine_start_mm[0])) / merged_length;
    if (deviation > kCoalescingMaximumDeviationMm) return false;

    tail.machine_target_mm = candidate.machine_target_mm;
    tail.request.target_mm[0] = candidate.request.target_mm[0];
    tail.request.target_mm[1] = candidate.request.target_mm[1];
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        tail.plan.steps[axis] += candidate.plan.steps[axis];
        if (candidate.plan.steps[axis] != 0) {
            tail.plan.positive_direction[axis] = candidate.plan.positive_direction[axis];
        }
    }
    tail.step_event_count = std::max(tail.plan.steps[0], tail.plan.steps[1]);
    tail.millimeters = merged_length;
    tail.unit_vector[0] = merged_dx / merged_length;
    tail.unit_vector[1] = merged_dy / merged_length;
    tail.acceleration_mm_per_sec2 = std::min(
        tail.acceleration_mm_per_sec2, candidate.acceleration_mm_per_sec2);
    s_coalesced_short_block_count.fetch_add(1, std::memory_order_relaxed);
    return true;
}

double junction_speed_sqr(const MotionBlock& previous, const MotionBlock& current) {
    const double minimum_speed_sqr =
        kMinimumJunctionSpeedMmPerMin * kMinimumJunctionSpeedMmPerMin;
    if (!is_plannable_motion(previous) || !is_plannable_motion(current)) {
        return 0.0;
    }

    const double junction_cos_theta =
        -(previous.unit_vector[0] * current.unit_vector[0] +
          previous.unit_vector[1] * current.unit_vector[1]);
    if (junction_cos_theta > 0.999999) {
        return minimum_speed_sqr;
    }
    if (junction_cos_theta < -0.999999) {
        return std::numeric_limits<double>::infinity();
    }

    const double sin_theta_d2 = std::sqrt(0.5 * (1.0 - junction_cos_theta));
    if (sin_theta_d2 >= 1.0) {
        return minimum_speed_sqr;
    }
    const double junction_acceleration =
        std::min(previous.acceleration_mm_per_sec2, current.acceleration_mm_per_sec2) * 60.0 * 60.0;
    return std::max(minimum_speed_sqr,
                    (junction_acceleration * kJunctionDeviationMm * sin_theta_d2) /
                        (1.0 - sin_theta_d2));
}

void planner_recalculate_locked() {
    if (s_block_count == 0) {
        s_block_planned = s_block_tail;
        return;
    }

    // A published segment owns its source block profile. Only that prefix is
    // frozen; later blocks must remain eligible for look-ahead when more G-code
    // arrives while the executor is running.
    const std::size_t frozen_blocks = std::min(
        s_segment_prepared_blocks.load(std::memory_order_acquire), s_block_count);
    const std::size_t mutable_blocks = s_block_count - frozen_blocks;
    if (mutable_blocks == 0) {
        s_block_planned = s_block_head;
        return;
    }

    std::size_t index = s_block_tail;
    for (std::size_t offset = 0; offset < frozen_blocks; ++offset) {
        index = block_index_after(index);
    }
    const std::size_t mutable_tail = index;
    MotionBlock* previous = frozen_blocks == 0
                                ? nullptr
                                : &s_block_buffer[(mutable_tail + kPlannerQueueLength - 1) % kPlannerQueueLength];
    const double frozen_exit_speed_sqr = previous == nullptr
                                             ? s_tail_entry_speed_sqr
                                             : previous->exit_speed_sqr;

    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        MotionBlock& block = s_block_buffer[index];
        block.entry_speed_sqr = 0.0;
        block.exit_speed_sqr = 0.0;
        block.max_entry_speed_sqr = 0.0;
        block.max_junction_speed_sqr = 0.0;
        if (is_plannable_motion(block)) {
            const double nominal_sqr = nominal_speed_sqr(block);
            if (previous != nullptr && is_plannable_motion(*previous)) {
                block.max_junction_speed_sqr = junction_speed_sqr(*previous, block);
                block.max_entry_speed_sqr = std::min(
                    std::min(nominal_sqr, nominal_speed_sqr(*previous)),
                    block.max_junction_speed_sqr);
            } else if (offset == 0) {
                block.max_entry_speed_sqr = frozen_exit_speed_sqr;
            }
        }
        previous = &block;
        index = block_index_after(index);
    }

    // Reverse pass: every mutable block must be able to decelerate to the next
    // mutable block's entry speed. The tail of the mutable window still has to
    // reach rest because no later block has been accepted yet.
    double next_entry_speed_sqr = 0.0;
    index = (s_block_head + kPlannerQueueLength - 1) % kPlannerQueueLength;
    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        MotionBlock& block = s_block_buffer[index];
        if (is_plannable_motion(block)) {
            const double deceleration_limit = next_entry_speed_sqr +
                2.0 * block.acceleration_mm_per_sec2 * 60.0 * 60.0 * block.millimeters;
            block.entry_speed_sqr = std::min(block.max_entry_speed_sqr, deceleration_limit);
            next_entry_speed_sqr = block.entry_speed_sqr;
        } else {
            next_entry_speed_sqr = 0.0;
        }
        index = (index + kPlannerQueueLength - 1) % kPlannerQueueLength;
    }

    // Forward pass: the first mutable block inherits the already frozen
    // prefix's exit speed. It may not rewrite that exit speed, but all later
    // mutable junctions remain free to converge on a continuous profile.
    double previous_entry_speed_sqr = frozen_exit_speed_sqr;
    double previous_acceleration_mm_per_sec2 = 0.0;
    double previous_millimeters = 0.0;
    index = mutable_tail;
    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        MotionBlock& block = s_block_buffer[index];
        if (is_plannable_motion(block)) {
            if (offset == 0) {
                block.entry_speed_sqr = std::min(block.entry_speed_sqr, frozen_exit_speed_sqr);
            } else {
                const double acceleration_limit = previous_entry_speed_sqr +
                    2.0 * previous_acceleration_mm_per_sec2 * 60.0 * 60.0 * previous_millimeters;
                block.entry_speed_sqr = std::min(block.entry_speed_sqr, acceleration_limit);
            }
            previous_entry_speed_sqr = block.entry_speed_sqr;
            previous_acceleration_mm_per_sec2 = block.acceleration_mm_per_sec2;
            previous_millimeters = block.millimeters;
        } else {
            previous_entry_speed_sqr = 0.0;
            previous_acceleration_mm_per_sec2 = 0.0;
            previous_millimeters = 0.0;
        }
        index = block_index_after(index);
    }

    index = mutable_tail;
    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        MotionBlock& block = s_block_buffer[index];
        const std::size_t next_index = block_index_after(index);
        if (offset + 1 < mutable_blocks && is_plannable_motion(s_block_buffer[next_index])) {
            block.exit_speed_sqr = s_block_buffer[next_index].entry_speed_sqr;
        }
        index = next_index;
    }
    s_block_planned = s_block_head;
}

void emit_execution_event(ExecutionEventType type,
                          const MotionBlock* block = nullptr,
                          const stepper::StepSegment* segment = nullptr) {
    ExecutionCallback callback = nullptr;
    void* user_data = nullptr;
    portENTER_CRITICAL(&s_execution_event_lock);
    callback = s_execution_callback;
    user_data = s_execution_callback_user_data;
    portEXIT_CRITICAL(&s_execution_event_lock);

    if (callback == nullptr) {
        return;
    }

    ExecutionEvent event = {};
    event.type = type;
    if (block != nullptr) {
        event.execution_id = block->execution_id;
        event.request = block->request;
        event.plan = block->plan;
    }
    if (segment != nullptr) {
        if (block == nullptr) {
            event.execution_id = segment->execution_id;
            event.request.motion_mode = segment->motion_mode;
            event.request.spindle_mode = segment->spindle_mode;
        }
        event.segment_power = segment->laser_power;
        event.segment_start_speed_mm_per_min = segment->start_speed_mm_per_min;
        event.segment_end_speed_mm_per_min = segment->end_speed_mm_per_min;
        event.segment_completes_block = segment->completes_block;
    }
    event.executed_machine_position = stepper::machine_position();
    callback(event, user_data);
}

bool active_block_snapshot(MotionBlock& block) {
    portENTER_CRITICAL(&s_execution_event_lock);
    const bool has_active_block = s_has_active_block;
    if (has_active_block) {
        block = s_active_block;
    }
    portEXIT_CRITICAL(&s_execution_event_lock);
    return has_active_block;
}

void set_active_block(const MotionBlock& block) {
    portENTER_CRITICAL(&s_execution_event_lock);
    s_active_block = block;
    s_has_active_block = true;
    portEXIT_CRITICAL(&s_execution_event_lock);
}

void clear_active_block() {
    portENTER_CRITICAL(&s_execution_event_lock);
    s_has_active_block = false;
    portEXIT_CRITICAL(&s_execution_event_lock);
}

void report_queue_drained() {
    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool is_empty = s_block_count == 0;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (is_empty && !s_queue_drain_reported.exchange(true)) {
        emit_execution_event(ExecutionEventType::QueueDrained);
    }
}

SubmitResult submit_block(MotionBlock block) {
    if (s_planner_task == nullptr) {
        ESP_LOGE(kLogTag, "PLANNER-REJECT reason=task-not-started type=%d steps=%ld interval=%lu",
                 static_cast<int>(block.type),
                 static_cast<long>(block.step_event_count),
                 static_cast<unsigned long>(block.plan.step_interval_us));
        return SubmitResult::Rejected;
    }
    block.reset_generation = s_reset_generation.load();
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (coalesce_tail_block_locked(block)) {
        planner_recalculate_locked();
        s_queue_drain_reported.store(false);
        portEXIT_CRITICAL(&s_block_buffer_lock);
        xTaskNotifyGive(s_planner_task);
        return SubmitResult::Accepted;
    }
    if (s_block_count == kPlannerQueueLength) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return SubmitResult::Full;
    }
    block.execution_id = s_next_execution_id.fetch_add(1);
    s_block_buffer[s_block_head] = block;
    s_block_head = (s_block_head + 1) % kPlannerQueueLength;
    ++s_block_count;
    planner_recalculate_locked();
    s_queue_drain_reported.store(false);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    xTaskNotifyGive(s_planner_task);
    return SubmitResult::Accepted;
}

SubmitResult submit_blocks(const MotionBlock* const blocks, const std::size_t count) {
    if (blocks == nullptr || count == 0 || count > kPlannerQueueLength || s_planner_task == nullptr) {
        return SubmitResult::Rejected;
    }
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (count > kPlannerQueueLength - s_block_count) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return SubmitResult::Full;
    }
    const std::uint32_t reset_generation = s_reset_generation.load();
    for (std::size_t index = 0; index < count; ++index) {
        MotionBlock block = blocks[index];
        block.reset_generation = reset_generation;
        block.execution_id = s_next_execution_id.fetch_add(1);
        s_block_buffer[s_block_head] = block;
        s_block_head = (s_block_head + 1) % kPlannerQueueLength;
        ++s_block_count;
    }
    planner_recalculate_locked();
    s_queue_drain_reported.store(false);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    xTaskNotifyGive(s_planner_task);
    return SubmitResult::Accepted;
}

bool current_block_snapshot(MotionBlock& block) {
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count == 0) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }
    block = s_block_buffer[s_block_tail];
    portEXIT_CRITICAL(&s_block_buffer_lock);
    return true;
}

void record_feed_hold_checkpoint(const MotionBlock& block) {
    FeedHoldCheckpoint checkpoint = {};
    checkpoint.valid = true;
    checkpoint.block = block;
    checkpoint.position = stepper::machine_position();
    portENTER_CRITICAL(&s_block_buffer_lock);
    checkpoint.block_tail = s_block_tail;
    portEXIT_CRITICAL(&s_block_buffer_lock);

    portENTER_CRITICAL(&s_feed_hold_lock);
    s_latest_checkpoint = checkpoint;
    portEXIT_CRITICAL(&s_feed_hold_lock);
}

void capture_feed_hold_snapshot() {
    MotionBlock active = {};
    const bool has_active = active_block_snapshot(active);
    const bool executor_busy = stepper::timer_stepper_executor_is_busy();
    const stepper::MachinePosition current_position = stepper::machine_position();

    portENTER_CRITICAL(&s_feed_hold_lock);
    s_feed_hold_snapshot = {};
    s_feed_hold_snapshot.state = FeedHoldState::Requested;
    s_feed_hold_snapshot.active_execution_id = has_active ? active.execution_id : 0;
    s_feed_hold_snapshot.stopped_position = current_position;
    // An idle Hold must not resurrect the previous job's checkpoint. During a
    // narrow start-up window the ISR may already be busy before planner has
    // published the active-block event, so executor activity is also valid.
    if ((has_active || executor_busy) && s_latest_checkpoint.valid) {
        s_feed_hold_snapshot.valid = true;
        s_feed_hold_snapshot.checkpoint = s_latest_checkpoint;
        if (!has_active) {
            s_feed_hold_snapshot.active_execution_id = s_latest_checkpoint.block.execution_id;
        }
    }
    portEXIT_CRITICAL(&s_feed_hold_lock);
}

void capture_feed_hold_stop_if_ready() {
    if (s_control_state.load(std::memory_order_acquire) != ControlState::Hold ||
        stepper::timer_stepper_executor_is_busy() || !stepper::stepper_prep_hold_complete()) {
        return;
    }

    portENTER_CRITICAL(&s_feed_hold_lock);
    if (s_feed_hold_snapshot.state == FeedHoldState::Requested) {
        s_feed_hold_snapshot.stopped_position = stepper::machine_position();
        s_feed_hold_snapshot.state = FeedHoldState::Stopped;
    }
    portEXIT_CRITICAL(&s_feed_hold_lock);
}

void clear_feed_hold_snapshot() {
    portENTER_CRITICAL(&s_feed_hold_lock);
    s_feed_hold_snapshot = {};
    portEXIT_CRITICAL(&s_feed_hold_lock);
}

bool feed_hold_is_stopped() {
    portENTER_CRITICAL(&s_feed_hold_lock);
    const bool stopped = s_feed_hold_snapshot.state == FeedHoldState::Stopped;
    portEXIT_CRITICAL(&s_feed_hold_lock);
    return stopped;
}

bool queued_block_snapshot(const std::size_t offset, MotionBlock& block) {
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (offset >= s_block_count) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }
    std::size_t index = s_block_tail;
    for (std::size_t position = 0; position < offset; ++position) {
        index = block_index_after(index);
    }
    block = s_block_buffer[index];
    portEXIT_CRITICAL(&s_block_buffer_lock);
    return true;
}

void discard_current_block(const MotionBlock& block, bool preserve_exit_speed = false);

void discard_current_block(const MotionBlock& block, const bool preserve_exit_speed) {
    TaskHandle_t capacity_waiter = nullptr;
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count != 0) {
        const MotionBlock& current = s_block_buffer[s_block_tail];
        if (current.execution_id == block.execution_id &&
            current.reset_generation == block.reset_generation) {
            s_block_tail = block_index_after(s_block_tail);
            --s_block_count;
            s_tail_entry_speed_sqr = preserve_exit_speed && s_block_count != 0
                                          ? std::max(0.0, current.exit_speed_sqr)
                                          : 0.0;
            const std::size_t prepared = s_segment_prepared_blocks.load(std::memory_order_acquire);
            if (prepared != 0) {
                s_segment_prepared_blocks.store(prepared - 1, std::memory_order_release);
            }
            planner_recalculate_locked();
            capacity_waiter = s_capacity_waiter;
        }
    }
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (capacity_waiter != nullptr) {
        xTaskNotifyGive(capacity_waiter);
    }
}

void disable_all_axes() {
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        stepper::stepper_set_enabled(static_cast<machine::Axis>(axis), false);
    }
}

void disable_axes_after_idle() {
    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool queue_empty_before_wait = s_block_count == 0;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (!queue_empty_before_wait) {
        return;
    }

    vTaskDelay(kStepperIdleLockTime);

    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool queue_still_empty = s_block_count == 0;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (queue_still_empty) {
        disable_all_axes();
    }
}

bool wait_for_run_permission(std::uint32_t block_generation) {
    while (s_control_state.load() == ControlState::Hold) {
        if (s_reset_generation.load() != block_generation) {
            return false;
        }
        vTaskDelay(kHoldWaitTime == 0 ? 1 : kHoldWaitTime);
    }
    return s_reset_generation.load() == block_generation;
}

void fill_segment_ring_from_planner() {
    if (s_control_state.load(std::memory_order_acquire) == ControlState::Hold) {
        if (stepper::stepper_prep_hold_boundary_pending()) {
            // Old cached segments carry the pre-hold profile and are invalid
            // once the actual boundary speed has been captured.
            stepper::stepper_segment_discard_pending();
            stepper::stepper_prep_begin_feed_hold();
        }
        if (!stepper::stepper_prep_profile().active) {
            MotionBlock candidate = {};
            if (queued_block_snapshot(0, candidate) && candidate.type == MotionBlockType::Motion &&
                stepper::stepper_prep_continue_feed_hold(candidate)) {
                s_segment_prepared_blocks.fetch_add(1, std::memory_order_release);
            }
        }
        if (stepper::stepper_prep_profile().active &&
            !stepper::stepper_prep_profile().generation_complete) {
            stepper::stepper_prep_fill();
        }
        return;
    }
    while (stepper::stepper_segment_free_slots() != 0) {
        if (stepper::stepper_prep_profile().active) {
            stepper::stepper_prep_fill();
            // A block remains prep-owned until its final segment has been
            // physically emitted. Do not prebuild later blocks into the cache.
            return;
        }

        MotionBlock candidate = {};
        const std::size_t offset = s_segment_prepared_blocks.load(std::memory_order_acquire);
        if (!queued_block_snapshot(offset, candidate) || candidate.type != MotionBlockType::Motion) {
            return;
        }
        if (!stepper::stepper_prep_start_block(candidate)) {
            return;
        }
        // Freeze before publishing the first segment. Concurrent submission
        // must not let look-ahead mutate this block's copied profile.
        s_segment_prepared_blocks.fetch_add(1, std::memory_order_release);
    }
}

void mark_isr_started_block_active() {
    if (!stepper::timer_stepper_executor_is_busy()) {
        return;
    }
    const auto diagnostics = stepper::stepper_segment_diagnostics();
    MotionBlock block = {};
    if (!diagnostics.active || !current_block_snapshot(block) ||
        block.execution_id != diagnostics.active_execution_id) {
        return;
    }
    MotionBlock active = {};
    if (!active_block_snapshot(active) || active.execution_id != block.execution_id) {
        set_active_block(block);
        emit_execution_event(ExecutionEventType::BlockStarted, &block);
    }
}

bool start_segment_ring() {
    MotionBlock block = {};
    if (current_block_snapshot(block)) {
        // Capture before the first STEP of this block, so a later recovery has
        // an exact integer-step boundary rather than a planned approximation.
        record_feed_hold_checkpoint(block);
    }
    if (stepper::timer_stepper_executor_start_from_segment_ring() != ESP_OK) {
        return false;
    }
    mark_isr_started_block_active();
    return true;
}

void consume_segment_completions() {
    stepper::SegmentCompletion completion = {};
    while (stepper::stepper_completion_consume(completion)) {
        MotionBlock block = {};
        if (!current_block_snapshot(block) || block.execution_id != completion.execution_id) {
            stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
            continue;
        }

        MotionBlock active = {};
        if (active_block_snapshot(active) && active.execution_id == completion.execution_id) {
            clear_active_block();
        }
        if (completion.status == stepper::SegmentCompletionStatus::Completed) {
            if (!stepper::stepper_prep_complete_block(completion.execution_id)) {
                stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
                emit_execution_event(ExecutionEventType::BlockFailed, &block);
                stepper::stepper_prep_cancel_block();
                discard_current_block(block);
                report_queue_drained();
                continue;
            }
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            discard_current_block(block, true);
            report_queue_drained();
            continue;
        } else {
            stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
            emit_execution_event(ExecutionEventType::BlockFailed, &block);
            stepper::stepper_prep_cancel_block();
        }
        discard_current_block(block);
        report_queue_drained();
    }
}

void planner_task(void*) {
    MotionBlock block = {};

    while (true) {
        consume_segment_completions();
        fill_segment_ring_from_planner();
        mark_isr_started_block_active();
        capture_feed_hold_stop_if_ready();
        if (stepper::timer_stepper_executor_is_busy()) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (s_control_state.load(std::memory_order_acquire) == ControlState::Hold &&
            stepper::stepper_segment_count() != 0) {
            start_segment_ring();
            continue;
        }

        if (!current_block_snapshot(block)) {
            disable_axes_after_idle();
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (!wait_for_run_permission(block.reset_generation)) {
            stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
            emit_execution_event(ExecutionEventType::BlockCancelled, &block);
            report_queue_drained();
            continue;
        }

        if (block.type == MotionBlockType::SpindleDisable) {
            set_active_block(block);
            emit_execution_event(ExecutionEventType::BlockStarted, &block);
            clear_active_block();
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            discard_current_block(block);
            report_queue_drained();
            continue;
        }
        if (block.type == MotionBlockType::ProgramPause) {
            set_active_block(block);
            emit_execution_event(ExecutionEventType::BlockStarted, &block);
            clear_active_block();
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            discard_current_block(block);
            s_control_state.store(ControlState::Hold);
            report_queue_drained();
            continue;
        }
        if (block.type == MotionBlockType::Dwell) {
            set_active_block(block);
            emit_execution_event(ExecutionEventType::BlockStarted, &block);
            std::uint32_t remaining = block.dwell_milliseconds;
            while (remaining != 0 && s_reset_generation.load() == block.reset_generation) {
                const TickType_t delay = pdMS_TO_TICKS(std::min<std::uint32_t>(remaining, 10));
                vTaskDelay(delay == 0 ? 1 : delay);
                remaining = remaining > 10 ? remaining - 10 : 0;
                while (s_control_state.load() == ControlState::Hold &&
                       s_reset_generation.load() == block.reset_generation) {
                    vTaskDelay(kHoldWaitTime == 0 ? 1 : kHoldWaitTime);
                }
            }
            clear_active_block();
            if (s_reset_generation.load() != block.reset_generation) {
                emit_execution_event(ExecutionEventType::BlockCancelled, &block);
            } else {
                ++s_completed_block_count;
                emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            }
            discard_current_block(block);
            report_queue_drained();
            continue;
        }
        if (!start_segment_ring()) {
            const auto prep = stepper::stepper_prep_diagnostics();
            if (prep.active && prep.execution_id == block.execution_id) {
                // A resumed partial block owns its geometry in prep. An empty
                // ring is a producer scheduling gap, never an execution
                // failure or permission to discard the planner head.
                vTaskDelay(1);
                continue;
            }
            if (s_reset_generation.load() != block.reset_generation) {
                stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
                emit_execution_event(ExecutionEventType::BlockCancelled, &block);
            } else {
                stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
                emit_execution_event(ExecutionEventType::BlockFailed, &block);
            }
            discard_current_block(block);
            report_queue_drained();
            disable_axes_after_idle();
            continue;
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

}  // namespace

esp_err_t planner_start() {
    if (s_planner_task != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(planner_task, "planner", 4096, nullptr, 7, &s_planner_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    stepper::stepper_completion_set_notification_task(s_planner_task);
    return ESP_OK;
}

SubmitResult planner_submit(const MotionBlock& block) {
    if (block.type != MotionBlockType::Motion || block.plan.step_interval_us == 0) {
        ESP_LOGE(kLogTag, "PLANNER-REJECT reason=invalid-motion-block type=%d steps=%ld interval=%lu",
                 static_cast<int>(block.type),
                 static_cast<long>(block.step_event_count),
                 static_cast<unsigned long>(block.plan.step_interval_us));
        return SubmitResult::Rejected;
    }
    return submit_block(block);
}

SubmitResult planner_submit_batch(const MotionBlock* const blocks, const std::size_t count) {
    if (blocks == nullptr || count == 0 || count > kPlannerQueueLength) return SubmitResult::Rejected;
    for (std::size_t index = 0; index < count; ++index) {
        if (blocks[index].type != MotionBlockType::Motion || blocks[index].plan.step_interval_us == 0) {
            return SubmitResult::Rejected;
        }
    }
    return submit_blocks(blocks, count);
}

SubmitResult planner_submit_spindle_disable() {
    MotionBlock block = {};
    block.type = MotionBlockType::SpindleDisable;
    block.request.spindle_mode = motion::SpindleMode::Off;
    return submit_block(block);
}

void planner_set_execution_callback(ExecutionCallback callback, void* user_data) {
    portENTER_CRITICAL(&s_execution_event_lock);
    s_execution_callback = callback;
    s_execution_callback_user_data = user_data;
    portEXIT_CRITICAL(&s_execution_event_lock);
}

void planner_request_feed_hold() {
    s_control_state.store(ControlState::Hold);
    stepper::timer_stepper_executor_request_feed_hold();
    capture_feed_hold_snapshot();
    portENTER_CRITICAL(&s_block_buffer_lock);
    planner_recalculate_locked();
    portEXIT_CRITICAL(&s_block_buffer_lock);
    MotionBlock block = {};
    if (active_block_snapshot(block)) {
        emit_execution_event(ExecutionEventType::FeedHoldRequested, &block);
    } else {
        emit_execution_event(ExecutionEventType::FeedHoldRequested);
    }
    if (s_planner_task != nullptr) {
        xTaskNotifyGive(s_planner_task);
    }
}

void planner_resume() {
    if (feed_hold_is_stopped()) {
        const bool has_partial_block = stepper::stepper_prep_diagnostics().active;
        if (has_partial_block && !stepper::stepper_prep_resume_from_hold()) {
            return;
        }
    }
    if (s_control_state.load() == ControlState::Hold) {
        s_control_state.store(ControlState::Running);
    }
    stepper::timer_stepper_executor_resume();
    clear_feed_hold_snapshot();
    MotionBlock block = {};
    if (active_block_snapshot(block)) {
        emit_execution_event(ExecutionEventType::Resumed, &block);
    } else {
        emit_execution_event(ExecutionEventType::Resumed);
    }
    if (s_planner_task != nullptr) {
        xTaskNotifyGive(s_planner_task);
    }
}

void planner_reset() {
    s_reset_generation.fetch_add(1);
    s_control_state.store(ControlState::Running);
    stepper::timer_stepper_executor_stop();
    clear_active_block();
    stepper::stepper_prep_reset();
    clear_feed_hold_snapshot();
    portENTER_CRITICAL(&s_feed_hold_lock);
    s_latest_checkpoint = {};
    portEXIT_CRITICAL(&s_feed_hold_lock);
    s_segment_prepared_blocks.store(0, std::memory_order_release);
    stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    TaskHandle_t capacity_waiter = nullptr;
    portENTER_CRITICAL(&s_block_buffer_lock);
    s_block_head = 0;
    s_block_tail = 0;
    s_block_planned = 0;
    s_block_count = 0;
    s_tail_entry_speed_sqr = 0.0;
    s_coalesced_short_block_count.store(0, std::memory_order_relaxed);
    capacity_waiter = s_capacity_waiter;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (capacity_waiter != nullptr) {
        xTaskNotifyGive(capacity_waiter);
    }
    if (s_planner_task != nullptr) {
        xTaskNotifyGive(s_planner_task);
    }
    report_queue_drained();
}

bool planner_is_idle() {
    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool queue_is_empty = s_block_count == 0;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    portENTER_CRITICAL(&s_execution_event_lock);
    const bool has_active_block = s_has_active_block;
    portEXIT_CRITICAL(&s_execution_event_lock);
    return queue_is_empty && !has_active_block;
}

bool planner_is_holding() {
    return s_control_state.load() == ControlState::Hold;
}

std::size_t planner_free_blocks() {
    if (s_planner_task == nullptr) {
        return 0;
    }
    portENTER_CRITICAL(&s_block_buffer_lock);
    const std::size_t free_blocks = kPlannerQueueLength - s_block_count;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    return free_blocks;
}

void planner_wait_for_free_block() {
    const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    while (true) {
        portENTER_CRITICAL(&s_block_buffer_lock);
        if (s_block_count < kPlannerQueueLength) {
            if (s_capacity_waiter == current_task) {
                s_capacity_waiter = nullptr;
            }
            portEXIT_CRITICAL(&s_block_buffer_lock);
            return;
        }
        s_capacity_waiter = current_task;
        portEXIT_CRITICAL(&s_block_buffer_lock);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

std::uint64_t planner_completed_block_count() {
    return s_completed_block_count.load();
}

SubmitResult planner_submit_program_pause() {
    MotionBlock block = {};
    block.type = MotionBlockType::ProgramPause;
    return submit_block(block);
}

SubmitResult planner_submit_dwell(const std::uint32_t milliseconds) {
    if (milliseconds == 0) return SubmitResult::Rejected;
    MotionBlock block = {};
    block.type = MotionBlockType::Dwell;
    block.dwell_milliseconds = milliseconds;
    return submit_block(block);
}

PlannerDiagnostics planner_diagnostics() {
    PlannerDiagnostics diagnostics = {};
    const bool executor_busy = stepper::timer_stepper_executor_is_busy();

    portENTER_CRITICAL(&s_block_buffer_lock);
    diagnostics.queued_blocks = s_block_count;
    diagnostics.frozen_blocks = std::min(
        s_segment_prepared_blocks.load(std::memory_order_acquire), s_block_count);
    diagnostics.coalesced_short_blocks =
        s_coalesced_short_block_count.load(std::memory_order_relaxed);
    for (std::size_t offset = 0; offset < diagnostics.blocks.size() && offset < s_block_count; ++offset) {
        const std::size_t index = (s_block_tail + offset) % kPlannerQueueLength;
        const MotionBlock& block = s_block_buffer[index];
        PlannerBlockDiagnostics& snapshot = diagnostics.blocks[offset];
        snapshot.present = true;
        snapshot.execution_id = block.execution_id;
        snapshot.entry_speed_mm_per_min = std::sqrt(std::max(0.0, block.entry_speed_sqr));
        snapshot.exit_speed_mm_per_min = std::sqrt(std::max(0.0, block.exit_speed_sqr));
        snapshot.max_junction_speed_mm_per_min =
            std::sqrt(std::max(0.0, block.max_junction_speed_sqr));
        snapshot.programmed_rate_mm_per_min = block.programmed_rate_mm_per_min;
        snapshot.state = offset == 0 && executor_busy
                             ? PlannerBlockState::Executing
                             : offset < diagnostics.frozen_blocks
                                   ? PlannerBlockState::Frozen
                                   : PlannerBlockState::Lookahead;
    }
    portEXIT_CRITICAL(&s_block_buffer_lock);
    portENTER_CRITICAL(&s_feed_hold_lock);
    diagnostics.feed_hold.state = s_feed_hold_snapshot.state;
    diagnostics.feed_hold.snapshot_valid = s_feed_hold_snapshot.valid;
    diagnostics.feed_hold.active_execution_id = s_feed_hold_snapshot.active_execution_id;
    diagnostics.feed_hold.checkpoint_execution_id =
        s_feed_hold_snapshot.checkpoint.valid ? s_feed_hold_snapshot.checkpoint.block.execution_id : 0;
    diagnostics.feed_hold.checkpoint_steps = s_feed_hold_snapshot.checkpoint.position.steps;
    diagnostics.feed_hold.stopped_steps = s_feed_hold_snapshot.stopped_position.steps;
    portEXIT_CRITICAL(&s_feed_hold_lock);
    return diagnostics;
}

}  // namespace ray2::planner
