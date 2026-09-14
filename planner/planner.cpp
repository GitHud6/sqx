#include "planner.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "runtime_affinity.hpp"

#include "timer_stepper_executor.hpp"
#include "segment_producer.hpp"
#include "segment_ring.hpp"
#include "service.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"

namespace ray2::planner {
namespace {

TaskHandle_t s_planner_task = nullptr;
SemaphoreHandle_t s_recalculate_mutex = nullptr;
std::array<MotionBlock, kPlannerQueueLength> s_block_buffer = {};
std::size_t s_block_head = 0;
std::size_t s_block_tail = 0;
std::size_t s_block_count = 0;
std::uint32_t s_block_generation = 0;
bool s_recalculation_required = false;
// Once the preceding motion block leaves the ring, preserve its exit speed as
// the immutable entry boundary for the newly exposed queue head.
double s_tail_entry_speed_sqr = 0.0;
portMUX_TYPE s_block_buffer_lock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_capacity_waiter = nullptr;
std::atomic<std::uint64_t> s_completed_block_count = 0;
std::atomic<std::uint64_t> s_next_execution_id = 1;
std::atomic<bool> s_queue_drain_reported = false;
portMUX_TYPE s_execution_event_lock = portMUX_INITIALIZER_UNLOCKED;
ExecutionCallback s_execution_callback = nullptr;
void* s_execution_callback_user_data = nullptr;
MotionBlock s_active_block = {};
bool s_has_active_block = false;

constexpr double kMinimumJunctionSpeedMmPerMin = 1.0;
constexpr double kCoalescingDirectionCosine = 0.999999;
constexpr double kCoalescingMaximumDeviationMm = 0.0125;
constexpr TickType_t kStepperIdleLockTime = pdMS_TO_TICKS(250);
constexpr char kLogTag[] = "planner";

struct PlannerSnapshotBlock {
    MotionBlockType type = MotionBlockType::Motion;
    PlannerBlockState state = PlannerBlockState::Lookahead;
    std::uint64_t execution_id = 0;
    std::int32_t step_event_count = 0;
    double millimeters = 0.0;
    double programmed_rate_mm_per_min = 0.0;
    double acceleration_mm_per_sec2 = 0.0;
    double unit_vector[2] = {};
    double entry_speed_sqr = 0.0;
    double exit_speed_sqr = 0.0;
    double max_entry_speed_sqr = 0.0;
    double max_junction_speed_sqr = 0.0;
};

struct PlannerSnapshot {
    std::array<PlannerSnapshotBlock, kPlannerQueueLength> blocks = {};
    std::size_t head = 0;
    std::size_t tail = 0;
    std::size_t count = 0;
    std::size_t frozen_count = 0;
    double tail_entry_speed_sqr = 0.0;
    std::uint32_t generation = 0;
};

// Only look-ahead fields are stored in static DRAM. Request, geometry,
// positions and spindle data never participate in velocity bounds.
// Recalculation only runs in non-realtime CPU1 tasks and is serialized by
// s_recalculate_mutex.
PlannerSnapshot s_recalculate_snapshot = {};

void planner_mark_dirty_locked() {
    ++s_block_generation;
    s_recalculation_required = true;
}

std::size_t block_index_after(const std::size_t index) {
    return (index + 1) % kPlannerQueueLength;
}

std::size_t frozen_block_count_locked() {
    std::size_t count = 0;
    std::size_t index = s_block_tail;
    while (count < s_block_count &&
           s_block_buffer[index].state != PlannerBlockState::Lookahead) {
        ++count;
        index = block_index_after(index);
    }
    return count;
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

bool is_plannable_snapshot_block(const PlannerSnapshotBlock& block) {
    return block.type == MotionBlockType::Motion && block.step_event_count > 0 &&
           std::isfinite(block.millimeters) && block.millimeters > 0.0 &&
           std::isfinite(block.programmed_rate_mm_per_min) &&
           block.programmed_rate_mm_per_min > 0.0 &&
           std::isfinite(block.acceleration_mm_per_sec2) &&
           block.acceleration_mm_per_sec2 > 0.0;
}

double nominal_speed_sqr(const PlannerSnapshotBlock& block) {
    return block.programmed_rate_mm_per_min * block.programmed_rate_mm_per_min;
}

bool coalesce_tail_block_locked(const MotionBlock& candidate) {
    const std::size_t frozen_blocks = frozen_block_count_locked();
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
        if (tail.geometry.steps[axis] != 0 && candidate.geometry.steps[axis] != 0 &&
            tail.geometry.positive_direction[axis] != candidate.geometry.positive_direction[axis]) {
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
        tail.geometry.steps[axis] += candidate.geometry.steps[axis];
        if (candidate.geometry.steps[axis] != 0) {
            tail.geometry.positive_direction[axis] = candidate.geometry.positive_direction[axis];
        }
    }
        tail.step_event_count = std::max(tail.geometry.steps[0], tail.geometry.steps[1]);
    tail.millimeters = merged_length;
    tail.unit_vector[0] = merged_dx / merged_length;
    tail.unit_vector[1] = merged_dy / merged_length;
    tail.acceleration_mm_per_sec2 = std::min(
        tail.acceleration_mm_per_sec2, candidate.acceleration_mm_per_sec2);
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

double junction_speed_sqr(const PlannerSnapshotBlock& previous,
                          const PlannerSnapshotBlock& current) {
    const double minimum_speed_sqr =
        kMinimumJunctionSpeedMmPerMin * kMinimumJunctionSpeedMmPerMin;
    if (!is_plannable_snapshot_block(previous) || !is_plannable_snapshot_block(current)) {
        return 0.0;
    }

    const double junction_cos_theta =
        -(previous.unit_vector[0] * current.unit_vector[0] +
          previous.unit_vector[1] * current.unit_vector[1]);
    if (junction_cos_theta > 0.999999) return minimum_speed_sqr;
    if (junction_cos_theta < -0.999999) return std::numeric_limits<double>::infinity();

    const double sin_theta_d2 = std::sqrt(0.5 * (1.0 - junction_cos_theta));
    if (sin_theta_d2 >= 1.0) return minimum_speed_sqr;

    const double junction_acceleration =
        std::min(previous.acceleration_mm_per_sec2, current.acceleration_mm_per_sec2) * 60.0 * 60.0;
    return std::max(minimum_speed_sqr,
                    (junction_acceleration * kJunctionDeviationMm * sin_theta_d2) /
                        (1.0 - sin_theta_d2));
}

void planner_recalculate_snapshot(PlannerSnapshot& snapshot) {
    if (snapshot.count == 0) {
        return;
    }

    // A published segment owns its source block profile. Only that prefix is
    // frozen; later blocks must remain eligible for look-ahead when more G-code
    // arrives while the executor is running.
    const std::size_t frozen_blocks = snapshot.frozen_count;
    const std::size_t mutable_blocks = snapshot.count - frozen_blocks;
    if (mutable_blocks == 0) {
        return;
    }

    std::size_t index = snapshot.tail;
    for (std::size_t offset = 0; offset < frozen_blocks; ++offset) {
        index = block_index_after(index);
    }
    const std::size_t mutable_tail = index;
    PlannerSnapshotBlock* previous = frozen_blocks == 0
                                        ? nullptr
                                        : &snapshot.blocks[(mutable_tail + kPlannerQueueLength - 1) %
                                                           kPlannerQueueLength];
    const double frozen_exit_speed_sqr = previous == nullptr
                                             ? snapshot.tail_entry_speed_sqr
                                             : previous->exit_speed_sqr;

    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        PlannerSnapshotBlock& block = snapshot.blocks[index];
        block.entry_speed_sqr = 0.0;
        block.exit_speed_sqr = 0.0;
        block.max_entry_speed_sqr = 0.0;
        block.max_junction_speed_sqr = 0.0;
        if (is_plannable_snapshot_block(block)) {
            const double nominal_sqr = nominal_speed_sqr(block);
            if (previous != nullptr && is_plannable_snapshot_block(*previous)) {
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
    index = (snapshot.head + kPlannerQueueLength - 1) % kPlannerQueueLength;
    for (std::size_t offset = 0; offset < mutable_blocks; ++offset) {
        PlannerSnapshotBlock& block = snapshot.blocks[index];
        if (is_plannable_snapshot_block(block)) {
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
        PlannerSnapshotBlock& block = snapshot.blocks[index];
        if (is_plannable_snapshot_block(block)) {
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
        PlannerSnapshotBlock& block = snapshot.blocks[index];
        const std::size_t next_index = block_index_after(index);
        if (offset + 1 < mutable_blocks &&
            is_plannable_snapshot_block(snapshot.blocks[next_index])) {
            block.exit_speed_sqr = snapshot.blocks[next_index].entry_speed_sqr;
        }
        index = next_index;
    }

}

bool planner_snapshot_locked(PlannerSnapshot& snapshot) {
    snapshot.head = s_block_head;
    snapshot.tail = s_block_tail;
    snapshot.count = s_block_count;
    snapshot.frozen_count = frozen_block_count_locked();
    snapshot.tail_entry_speed_sqr = s_tail_entry_speed_sqr;
    snapshot.generation = s_block_generation;
    for (std::size_t offset = 0; offset < snapshot.count; ++offset) {
        const std::size_t index = (snapshot.tail + offset) % kPlannerQueueLength;
        const MotionBlock& source = s_block_buffer[index];
        PlannerSnapshotBlock& destination = snapshot.blocks[index];
        destination.type = source.type;
        destination.state = source.state;
        destination.execution_id = source.execution_id;
        destination.step_event_count = source.step_event_count;
        destination.millimeters = source.millimeters;
        destination.programmed_rate_mm_per_min = source.programmed_rate_mm_per_min;
        destination.acceleration_mm_per_sec2 = source.acceleration_mm_per_sec2;
        destination.unit_vector[0] = source.unit_vector[0];
        destination.unit_vector[1] = source.unit_vector[1];
        destination.entry_speed_sqr = source.entry_speed_sqr;
        destination.exit_speed_sqr = source.exit_speed_sqr;
        destination.max_entry_speed_sqr = source.max_entry_speed_sqr;
        destination.max_junction_speed_sqr = source.max_junction_speed_sqr;
    }
    return snapshot.count != 0 && snapshot.frozen_count < snapshot.count;
}

bool planner_commit_snapshot_locked(const PlannerSnapshot& snapshot) {
    if (snapshot.generation != s_block_generation || snapshot.head != s_block_head ||
        snapshot.tail != s_block_tail || snapshot.count != s_block_count ||
        snapshot.frozen_count != frozen_block_count_locked() ||
        snapshot.tail_entry_speed_sqr != s_tail_entry_speed_sqr) {
        return false;
    }
    for (std::size_t offset = snapshot.frozen_count; offset < snapshot.count; ++offset) {
        const std::size_t index = (snapshot.tail + offset) % kPlannerQueueLength;
        if (s_block_buffer[index].execution_id != snapshot.blocks[index].execution_id ||
            s_block_buffer[index].state != PlannerBlockState::Lookahead) {
            return false;
        }
    }
    for (std::size_t offset = snapshot.frozen_count; offset < snapshot.count; ++offset) {
        const std::size_t index = (snapshot.tail + offset) % kPlannerQueueLength;
        s_block_buffer[index].entry_speed_sqr = snapshot.blocks[index].entry_speed_sqr;
        s_block_buffer[index].exit_speed_sqr = snapshot.blocks[index].exit_speed_sqr;
        s_block_buffer[index].max_entry_speed_sqr = snapshot.blocks[index].max_entry_speed_sqr;
        s_block_buffer[index].max_junction_speed_sqr = snapshot.blocks[index].max_junction_speed_sqr;
    }
    s_recalculation_required = false;
    return true;
}

void planner_recalculate() {
    if (s_recalculate_mutex == nullptr ||
        xSemaphoreTake(s_recalculate_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    PlannerSnapshot& snapshot = s_recalculate_snapshot;
    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool has_mutable_blocks = planner_snapshot_locked(snapshot);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (!has_mutable_blocks) {
        xSemaphoreGive(s_recalculate_mutex);
        return;
    }

    planner_recalculate_snapshot(snapshot);

    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool committed = planner_commit_snapshot_locked(snapshot);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (!committed) {
        // The next block submission, freeze, release, or completion will
        // trigger a fresh calculation against the newer queue generation.
        xSemaphoreGive(s_recalculate_mutex);
        return;
    }
    xSemaphoreGive(s_recalculate_mutex);
}

bool planner_recalculation_pending() {
    portENTER_CRITICAL(&s_block_buffer_lock);
    const bool pending = s_recalculation_required;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    return pending;
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
        event.geometry = block->geometry;
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
    if (is_empty && !stepper::timer_stepper_executor_is_busy() &&
        !stepper::segment_producer_has_pending_segments() &&
        !stepper::stepper_prep_has_active_context()) {
        stepper::stepper_segment_clear_block_boundary_pending();
    }
    if (is_empty && !s_queue_drain_reported.exchange(true)) {
        emit_execution_event(ExecutionEventType::QueueDrained);
    }
}

SubmitResult submit_block(MotionBlock block) {
    if (s_planner_task == nullptr) {
        ESP_LOGE(kLogTag, "PLANNER-REJECT reason=task-not-started type=%d steps=%ld",
                 static_cast<int>(block.type),
                 static_cast<long>(block.step_event_count));
        return SubmitResult::Rejected;
    }
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (coalesce_tail_block_locked(block)) {
        planner_mark_dirty_locked();
        s_queue_drain_reported.store(false);
        portEXIT_CRITICAL(&s_block_buffer_lock);
        planner_recalculate();
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
    planner_mark_dirty_locked();
    s_queue_drain_reported.store(false);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    planner_recalculate();
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
    for (std::size_t index = 0; index < count; ++index) {
        MotionBlock block = blocks[index];
        block.execution_id = s_next_execution_id.fetch_add(1);
        s_block_buffer[s_block_head] = block;
        s_block_head = (s_block_head + 1) % kPlannerQueueLength;
        ++s_block_count;
    }
    planner_mark_dirty_locked();
    s_queue_drain_reported.store(false);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    planner_recalculate();
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

bool freeze_next_motion_block(MotionBlock& block) {
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_recalculation_required) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }
    const std::size_t frozen_blocks = frozen_block_count_locked();
    if (frozen_blocks >= s_block_count) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }

    std::size_t index = s_block_tail;
    for (std::size_t position = 0; position < frozen_blocks; ++position) {
        index = block_index_after(index);
    }
    MotionBlock& candidate = s_block_buffer[index];
    if (candidate.type != MotionBlockType::Motion ||
        candidate.state != PlannerBlockState::Lookahead) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }

    // Freeze the solved boundary before handing a copy to the producer. A
    // later G-code submission can only recalculate the remaining look-ahead.
    candidate.state = PlannerBlockState::Frozen;
    planner_mark_dirty_locked();
    block = candidate;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    planner_recalculate();
    return true;
}

void release_frozen_block(const std::uint64_t execution_id) {
    portENTER_CRITICAL(&s_block_buffer_lock);
    for (std::size_t offset = 0, index = s_block_tail; offset < s_block_count;
         ++offset, index = block_index_after(index)) {
        MotionBlock& block = s_block_buffer[index];
        if (block.execution_id == execution_id && block.state == PlannerBlockState::Frozen) {
            block.state = PlannerBlockState::Lookahead;
            planner_mark_dirty_locked();
            break;
        }
    }
    portEXIT_CRITICAL(&s_block_buffer_lock);
    planner_recalculate();
}

void discard_current_block(const MotionBlock& block, bool preserve_exit_speed = false);

void discard_current_block(const MotionBlock& block, const bool preserve_exit_speed) {
    TaskHandle_t capacity_waiter = nullptr;
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count != 0) {
        const MotionBlock& current = s_block_buffer[s_block_tail];
        if (current.execution_id == block.execution_id) {
            s_block_tail = block_index_after(s_block_tail);
            --s_block_count;
            s_tail_entry_speed_sqr = preserve_exit_speed && s_block_count != 0
                                          ? std::max(0.0, current.exit_speed_sqr)
                                          : 0.0;
            planner_mark_dirty_locked();
            capacity_waiter = s_capacity_waiter;
        }
    }
    portEXIT_CRITICAL(&s_block_buffer_lock);
    planner_recalculate();
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

void fill_segment_ring_from_planner(const std::uint32_t transport_events) {
    if (stepper::timer_stepper_executor_is_held()) return;
    stepper::segment_producer_process_events(transport_events);
    while (true) {
        if (stepper::segment_producer_profile().active &&
            !stepper::segment_producer_profile().generation_complete) {
            return;
        }
        if (!stepper::segment_producer_can_accept_plan()) {
            return;
        }

        MotionBlock candidate = {};
        if (!freeze_next_motion_block(candidate)) {
            return;
        }
        if (!stepper::segment_producer_submit_plan(candidate)) {
            release_frozen_block(candidate.execution_id);
            return;
        }
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
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count != 0 && s_block_buffer[s_block_tail].execution_id == block.execution_id) {
        s_block_buffer[s_block_tail].state = PlannerBlockState::Executing;
        block = s_block_buffer[s_block_tail];
    }
    portEXIT_CRITICAL(&s_block_buffer_lock);
    MotionBlock active = {};
    if (!active_block_snapshot(active) || active.execution_id != block.execution_id) {
        set_active_block(block);
        emit_execution_event(ExecutionEventType::BlockStarted, &block);
    }
}

bool start_segment_ring() {
    if (!stepper::segment_producer_start()) {
        return false;
    }
    mark_isr_started_block_active();
    return true;
}

void consume_segment_completions() {
    stepper::SegmentCompletion completion = {};
    while (stepper::segment_producer_consume_completion(completion)) {
        MotionBlock block = {};
        if (!current_block_snapshot(block) || block.execution_id != completion.execution_id) {
            ESP_LOGE(kLogTag,
                     "EXECUTION-FAULT reason=completion-mismatch completion_id=%llu "
                     "current_id=%llu status=%u",
                     static_cast<unsigned long long>(completion.execution_id),
                     static_cast<unsigned long long>(block.execution_id),
                     static_cast<unsigned>(completion.status));
            stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
            stepper::timer_stepper_executor_stop();
            stepper::segment_producer_abort();
            planner_reset();
            continue;
        }

        if (completion.status == stepper::SegmentCompletionStatus::Completed) {
            if (!stepper::segment_producer_record_segment_completed(completion)) {
                ESP_LOGE(kLogTag, "EXECUTION-FAULT reason=segment-accounting-failed id=%llu",
                         static_cast<unsigned long long>(completion.execution_id));
                stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
                emit_execution_event(ExecutionEventType::BlockFailed, &block);
                stepper::timer_stepper_executor_stop();
                stepper::segment_producer_abort();
                planner_reset();
                continue;
            }
            if (!completion.completes_block) {
                continue;
            }
            MotionBlock active = {};
            if (active_block_snapshot(active) && active.execution_id == completion.execution_id) {
                clear_active_block();
            }
            if (!stepper::segment_producer_complete_plan(completion.execution_id)) {
                const auto transport = stepper::stepper_segment_diagnostics();
                ESP_LOGE(kLogTag,
                         "EXECUTION-FAULT reason=complete-plan-failed id=%llu "
                         "segment=(queued=%u,active=%d,active_id=%llu)",
                         static_cast<unsigned long long>(completion.execution_id),
                         static_cast<unsigned>(transport.queued_segments),
                         transport.active ? 1 : 0,
                         static_cast<unsigned long long>(transport.active_execution_id));
                stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
                emit_execution_event(ExecutionEventType::BlockFailed, &block);
                stepper::timer_stepper_executor_stop();
                stepper::segment_producer_abort();
                planner_reset();
                continue;
            }
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            discard_current_block(block, true);
            report_queue_drained();
            continue;
        } else {
            ESP_LOGE(kLogTag,
                     "EXECUTION-FAULT reason=segment-fault id=%llu queued=%u active=%d",
                     static_cast<unsigned long long>(completion.execution_id),
                     static_cast<unsigned>(stepper::stepper_segment_diagnostics().queued_segments),
                     stepper::timer_stepper_executor_is_busy() ? 1 : 0);
            stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::ExecutionFailure);
            emit_execution_event(ExecutionEventType::BlockFailed, &block);
            stepper::timer_stepper_executor_stop();
            stepper::segment_producer_abort();
            planner_reset();
            continue;
        }
        discard_current_block(block);
        report_queue_drained();
    }
}

void planner_task(void*) {
    MotionBlock block = {};

    while (true) {
        if (planner_recalculation_pending()) {
            planner_recalculate();
        }
        const std::uint32_t transport_events = stepper::segment_producer_take_events();
        if ((transport_events & stepper::StepperTransportEventCompletionAvailable) != 0U) {
            consume_segment_completions();
        }
        if ((transport_events & stepper::StepperTransportEventHoldReached) != 0U) {
            service::service_post_event(service::EventType::HoldReached);
        }
        fill_segment_ring_from_planner(transport_events);
        mark_isr_started_block_active();
        if (stepper::timer_stepper_executor_is_busy()) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        if (stepper::timer_stepper_executor_is_held()) {
            // The service owns the transition out of hold. Do not consume or
            // create segments until it explicitly resumes the executor.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (!current_block_snapshot(block)) {
            disable_axes_after_idle();
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
            report_queue_drained();
            continue;
        }
        if (block.type == MotionBlockType::Dwell) {
            set_active_block(block);
            emit_execution_event(ExecutionEventType::BlockStarted, &block);
            std::uint32_t remaining = block.dwell_milliseconds;
            while (remaining != 0) {
                const TickType_t delay = pdMS_TO_TICKS(std::min<std::uint32_t>(remaining, 10));
                vTaskDelay(delay == 0 ? 1 : delay);
                remaining = remaining > 10 ? remaining - 10 : 0;
            }
            clear_active_block();
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
            discard_current_block(block);
            report_queue_drained();
            continue;
        }
        if (!start_segment_ring()) {
            if (stepper::segment_producer_profile().active &&
                stepper::segment_producer_profile().execution_plan.execution_id == block.execution_id) {
                // A resumed partial block owns its geometry in prep. An empty
                // ring is a producer scheduling gap, never an execution
                // failure or permission to discard the planner head.
                vTaskDelay(1);
                continue;
            }
            {
                ESP_LOGE(kLogTag,
                         "EXECUTION-FAULT reason=start-segment-ring-failed id=%llu "
                         "queued=%u",
                         static_cast<unsigned long long>(block.execution_id),
                         static_cast<unsigned>(stepper::stepper_segment_diagnostics().queued_segments));
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

    s_recalculate_mutex = xSemaphoreCreateMutex();
    if (s_recalculate_mutex == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(planner_task, "planner", 4096, nullptr, 7, &s_planner_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        vSemaphoreDelete(s_recalculate_mutex);
        s_recalculate_mutex = nullptr;
        return ESP_ERR_NO_MEM;
    }
    stepper::segment_producer_set_notification_task(s_planner_task);
    return ESP_OK;
}

SubmitResult planner_submit(const MotionBlock& block) {
    if (block.type != MotionBlockType::Motion || block.step_event_count <= 0) {
        ESP_LOGE(kLogTag, "PLANNER-REJECT reason=invalid-motion-block type=%d steps=%ld",
                 static_cast<int>(block.type),
                 static_cast<long>(block.step_event_count));
        return SubmitResult::Rejected;
    }
    return submit_block(block);
}

SubmitResult planner_submit_batch(const MotionBlock* const blocks, const std::size_t count) {
    if (blocks == nullptr || count == 0 || count > kPlannerQueueLength) return SubmitResult::Rejected;
    for (std::size_t index = 0; index < count; ++index) {
        if (blocks[index].type != MotionBlockType::Motion || blocks[index].step_event_count <= 0) {
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

void planner_reset() {
    stepper::timer_stepper_executor_stop();
    clear_active_block();
    stepper::segment_producer_reset();
    stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    TaskHandle_t capacity_waiter = nullptr;
    portENTER_CRITICAL(&s_block_buffer_lock);
    s_block_head = 0;
    s_block_tail = 0;
    s_block_count = 0;
    ++s_block_generation;
    s_recalculation_required = false;
    s_tail_entry_speed_sqr = 0.0;
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

}  // namespace ray2::planner
