#include "planner.hpp"

#include <algorithm>
#include <array>
#include <atomic>

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "raycube_machine.hpp"
#include "rmt_stepper_driver.hpp"
#include "stepper_driver.hpp"

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
portMUX_TYPE s_block_buffer_lock = portMUX_INITIALIZER_UNLOCKED;
std::atomic<ControlState> s_control_state = ControlState::Running;
std::atomic<std::uint32_t> s_reset_generation = 0;
std::atomic<std::uint64_t> s_completed_block_count = 0;
std::atomic<std::uint64_t> s_next_execution_id = 1;
std::atomic<bool> s_queue_drain_reported = false;
portMUX_TYPE s_execution_event_lock = portMUX_INITIALIZER_UNLOCKED;
ExecutionCallback s_execution_callback = nullptr;
void* s_execution_callback_user_data = nullptr;
MotionBlock s_active_block = {};
bool s_has_active_block = false;

void emit_execution_event(ExecutionEventType type, const MotionBlock* block = nullptr) {
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
        return SubmitResult::Rejected;
    }
    block.reset_generation = s_reset_generation.load();
    block.execution_id = s_next_execution_id.fetch_add(1);
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count == kPlannerQueueLength) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return SubmitResult::Full;
    }
    s_block_buffer[s_block_head] = block;
    s_block_head = (s_block_head + 1) % kPlannerQueueLength;
    ++s_block_count;
    s_queue_drain_reported.store(false);
    portEXIT_CRITICAL(&s_block_buffer_lock);
    xTaskNotifyGive(s_planner_task);
    return SubmitResult::Accepted;
}

bool pop_block(MotionBlock& block) {
    portENTER_CRITICAL(&s_block_buffer_lock);
    if (s_block_count == 0) {
        portEXIT_CRITICAL(&s_block_buffer_lock);
        return false;
    }
    block = s_block_buffer[s_block_tail];
    s_block_tail = (s_block_tail + 1) % kPlannerQueueLength;
    --s_block_count;
    // No look-ahead pass exists yet. Keep this explicit marker at the first
    // unconsumed block so phase two can introduce forward/reverse planning.
    s_block_planned = s_block_tail;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    return true;
}

void disable_axes(const bool enabled[]) {
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (enabled[axis]) {
            stepper::stepper_set_enabled(static_cast<machine::Axis>(axis), false);
        }
    }
}

bool configure_axes(const MotionBlock& block, bool enabled[]) {
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (block.plan.steps[axis] == 0) {
            continue;
        }

        const auto current_axis = static_cast<machine::Axis>(axis);
        const auto direction = block.plan.positive_direction[axis]
                                   ? stepper::Direction::Positive
                                   : stepper::Direction::Negative;
        if (stepper::stepper_set_direction(current_axis, direction) != ESP_OK ||
            stepper::stepper_set_enabled(current_axis, true) != ESP_OK) {
            disable_axes(enabled);
            return false;
        }
        enabled[axis] = true;
    }
    return true;
}

bool wait_for_run_permission(std::uint32_t block_generation) {
    while (s_control_state.load() == ControlState::Hold) {
        if (s_reset_generation.load() != block_generation) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return s_reset_generation.load() == block_generation;
}

bool execute_block(const MotionBlock& block) {
    const std::int32_t major_steps = std::max(block.plan.steps[0], block.plan.steps[1]);
    if (major_steps <= 0 || block.plan.step_interval_us == 0 || stepper::stepper_init() != ESP_OK) {
        return false;
    }

    bool enabled[machine::kAxisCount] = {};
    if (!configure_axes(block, enabled)) {
        return false;
    }

    std::size_t active_axis_count = 0;
    machine::Axis active_axis = machine::Axis::X;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (block.plan.steps[axis] > 0) {
            active_axis = static_cast<machine::Axis>(axis);
            ++active_axis_count;
        }
    }
    if (active_axis_count == 1 && stepper::rmt_stepper_can_execute(
                                      block.plan.steps[static_cast<std::size_t>(active_axis)],
                                      block.plan.step_interval_us)) {
        const auto axis_index = static_cast<std::size_t>(active_axis);
        const bool completed = stepper::rmt_stepper_execute(
            active_axis, block.plan.steps[axis_index], block.plan.step_interval_us) == ESP_OK;
        disable_axes(enabled);
        return completed;
    }
    if (active_axis_count == 2 &&
        stepper::rmt_stepper_can_execute_xy(block.plan.steps, block.plan.step_interval_us)) {
        const bool completed = stepper::rmt_stepper_execute_xy(
            block.plan.steps, block.plan.step_interval_us) == ESP_OK;
        disable_axes(enabled);
        return completed;
    }

    const std::int32_t minor_steps = std::min(block.plan.steps[0], block.plan.steps[1]);
    const bool x_is_major = block.plan.steps[0] >= block.plan.steps[1];
    const auto major_axis = x_is_major ? machine::Axis::X : machine::Axis::Y;
    const auto minor_axis = x_is_major ? machine::Axis::Y : machine::Axis::X;
    std::int64_t error = 0;
    bool completed = false;

    for (std::int32_t step = 0; step < major_steps; ++step) {
        if (!wait_for_run_permission(block.reset_generation) ||
            stepper::stepper_pulse(major_axis) != ESP_OK) {
            break;
        }

        error += minor_steps;
        if (error * 2 >= major_steps) {
            if (stepper::stepper_pulse(minor_axis) != ESP_OK) {
                break;
            }
            error -= major_steps;
        }

        if (block.plan.step_interval_us > machine::kStepPulseMicroseconds) {
            esp_rom_delay_us(block.plan.step_interval_us - machine::kStepPulseMicroseconds);
        }
        completed = step + 1 == major_steps;
    }

    disable_axes(enabled);
    return completed;
}

void planner_task(void*) {
    MotionBlock block = {};

    while (true) {
        if (!pop_block(block)) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (!wait_for_run_permission(block.reset_generation)) {
            emit_execution_event(ExecutionEventType::BlockCancelled, &block);
            report_queue_drained();
            continue;
        }
        set_active_block(block);
        emit_execution_event(ExecutionEventType::BlockStarted, &block);
        const bool completed = block.type == MotionBlockType::SpindleDisable || execute_block(block);
        clear_active_block();
        if (completed) {
            ++s_completed_block_count;
            emit_execution_event(ExecutionEventType::BlockCompleted, &block);
        } else if (s_reset_generation.load() != block.reset_generation) {
            emit_execution_event(ExecutionEventType::BlockCancelled, &block);
        } else {
            emit_execution_event(ExecutionEventType::BlockFailed, &block);
        }
        report_queue_drained();
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
    return ESP_OK;
}

SubmitResult planner_submit(const MotionBlock& block) {
    if (block.type != MotionBlockType::Motion || block.plan.step_interval_us == 0) {
        return SubmitResult::Rejected;
    }
    return submit_block(block);
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
    stepper::rmt_stepper_request_feed_hold();
    MotionBlock block = {};
    if (active_block_snapshot(block)) {
        emit_execution_event(ExecutionEventType::FeedHoldRequested, &block);
    } else {
        emit_execution_event(ExecutionEventType::FeedHoldRequested);
    }
}

void planner_resume() {
    if (s_control_state.load() == ControlState::Hold) {
        s_control_state.store(ControlState::Running);
    }
    stepper::rmt_stepper_resume();
    MotionBlock block = {};
    if (active_block_snapshot(block)) {
        emit_execution_event(ExecutionEventType::Resumed, &block);
    } else {
        emit_execution_event(ExecutionEventType::Resumed);
    }
}

void planner_reset() {
    s_reset_generation.fetch_add(1);
    s_control_state.store(ControlState::Running);
    stepper::rmt_stepper_stop();
    stepper::rmt_stepper_resume();
    portENTER_CRITICAL(&s_block_buffer_lock);
    s_block_head = 0;
    s_block_tail = 0;
    s_block_planned = 0;
    s_block_count = 0;
    portEXIT_CRITICAL(&s_block_buffer_lock);
    if (s_planner_task != nullptr) {
        xTaskNotifyGive(s_planner_task);
    }
    report_queue_drained();
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

std::uint64_t planner_completed_block_count() {
    return s_completed_block_count.load();
}

}  // namespace ray2::planner
