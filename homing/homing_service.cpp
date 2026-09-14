#include "homing_service.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_affinity.hpp"
#include "service.hpp"
#include "limit_switch_driver.hpp"
#include "laser_driver.hpp"
#include "planner.hpp"
#include "command_dispatch.hpp"
#include "raycube_machine.hpp"
#include "sd_job_service.hpp"
#include "stepper_driver.hpp"
#include "stepper_prep.hpp"
#include "timer_stepper_executor.hpp"

namespace ray2::homing {
namespace {

std::atomic<HomingState> s_state = HomingState::Idle;
std::atomic<bool> s_seek_active = false;
std::atomic<bool> s_limit_triggered = false;
std::atomic<std::size_t> s_seek_axis = 0;
std::atomic<std::uint32_t> s_active_axes_mask = 0;
std::atomic<std::uint32_t> s_triggered_axes_mask = 0;
TaskHandle_t s_seek_task = nullptr;

constexpr double kSeekRateMmPerMin = 2000.0;
constexpr double kPullOffMm = 3.0;
constexpr double kSeekDistanceMultiplier = 1.1;
constexpr TickType_t kMotionTimeoutMargin = pdMS_TO_TICKS(2000);
std::atomic<bool> s_stop_on_limit = false;
std::atomic<bool> s_abort_requested = false;

void IRAM_ATTR on_limit_switch(std::size_t axis, void*) {
    const std::uint32_t axis_bit = 1U << axis;
    const std::uint32_t active_axes = s_active_axes_mask.load(std::memory_order_relaxed);
    if (!s_seek_active.load(std::memory_order_relaxed) ||
        !s_stop_on_limit.load(std::memory_order_relaxed) || (active_axes & axis_bit) == 0) {
        return;
    }
    s_limit_triggered.store(true, std::memory_order_relaxed);
    s_triggered_axes_mask.fetch_or(axis_bit, std::memory_order_relaxed);
    if (stepper::timer_stepper_executor_stop_axis_from_isr(axis) && s_seek_task != nullptr) {
        BaseType_t task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_seek_task, &task_woken);
        if (task_woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

bool axis_limit_is_triggered(std::size_t axis_index);
bool wait_for_limit_state(std::size_t axis_index, bool triggered);

bool run_parallel_homing() {
    constexpr std::uint32_t kBothAxes = (1U << machine::kAxisCount) - 1U;
    if (axis_limit_is_triggered(0) || axis_limit_is_triggered(1)) return false;
    stepper::StepSegment seek = {};
    seek.execution_id = 1;
    seek.system_motion = true;
    seek.step_interval_us = static_cast<std::uint32_t>(std::llround(
        60000000.0 / (kSeekRateMmPerMin * machine::raycube_axis(machine::Axis::X).steps_per_mm)));
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis));
        seek.axis_steps[axis] = static_cast<std::int32_t>(std::llround(
            hardware.max_travel_mm * kSeekDistanceMultiplier * hardware.steps_per_mm));
        seek.positive_direction[axis] = hardware.homing_seek_positive_direction;
        seek.major_steps = std::max(seek.major_steps, seek.axis_steps[axis]);
    }
    seek.duration_us = seek.step_interval_us * static_cast<std::uint32_t>(seek.major_steps);
    s_active_axes_mask.store(kBothAxes, std::memory_order_relaxed);
    s_triggered_axes_mask.store(0, std::memory_order_relaxed);
    s_stop_on_limit.store(true, std::memory_order_release);
    if (stepper::timer_stepper_executor_start_segment(seek) != ESP_OK) return false;
    const TickType_t timeout = pdMS_TO_TICKS(seek.duration_us / 1000U) + kMotionTimeoutMargin;
    const TickType_t deadline = xTaskGetTickCount() + timeout;
    while (stepper::timer_stepper_executor_is_busy() &&
           !s_abort_requested.load(std::memory_order_acquire) &&
           static_cast<std::int32_t>(deadline - xTaskGetTickCount()) > 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    const bool both_triggered = s_triggered_axes_mask.load(std::memory_order_acquire) == kBothAxes;
    stepper::timer_stepper_executor_stop();
    s_stop_on_limit.store(false, std::memory_order_release);
    if (s_abort_requested.load(std::memory_order_acquire)) return false;
    if (!both_triggered) return false;

    stepper::StepSegment pull_off = {};
    pull_off.execution_id = 1;
    pull_off.system_motion = true;
    pull_off.step_interval_us = static_cast<std::uint32_t>(std::llround(
        60000000.0 / (kSeekRateMmPerMin * machine::raycube_axis(machine::Axis::X).steps_per_mm)));
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis));
        pull_off.axis_steps[axis] = static_cast<std::int32_t>(std::llround(kPullOffMm * hardware.steps_per_mm));
        pull_off.positive_direction[axis] = !hardware.homing_seek_positive_direction;
        pull_off.major_steps = std::max(pull_off.major_steps, pull_off.axis_steps[axis]);
    }
    pull_off.duration_us = pull_off.step_interval_us * static_cast<std::uint32_t>(pull_off.major_steps);
    s_active_axes_mask.store(0, std::memory_order_relaxed);
    if (stepper::timer_stepper_executor_start_segment(pull_off) != ESP_OK) return false;
    while (stepper::timer_stepper_executor_is_busy() &&
           !s_abort_requested.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    stepper::timer_stepper_executor_stop();
    if (s_abort_requested.load(std::memory_order_acquire)) return false;
    return wait_for_limit_state(0, false) && wait_for_limit_state(1, false);
}

bool axis_limit_is_triggered(const std::size_t axis_index) {
    return driver::limit_switch_driver_state().triggered[axis_index];
}

bool wait_for_limit_state(const std::size_t axis_index, const bool triggered) {
    vTaskDelay(pdMS_TO_TICKS(10));
    return axis_limit_is_triggered(axis_index) == triggered;
}

bool commit_machine_origin() {
    const double origin[machine::kAxisCount] = {};
    if (stepper::stepper_set_machine_position_mm(origin) != ESP_OK) return false;
    if (protocol::command_sync_parser_position_from_machine() == command::Error::Ok) return true;
    stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    return false;
}

void homing_task(void*) {
    laser::laser_disable();
    const bool completed = run_parallel_homing() && commit_machine_origin();
    s_stop_on_limit.store(false, std::memory_order_release);
    if (!completed) {
        stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    } else {
        // A successful homing establishes a new trusted coordinate after
        // Ctrl-X, so laser motion output may be armed for later G-code.
        laser::laser_set_motion_output_permitted(true);
        if (service::service_state() == service::State::Aborted ||
            service::service_state() == service::State::Faulted) {
            service::service_post_event(service::EventType::ResetToIdle);
        }
    }
    s_state.store(completed ? HomingState::Completed : HomingState::Faulted,
                  std::memory_order_release);
    s_seek_active.store(false, std::memory_order_release);
    s_seek_task = nullptr;
    vTaskDelete(nullptr);
}

void seek_test_task(void*) {
    const std::size_t axis_index = s_seek_axis.load(std::memory_order_relaxed);
    const auto axis = static_cast<machine::Axis>(axis_index);
    const auto& hardware = machine::raycube_axis(axis);
    const std::int32_t steps = static_cast<std::int32_t>(
        std::llround(hardware.max_travel_mm * kSeekDistanceMultiplier * hardware.steps_per_mm));
    const std::uint32_t interval_us = static_cast<std::uint32_t>(std::llround(
        60000000.0 / (kSeekRateMmPerMin * hardware.steps_per_mm)));
    stepper::StepSegment segment = {};
    segment.execution_id = 1;
    segment.axis_steps[axis_index] = steps;
    segment.positive_direction[axis_index] = hardware.homing_seek_positive_direction;
    segment.major_steps = steps;
    segment.duration_us = interval_us * static_cast<std::uint32_t>(steps);
    segment.step_interval_us = interval_us;
    segment.system_motion = true;

    laser::laser_disable();
    const esp_err_t result = stepper::timer_stepper_executor_start_segment(segment);
    if (result != ESP_OK) {
        s_seek_active.store(false, std::memory_order_release);
        s_state.store(HomingState::Faulted, std::memory_order_release);
        s_seek_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    while (stepper::timer_stepper_executor_is_busy() &&
           !s_limit_triggered.load(std::memory_order_acquire) &&
           !s_abort_requested.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    stepper::timer_stepper_executor_stop();
    s_seek_active.store(false, std::memory_order_release);
    s_stop_on_limit.store(false, std::memory_order_release);
    // A single-axis diagnostic establishes no machine coordinate reference.
    stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    s_state.store(!s_abort_requested.load(std::memory_order_acquire) &&
                          s_limit_triggered.load(std::memory_order_acquire)
                      ? HomingState::Completed
                      : HomingState::Faulted,
                  std::memory_order_release);
    s_seek_task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

esp_err_t homing_init() {
    if (!driver::limit_switch_driver_is_initialized()) return ESP_ERR_INVALID_STATE;
    driver::limit_switch_driver_set_isr_callback(on_limit_switch, nullptr);
    return ESP_OK;
}

StartResult homing_start() {
    if (homing_is_active() || !planner::planner_is_idle() || sd::sd_job_is_active()) {
        return StartResult::Busy;
    }

    if (!driver::limit_switch_driver_is_initialized()) return StartResult::Unavailable;
    s_seek_active.store(true, std::memory_order_release);
    s_abort_requested.store(false, std::memory_order_release);
    s_state.store(HomingState::Running, std::memory_order_release);
    if (xTaskCreatePinnedToCore(homing_task, "homing", 3072, nullptr, 6, &s_seek_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        s_seek_active.store(false, std::memory_order_release);
        s_state.store(HomingState::Faulted, std::memory_order_release);
        s_seek_task = nullptr;
        return StartResult::Unavailable;
    }
    return StartResult::Accepted;
}

StartResult homing_start_seek_test(const TestAxis axis) {
    if (homing_is_active() || !planner::planner_is_idle() || sd::sd_job_is_active()) {
        return StartResult::Busy;
    }
    if (!driver::limit_switch_driver_is_initialized()) return StartResult::Unavailable;
    const std::size_t axis_index = static_cast<std::size_t>(axis);
    if (driver::limit_switch_driver_state().triggered[axis_index]) return StartResult::Unavailable;

    s_seek_axis.store(axis_index, std::memory_order_relaxed);
    s_active_axes_mask.store(1U << axis_index, std::memory_order_relaxed);
    s_triggered_axes_mask.store(0, std::memory_order_relaxed);
    s_limit_triggered.store(false, std::memory_order_relaxed);
    s_stop_on_limit.store(true, std::memory_order_release);
    s_seek_active.store(true, std::memory_order_release);
    s_abort_requested.store(false, std::memory_order_release);
    s_state.store(HomingState::Running, std::memory_order_release);
    if (xTaskCreatePinnedToCore(seek_test_task, "homing_seek", 3072, nullptr, 6, &s_seek_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        s_seek_active.store(false, std::memory_order_release);
        s_stop_on_limit.store(false, std::memory_order_release);
        s_state.store(HomingState::Faulted, std::memory_order_release);
        s_seek_task = nullptr;
        return StartResult::Unavailable;
    }
    return StartResult::Accepted;
}

HomingState homing_state() {
    return s_state.load(std::memory_order_relaxed);
}

bool homing_is_active() {
    return homing_state() == HomingState::Running;
}

void homing_request_abort() {
    if (homing_is_active()) {
        s_abort_requested.store(true, std::memory_order_release);
    }
}

const char* homing_state_text(const HomingState state) {
    switch (state) {
        case HomingState::Idle: return "idle";
        case HomingState::Running: return "running";
        case HomingState::Completed: return "completed";
        case HomingState::Faulted: return "faulted";
    }
    return "unknown";
}

}  // namespace ray2::homing
