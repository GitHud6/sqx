#include "homing_service.hpp"

#include <atomic>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
TaskHandle_t s_seek_task = nullptr;

constexpr double kSeekRateMmPerMin = 2000.0;
constexpr double kLocateRateMmPerMin = 500.0;
constexpr double kPullOffMm = 3.0;
constexpr double kSeekDistanceMultiplier = 1.1;
constexpr TickType_t kMotionTimeoutMargin = pdMS_TO_TICKS(2000);
std::atomic<bool> s_stop_on_limit = false;

enum class SystemMotionResult {
    Completed,
    LimitTriggered,
    TimedOut,
    StartFailed,
};

void IRAM_ATTR on_limit_switch(std::size_t axis, void*) {
    if (!s_seek_active.load(std::memory_order_relaxed) ||
        !s_stop_on_limit.load(std::memory_order_relaxed) ||
        axis != s_seek_axis.load(std::memory_order_relaxed)) {
        return;
    }
    s_limit_triggered.store(true, std::memory_order_relaxed);
    stepper::timer_stepper_executor_stop_from_isr();
    if (s_seek_task != nullptr) {
        BaseType_t task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_seek_task, &task_woken);
        if (task_woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

SystemMotionResult run_axis_motion(const std::size_t axis_index, const bool positive_direction,
                                   const double distance_mm, const double rate_mm_per_min,
                                   const bool stop_on_limit) {
    const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis_index));
    const std::int32_t steps = static_cast<std::int32_t>(
        std::llround(distance_mm * hardware.steps_per_mm));
    const std::uint32_t interval_us = static_cast<std::uint32_t>(std::llround(
        60000000.0 / (rate_mm_per_min * hardware.steps_per_mm)));
    if (steps <= 0 || interval_us <= 3) return SystemMotionResult::StartFailed;

    stepper::StepSegment segment = {};
    segment.execution_id = 1;
    segment.axis_steps[axis_index] = steps;
    segment.positive_direction[axis_index] = positive_direction;
    segment.major_steps = steps;
    segment.duration_us = interval_us * static_cast<std::uint32_t>(steps);
    segment.step_interval_us = interval_us;
    segment.system_motion = true;

    s_seek_axis.store(axis_index, std::memory_order_relaxed);
    s_limit_triggered.store(false, std::memory_order_relaxed);
    s_stop_on_limit.store(stop_on_limit, std::memory_order_release);
    ulTaskNotifyTake(pdTRUE, 0);
    const esp_err_t result = stepper::timer_stepper_executor_start_segment(segment);
    if (result != ESP_OK) {
        s_stop_on_limit.store(false, std::memory_order_release);
        return SystemMotionResult::StartFailed;
    }

    const TickType_t timeout = pdMS_TO_TICKS(segment.duration_us / 1000U) + kMotionTimeoutMargin;
    const TickType_t deadline = xTaskGetTickCount() + timeout;
    while (stepper::timer_stepper_executor_is_busy() &&
           !s_limit_triggered.load(std::memory_order_acquire) &&
           static_cast<std::int32_t>(deadline - xTaskGetTickCount()) > 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    const bool limit_triggered = s_limit_triggered.load(std::memory_order_acquire);
    const bool timed_out = stepper::timer_stepper_executor_is_busy() && !limit_triggered;
    stepper::timer_stepper_executor_stop();
    s_stop_on_limit.store(false, std::memory_order_release);
    if (limit_triggered) return SystemMotionResult::LimitTriggered;
    return timed_out ? SystemMotionResult::TimedOut : SystemMotionResult::Completed;
}

bool axis_limit_is_triggered(const std::size_t axis_index) {
    return driver::limit_switch_driver_state().triggered[axis_index];
}

bool wait_for_limit_state(const std::size_t axis_index, const bool triggered) {
    vTaskDelay(pdMS_TO_TICKS(10));
    return axis_limit_is_triggered(axis_index) == triggered;
}

bool run_axis_homing(const std::size_t axis_index) {
    const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis_index));
    const bool seek_direction = hardware.homing_seek_positive_direction;
    if (axis_limit_is_triggered(axis_index)) return false;
    if (run_axis_motion(axis_index, seek_direction,
                        hardware.max_travel_mm * kSeekDistanceMultiplier,
                        kSeekRateMmPerMin, true) != SystemMotionResult::LimitTriggered) return false;
    if (run_axis_motion(axis_index, !seek_direction, kPullOffMm,
                        kLocateRateMmPerMin, false) != SystemMotionResult::Completed ||
        !wait_for_limit_state(axis_index, false)) return false;
    if (run_axis_motion(axis_index, seek_direction, kPullOffMm * 2.0,
                        kLocateRateMmPerMin, true) != SystemMotionResult::LimitTriggered) return false;
    return run_axis_motion(axis_index, !seek_direction, kPullOffMm,
                           kLocateRateMmPerMin, false) == SystemMotionResult::Completed &&
           wait_for_limit_state(axis_index, false);
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
    const bool x_complete = run_axis_homing(static_cast<std::size_t>(machine::Axis::X));
    const bool y_complete = x_complete && run_axis_homing(static_cast<std::size_t>(machine::Axis::Y));
    const bool completed = y_complete && commit_machine_origin();
    s_stop_on_limit.store(false, std::memory_order_release);
    if (!completed) stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
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
           !s_limit_triggered.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
    stepper::timer_stepper_executor_stop();
    s_seek_active.store(false, std::memory_order_release);
    s_stop_on_limit.store(false, std::memory_order_release);
    // A single-axis diagnostic establishes no machine coordinate reference.
    stepper::stepper_invalidate_position(stepper::PositionInvalidationReason::Reset);
    s_state.store(s_limit_triggered.load(std::memory_order_acquire) ? HomingState::Completed
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
    s_state.store(HomingState::Running, std::memory_order_release);
    if (xTaskCreate(homing_task, "homing", 3072, nullptr, 6, &s_seek_task) != pdPASS) {
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
    s_limit_triggered.store(false, std::memory_order_relaxed);
    s_stop_on_limit.store(true, std::memory_order_release);
    s_seek_active.store(true, std::memory_order_release);
    s_state.store(HomingState::Running, std::memory_order_release);
    if (xTaskCreate(seek_test_task, "homing_seek", 3072, nullptr, 6, &s_seek_task) != pdPASS) {
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
