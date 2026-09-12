#include "service.hpp"

#include <atomic>
#include <cstddef>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "command_dispatch.hpp"
#include "homing_service.hpp"
#include "laser_driver.hpp"
#include "planner.hpp"
#include "runtime_affinity.hpp"
#include "sd_job_service.hpp"
#include "timer_stepper_executor.hpp"
#include "uart0_driver.hpp"

namespace ray2::service {
namespace {

constexpr char kLogTag[] = "service";
constexpr std::size_t kEventQueueLength = 16;
constexpr std::uint32_t kTaskStackBytes = 3072;

struct ServiceEvent {
    EventType type = EventType::StartRequested;
    FaultReason reason = FaultReason::None;
};

TaskHandle_t s_task = nullptr;
QueueHandle_t s_event_queue = nullptr;
std::atomic<State> s_state = State::Idle;
std::atomic<FaultReason> s_fault_reason = FaultReason::None;

bool event_allowed(const State state, const EventType type) {
    switch (type) {
        case EventType::StartRequested: return state == State::Idle;
        case EventType::StartConfirmed: return state == State::Starting;
        case EventType::HoldRequested: return state == State::Running;
        case EventType::HoldReached: return state == State::HoldRequested;
        case EventType::ResumeRequested: return state == State::Held;
        case EventType::ResumeConfirmed: return state == State::Resuming;
        case EventType::Completed: return state == State::Running || state == State::Resuming;
        case EventType::Aborted:
            return state == State::Starting || state == State::Running ||
                   state == State::HoldRequested || state == State::Held ||
                   state == State::Resuming;
        case EventType::Faulted:
            return state != State::Completed && state != State::Aborted &&
                   state != State::Faulted;
        case EventType::ResetToIdle:
            return state == State::Completed || state == State::Aborted ||
                   state == State::Faulted;
        case EventType::HardResetRequested: return true;
    }
    return false;
}

void reject_event(const EventType type) {
    s_fault_reason.store(FaultReason::InvalidTransition, std::memory_order_release);
    ESP_LOGW(kLogTag, "event rejected: state=%s event=%u",
             service_state_text(s_state.load(std::memory_order_acquire)),
             static_cast<unsigned>(type));
}

void apply_hard_reset() {
    // The realtime ingress path has already made pulse and laser output safe.
    // Keep all queue, task, and parser cleanup in this non-realtime task.
    ray2::planner::planner_reset();
    ray2::protocol::command_reset_after_hard_reset();
    ray2::laser::laser_set_motion_output_permitted(false);
    s_fault_reason.store(FaultReason::HardReset, std::memory_order_release);
    s_state.store(State::Aborted, std::memory_order_release);
    ESP_LOGW(kLogTag, "hard reset completed");
}

void apply_event(const ServiceEvent& event) {
    if (event.type == EventType::HardResetRequested) {
        apply_hard_reset();
        return;
    }
    const State current = s_state.load(std::memory_order_acquire);
    if (!event_allowed(current, event.type)) {
        reject_event(event.type);
        return;
    }

    switch (event.type) {
        case EventType::StartRequested: s_state.store(State::Starting); break;
        case EventType::StartConfirmed: s_state.store(State::Running); break;
        case EventType::HoldRequested: s_state.store(State::HoldRequested); break;
        case EventType::HoldReached: s_state.store(State::Held); break;
        case EventType::ResumeRequested: s_state.store(State::Resuming); break;
        case EventType::ResumeConfirmed: s_state.store(State::Running); break;
        case EventType::Completed: s_state.store(State::Completed); break;
        case EventType::Aborted: s_state.store(State::Aborted); break;
        case EventType::Faulted:
            s_fault_reason.store(event.reason == FaultReason::None ? FaultReason::General
                                                                     : event.reason,
                                 std::memory_order_release);
            s_state.store(State::Faulted);
            break;
        case EventType::ResetToIdle:
            s_fault_reason.store(FaultReason::None, std::memory_order_release);
            s_state.store(State::Idle);
            break;
        case EventType::HardResetRequested:
            // Handled before transition validation above.
            return;
    }
}

void service_task(void*) {
    ServiceEvent event = {};
    while (true) {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            apply_event(event);
        }
    }
}

}  // namespace

esp_err_t service_start() {
    if (s_task != nullptr) return ESP_ERR_INVALID_STATE;

    s_state.store(State::Idle, std::memory_order_release);
    s_fault_reason.store(FaultReason::None, std::memory_order_release);
    s_event_queue = xQueueCreate(kEventQueueLength, sizeof(ServiceEvent));
    if (s_event_queue == nullptr) return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(service_task, "service", kTaskStackBytes, nullptr, 4, &s_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        vQueueDelete(s_event_queue);
        s_event_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(kLogTag, "started state=idle");
    return ESP_OK;
}

bool service_is_started() {
    return s_task != nullptr;
}

bool service_post_event(const EventType type, const FaultReason reason) {
    if (s_event_queue == nullptr || s_task == nullptr) return false;

    const ServiceEvent event = {.type = type, .reason = reason};
    if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
        s_fault_reason.store(FaultReason::EventQueueFull, std::memory_order_release);
        return false;
    }
    return true;
}

bool service_request_hard_reset() {
    if (s_event_queue == nullptr || s_task == nullptr) return false;

    // This makes the output safe before the service task can be scheduled.
    // The stop function is task-context only and must not be called by an ISR.
    ray2::laser::laser_set_motion_output_permitted(false);
    // These are atomic cancellation requests. Send them before stopping the
    // executor so neither producer can schedule another action in the gap.
    ray2::sd::sd_job_request_abort();
    ray2::homing::homing_request_abort();
    ray2::stepper::timer_stepper_executor_stop();
    ray2::driver::uart0_discard_pending_normal_input();
    s_fault_reason.store(FaultReason::HardReset, std::memory_order_release);
    s_state.store(State::Aborted, std::memory_order_release);
    // Hard reset supersedes every previously queued lifecycle event. This
    // also guarantees that cleanup cannot be starved by a future full queue.
    xQueueReset(s_event_queue);
    return service_post_event(EventType::HardResetRequested, FaultReason::HardReset);
}

State service_state() {
    return s_state.load(std::memory_order_acquire);
}

const char* service_state_text(const State state) {
    switch (state) {
        case State::Idle: return "idle";
        case State::Starting: return "starting";
        case State::Running: return "running";
        case State::HoldRequested: return "hold-requested";
        case State::Held: return "held";
        case State::Resuming: return "resuming";
        case State::Completed: return "completed";
        case State::Aborted: return "aborted";
        case State::Faulted: return "faulted";
    }
    return "unknown";
}

FaultReason service_fault_reason() {
    return s_fault_reason.load(std::memory_order_acquire);
}

const char* service_fault_reason_text(const FaultReason reason) {
    switch (reason) {
        case FaultReason::None: return "none";
        case FaultReason::HardReset: return "hard-reset";
        case FaultReason::InvalidTransition: return "invalid-transition";
        case FaultReason::EventQueueFull: return "event-queue-full";
        case FaultReason::General: return "general";
    }
    return "unknown";
}

}  // namespace ray2::service
