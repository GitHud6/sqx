#include "service.hpp"

#include <atomic>
#include <cstddef>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "runtime_affinity.hpp"

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
    }
    return false;
}

void reject_event(const EventType type) {
    s_fault_reason.store(FaultReason::InvalidTransition, std::memory_order_release);
    ESP_LOGW(kLogTag, "event rejected: state=%s event=%u",
             service_state_text(s_state.load(std::memory_order_acquire)),
             static_cast<unsigned>(type));
}

void apply_event(const ServiceEvent& event) {
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
        case FaultReason::InvalidTransition: return "invalid-transition";
        case FaultReason::EventQueueFull: return "event-queue-full";
        case FaultReason::General: return "general";
    }
    return "unknown";
}

}  // namespace ray2::service
