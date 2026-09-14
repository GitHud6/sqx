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
#include "sdcard_driver.hpp"
#include "sd_job_service.hpp"
#include "timer_stepper_executor.hpp"
#include "uart0_driver.hpp"

namespace ray2::service {
namespace {

constexpr char kLogTag[] = "service";
constexpr std::size_t kEventQueueLength = 16;
constexpr std::uint32_t kTaskStackBytes = 3072;
constexpr TickType_t kStoragePollTicks = pdMS_TO_TICKS(50);
constexpr unsigned kCardDebounceSamples = 3;
constexpr TickType_t kMountRetryTicks = pdMS_TO_TICKS(1000);

struct ServiceEvent {
    EventType type = EventType::StartRequested;
    FaultReason reason = FaultReason::None;
    InputSource source = InputSource::Unknown;
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
        case EventType::ResumeRequested: return state == State::Resuming;
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
        case EventType::InputStarted:
            return state == State::Idle || state == State::Completed ||
                   state == State::Aborted || state == State::Faulted;
        case EventType::InputDataAvailable:
            return state == State::Starting || state == State::Running;
        case EventType::InputEnded:
            return state == State::Starting || state == State::Running ||
                   state == State::HoldRequested || state == State::Held;
        case EventType::InputAborted:
            return state == State::Starting || state == State::Running ||
                   state == State::HoldRequested || state == State::Held ||
                   state == State::Resuming;
        case EventType::InputFaulted:
            return state != State::Completed && state != State::Aborted &&
                   state != State::Faulted;
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

    if (event.type == EventType::InputStarted) {
        s_fault_reason.store(FaultReason::None, std::memory_order_release);
        s_state.store(State::Starting, std::memory_order_release);
        return;
    }
    if (event.type == EventType::InputDataAvailable) {
        s_state.store(State::Running, std::memory_order_release);
        return;
    }
    if (event.type == EventType::InputEnded) {
        // InputEnded means that no more commands will arrive. Already
        // submitted planner/segment work is intentionally left intact; the
        // producer will publish Completed after the normal drain.
        return;
    }
    if (event.type == EventType::InputAborted) {
        s_state.store(State::Aborted, std::memory_order_release);
        return;
    }
    if (event.type == EventType::InputFaulted) {
        s_fault_reason.store(event.reason == FaultReason::None ? FaultReason::General
                                                                 : event.reason,
                             std::memory_order_release);
        s_state.store(State::Faulted, std::memory_order_release);
        return;
    }

    if (event.type == EventType::ResumeRequested) {
        ray2::laser::laser_set_motion_output_permitted(true);
        if (ray2::stepper::timer_stepper_executor_resume_from_hold() == ESP_OK) {
            s_state.store(State::Running, std::memory_order_release);
        } else {
            ray2::laser::laser_set_motion_output_permitted(false);
            s_state.store(State::Held, std::memory_order_release);
        }
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
        case EventType::InputStarted:
        case EventType::InputDataAvailable:
        case EventType::InputEnded:
        case EventType::InputAborted:
        case EventType::InputFaulted:
            // Handled by the input-lifecycle branches above. Keep the switch
            // exhaustive so future enum additions remain compiler-checked.
            return;
    }
}

void service_task(void*) {
    ServiceEvent event = {};
    bool presence_initialized = false;
    bool stable_inserted = false;
    bool candidate_inserted = false;
    unsigned candidate_samples = 0;
    TickType_t next_mount_retry = 0;

    while (true) {
        if (xQueueReceive(s_event_queue, &event, kStoragePollTicks) == pdTRUE) {
            apply_event(event);
        }

        // Card detection is deliberately outside the pulse ISR. A removal
        // request only stops the SD feeder; it never resets planner motion.
        const bool inserted = ray2::driver::sdcard_is_inserted();
        if (!presence_initialized) {
            stable_inserted = inserted;
            candidate_inserted = inserted;
            candidate_samples = kCardDebounceSamples;
            presence_initialized = true;
        } else {
            if (inserted != candidate_inserted) {
                candidate_inserted = inserted;
                candidate_samples = 1;
            } else if (candidate_samples < kCardDebounceSamples) {
                ++candidate_samples;
            }
            if (candidate_samples >= kCardDebounceSamples &&
                candidate_inserted != stable_inserted) {
                stable_inserted = candidate_inserted;
                if (!stable_inserted) {
                    if (ray2::sd::sd_job_is_active()) {
                        ESP_LOGI(kLogTag, "sd card removed: input-ended, draining motion");
                        ray2::sd::sd_job_request_input_end();
                    } else if (ray2::driver::sdcard_is_mounted()) {
                        ray2::driver::sdcard_stop();
                    }
                } else if (!ray2::driver::sdcard_is_mounted()) {
                    ESP_LOGI(kLogTag, "sd card inserted: remount pending");
                    next_mount_retry = 0;
                }
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if (stable_inserted && !ray2::driver::sdcard_is_mounted() &&
            !ray2::sd::sd_job_is_active() &&
            static_cast<std::int32_t>(now - next_mount_retry) >= 0) {
            // Covers boot without a card and a mount attempt that raced the
            // first stable detect sample.
            ray2::driver::sdcard_start();
            next_mount_retry = now + kMountRetryTicks;
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

    const ServiceEvent event = {.type = type, .reason = reason, .source = InputSource::Unknown};
    if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
        s_fault_reason.store(FaultReason::EventQueueFull, std::memory_order_release);
        return false;
    }
    return true;
}

namespace {
bool post_input_event(const EventType type, const InputSource source, const FaultReason reason) {
    if (s_event_queue == nullptr || s_task == nullptr) return false;
    const ServiceEvent event = {.type = type, .reason = reason, .source = source};
    return xQueueSend(s_event_queue, &event, 0) == pdTRUE;
}
}  // namespace

bool service_input_started(const InputSource source) {
    return post_input_event(EventType::InputStarted, source, FaultReason::None);
}

bool service_input_data_available(const InputSource source) {
    return post_input_event(EventType::InputDataAvailable, source, FaultReason::None);
}

bool service_input_ended(const InputSource source) {
    return post_input_event(EventType::InputEnded, source, FaultReason::None);
}

bool service_input_aborted(const InputSource source) {
    return post_input_event(EventType::InputAborted, source, FaultReason::None);
}

bool service_input_faulted(const InputSource source, const FaultReason reason) {
    return post_input_event(EventType::InputFaulted, source, reason);
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

bool service_request_feed_hold() {
    if (ray2::planner::planner_is_idle()) return false;
    State state = s_state.load(std::memory_order_acquire);
    const State previous_state = state;
    while (true) {
        if (state == State::HoldRequested || state == State::Held) return true;
        if (state == State::Aborted || state == State::Faulted || state == State::Completed ||
            state == State::Resuming) {
            return false;
        }
        if (s_state.compare_exchange_weak(state, State::HoldRequested,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire)) {
            break;
        }
    }

    ray2::laser::laser_set_motion_output_permitted(false);
    if (ray2::stepper::timer_stepper_executor_request_hold()) return true;

    // No segment was active to own a deterministic boundary. Leave state
    // unchanged rather than allowing a stale hold to affect later motion.
    State expected = State::HoldRequested;
    s_state.compare_exchange_strong(expected, previous_state, std::memory_order_acq_rel,
                                    std::memory_order_acquire);
    ray2::laser::laser_set_motion_output_permitted(true);
    return false;
}

bool service_request_cycle_start() {
    State expected = State::Held;
    if (!s_state.compare_exchange_strong(expected, State::Resuming,
                                         std::memory_order_acq_rel,
                                         std::memory_order_acquire)) {
        return false;
    }
    if (service_post_event(EventType::ResumeRequested)) return true;
    s_state.store(State::Held, std::memory_order_release);
    return false;
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
