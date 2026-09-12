#pragma once

#include <cstdint>

#include "esp_err.h"

namespace ray2::service {

enum class State : std::uint8_t {
    Idle,
    Starting,
    Running,
    HoldRequested,
    Held,
    Resuming,
    Completed,
    Aborted,
    Faulted,
};

enum class FaultReason : std::uint8_t {
    None,
    HardReset,
    InvalidTransition,
    EventQueueFull,
    General,
};

enum class EventType : std::uint8_t {
    StartRequested,
    StartConfirmed,
    HoldRequested,
    HoldReached,
    ResumeRequested,
    ResumeConfirmed,
    Completed,
    Aborted,
    Faulted,
    ResetToIdle,
    HardResetRequested,
};

esp_err_t service_start();
bool service_is_started();

// Events are serialized by the service task. The function is non-blocking and
// returns false only when the queue is unavailable/full; an illegal transition
// is rejected when the service task processes the event.
bool service_post_event(EventType type, FaultReason reason = FaultReason::None);

// Called from the UART realtime ingress task. It immediately disables laser
// output and stops pulse generation, then queues the remaining reset cleanup
// for the service task. It never runs in a hardware ISR.
bool service_request_hard_reset();

State service_state();
const char* service_state_text(State state);
FaultReason service_fault_reason();
const char* service_fault_reason_text(FaultReason reason);

}  // namespace ray2::service
