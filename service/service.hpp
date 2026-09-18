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
    HardLimit,
    MotionAlarm,
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
    SafetyAbortRequested,
    InputStarted,
    InputDataAvailable,
    InputEnded,
    InputAborted,
    InputFaulted,
};

enum class InputSource : std::uint8_t {
    Unknown,
    Uart,
    Telnet,
    WebSocket,
    SdCard,
    FFat,
};

esp_err_t service_start();
bool service_is_started();

// Events are serialized by the service task. The function is non-blocking and
// returns false only when the queue is unavailable/full; an illegal transition
// is rejected when the service task processes the event.
bool service_post_event(EventType type, FaultReason reason = FaultReason::None);

// Input lifecycle notifications are source-neutral. The producer owns its
// reader; service only serializes lifecycle state and motion-drain policy.
bool service_input_started(InputSource source);
bool service_input_data_available(InputSource source);
bool service_input_ended(InputSource source);
bool service_input_aborted(InputSource source);
bool service_input_faulted(InputSource source, FaultReason reason = FaultReason::General);

// Normal command gate shared by every protocol adapter. Once an input producer
// claims a job, only that producer may submit G-code; all system/configuration
// commands are blocked until the job drains or is aborted. Realtime bytes use
// their dedicated ingress paths and intentionally bypass this gate.
bool service_normal_input_allowed(InputSource source, bool motion_line);

// Called from the UART realtime ingress task. It immediately disables laser
// output and stops pulse generation, then queues the remaining reset cleanup
// for the service task. It never runs in a hardware ISR.
bool service_request_hard_reset();
// Clears a completed abort/fault lifecycle gate without changing the machine
// coordinate trust state or re-enabling laser motion output.
bool service_request_alarm_unlock();
// Task-context safety stop for confirmed hard limits and motion alarms. It
// makes output safe immediately, then serializes destructive cleanup through
// the service task. It must not be called from a hardware ISR.
bool service_request_safety_abort(FaultReason reason);
// Realtime feed-hold controls. Hold completes the active segment before
// stopping; resume consumes only already-unexecuted segments.
bool service_request_feed_hold();
bool service_request_cycle_start();

State service_state();
const char* service_state_text(State state);
FaultReason service_fault_reason();
const char* service_fault_reason_text(FaultReason reason);

}  // namespace ray2::service
