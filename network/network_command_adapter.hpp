#pragma once

#include <cstddef>

#include "esp_err.h"

namespace laserctl::network {

enum class SubmitResult {
    Accepted,
    Busy,
    Invalid,
    Unavailable,
};

// Starts the bounded network-to-protocol ingress task. The adapter owns no
// motion state; normal lines are executed only through protocol/service.
esp_err_t network_command_adapter_start();
bool network_command_adapter_is_started();

// Submits decoded command text. Multi-line G-code command_text requests create
// one input lifecycle batch; standalone `$` requests remain control commands.
// Plain requests are restricted to one ordinary line.
SubmitResult network_command_adapter_submit(const char* text, std::size_t length,
                                            bool command_text);

// Drops queued network input after a hard reset or safety abort. This is safe
// from task context and deliberately does not alter planner state itself.
void network_command_adapter_abort_pending();

}  // namespace laserctl::network
