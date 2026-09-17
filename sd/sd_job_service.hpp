#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "command_executor.hpp"

namespace ray2::sd {

enum class JobState {
    Idle,
    Starting,
    Running,
    Draining,
    Completed,
    Faulted,
};

enum class JobFault {
    None,
    OpenFailed,
    ReadFailed,
    CommandRejected,
    Aborted,
};

esp_err_t sd_job_start(const char* path);
void sd_job_request_abort();
// Stops reading new commands because the input medium ended or disappeared.
// Existing planner/segment work is drained normally.
void sd_job_request_input_end();
JobState sd_job_state();
bool sd_job_is_active();
const char* sd_job_state_text(JobState state);
std::uint32_t sd_job_current_line();
std::uint32_t sd_job_progress_tenths_percent();
JobFault sd_job_fault();
const char* sd_job_fault_text(JobFault fault);
command::Error sd_job_command_error();

}  // namespace ray2::sd
