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
JobState sd_job_state();
bool sd_job_is_active();
const char* sd_job_state_text(JobState state);
std::uint32_t sd_job_current_line();
JobFault sd_job_fault();
const char* sd_job_fault_text(JobFault fault);
command::Error sd_job_command_error();

}  // namespace ray2::sd
