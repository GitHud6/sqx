#include "sd_job_service.hpp"

#include <atomic>
#include <cstring>

#include "command_dispatch.hpp"
#include "command_executor.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "planner.hpp"
#include "plain_file_reader.hpp"
#include "gzip_file_reader.hpp"
#include "sdcard_driver.hpp"
#include "stepper_driver.hpp"

namespace ray2::sd {
namespace {

constexpr char kLogTag[] = "sd_job";
constexpr std::size_t kJobPathCapacity = kFilePathCapacity;

TaskHandle_t s_job_task = nullptr;
portMUX_TYPE s_job_lock = portMUX_INITIALIZER_UNLOCKED;
std::atomic<JobState> s_job_state = JobState::Idle;
std::atomic<JobFault> s_job_fault = JobFault::None;
std::atomic<command::Error> s_job_command_error = command::Error::Ok;
std::atomic<std::uint32_t> s_job_current_line = 0;
std::atomic<bool> s_abort_requested = false;
char s_job_path[kJobPathCapacity] = {};

// The gzip dictionary is 32 KiB. Keep reader storage outside the 4 KiB job
// task stack; only one SD job can be active at a time.
PlainFileReader s_plain_reader;
GzipFileReader s_gzip_reader;

bool copy_path(char* destination, const char* source) {
    if (destination == nullptr || source == nullptr || source[0] != '/') {
        return false;
    }
    const std::size_t length = std::strlen(source);
    if (length == 0 || length >= kJobPathCapacity) {
        return false;
    }
    std::memcpy(destination, source, length + 1);
    return true;
}

bool has_gzip_suffix(const char* path) {
    const std::size_t length = std::strlen(path);
    return length >= 3 && path[length - 3] == '.' &&
           (path[length - 2] == 'g' || path[length - 2] == 'G') &&
           (path[length - 1] == 'z' || path[length - 1] == 'Z');
}

const char* position_state_text(stepper::PositionState state) {
    switch (state) {
        case stepper::PositionState::Unknown: return "unknown";
        case stepper::PositionState::Known: return "known";
        case stepper::PositionState::Invalidated: return "invalidated";
    }
    return "invalid";
}

void log_position(const char* lifecycle) {
    const stepper::MachinePosition position = stepper::machine_position();
    ESP_LOGI(kLogTag,
             "SD-POSITION lifecycle=%s state=%s steps=(%lld,%lld) mm=(%.4f,%.4f)",
             lifecycle,
             position_state_text(position.state),
             static_cast<long long>(position.steps[0]),
             static_cast<long long>(position.steps[1]),
             position.millimeters[0],
             position.millimeters[1]);
}

void finish_job(JobState state,
                JobFault fault = JobFault::None,
                command::Error command_error = command::Error::Ok) {
    // A realtime Ctrl-X has already synchronously reset motion before asking
    // the SD feeder to exit. Avoid a second concurrent executor reset here.
    if (state == JobState::Faulted && fault != JobFault::Aborted) {
        ray2::planner::planner_reset();
    }
    log_position(state == JobState::Completed ? "completed" : "faulted");
    s_job_fault.store(fault);
    s_job_command_error.store(command_error);
    s_job_state.store(state);
    s_job_task = nullptr;
    vTaskDelete(nullptr);
}

void sd_job_task(void*) {
    char path[kJobPathCapacity] = {};
    portENTER_CRITICAL(&s_job_lock);
    std::memcpy(path, s_job_path, sizeof(path));
    portEXIT_CRITICAL(&s_job_lock);

    FileReader* reader = has_gzip_suffix(path)
                             ? static_cast<FileReader*>(&s_gzip_reader)
                             : static_cast<FileReader*>(&s_plain_reader);
    s_job_state.store(JobState::Starting);
    log_position("starting");
    if (!reader->open(path)) {
        finish_job(JobState::Faulted, JobFault::OpenFailed);
        return;
    }

    s_job_state.store(JobState::Running);
    log_position("running");
    char line[kGcodeLineCapacity] = {};
    bool program_ended = false;
    while (true) {
        if (s_abort_requested.load(std::memory_order_acquire)) {
            reader->close();
            finish_job(JobState::Faulted, JobFault::Aborted);
            return;
        }
        if (reader->read_line(line, sizeof(line))) {
            s_job_current_line.fetch_add(1);
            if (line[0] == '\0') {
                continue;
            }

            while (true) {
                if (s_abort_requested.load(std::memory_order_acquire)) {
                    reader->close();
                    finish_job(JobState::Faulted, JobFault::Aborted);
                    return;
                }
                const command::Error result =
                    ray2::protocol::command_dispatch_external(line, std::strlen(line));
                if (result == command::Error::Ok) {
                    break;
                }
                if (result == command::Error::ProgramEnd) {
                    program_ended = true;
                    break;
                }
                if (result == command::Error::MotionBusy) {
                    ray2::planner::planner_wait_for_free_block();
                    continue;
                }
                ESP_LOGE(kLogTag,
                         "SD-GCODE-REJECT line=%u error=%s payload=\"%.*s\"",
                         static_cast<unsigned>(s_job_current_line.load()),
                         command::error_text(result),
                         static_cast<int>(std::strlen(line)),
                         line);
                reader->close();
                finish_job(JobState::Faulted, JobFault::CommandRejected, result);
                return;
            }
            if (program_ended) {
                break;
            }
            continue;
        }

        if (reader->has_error()) {
            reader->close();
            finish_job(JobState::Faulted, JobFault::ReadFailed);
            return;
        }
        break;
    }

    reader->close();
    s_job_state.store(JobState::Draining);
    log_position("draining");
    while (ray2::planner::planner_free_blocks() != ray2::planner::kPlannerQueueLength) {
        if (s_abort_requested.load(std::memory_order_acquire)) {
            finish_job(JobState::Faulted, JobFault::Aborted);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    finish_job(JobState::Completed);
}

}  // namespace

esp_err_t sd_job_start(const char* path) {
    if (!ray2::driver::sdcard_is_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ray2::stepper::machine_position_state() != ray2::stepper::PositionState::Known) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sd_job_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    char normalized_path[kJobPathCapacity] = {};
    if (!copy_path(normalized_path, path)) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_job_lock);
    std::memcpy(s_job_path, normalized_path, sizeof(s_job_path));
    portEXIT_CRITICAL(&s_job_lock);
    s_job_state.store(JobState::Starting);
    s_job_fault.store(JobFault::None);
    s_job_command_error.store(command::Error::Ok);
    s_job_current_line.store(0);
    s_abort_requested.store(false, std::memory_order_release);

    if (xTaskCreate(sd_job_task, "sd_job", 4096, nullptr, 5, &s_job_task) != pdPASS) {
        s_job_state.store(JobState::Faulted);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void sd_job_request_abort() {
    if (sd_job_is_active()) {
        s_abort_requested.store(true, std::memory_order_release);
    }
}

JobState sd_job_state() {
    return s_job_state.load();
}

bool sd_job_is_active() {
    const JobState state = sd_job_state();
    return state == JobState::Starting || state == JobState::Running || state == JobState::Draining;
}

const char* sd_job_state_text(JobState state) {
    switch (state) {
        case JobState::Idle: return "idle";
        case JobState::Starting: return "starting";
        case JobState::Running: return "running";
        case JobState::Draining: return "draining";
        case JobState::Completed: return "completed";
        case JobState::Faulted: return "faulted";
    }
    return "unknown";
}

std::uint32_t sd_job_current_line() {
    return s_job_current_line.load();
}

JobFault sd_job_fault() {
    return s_job_fault.load();
}

const char* sd_job_fault_text(JobFault fault) {
    switch (fault) {
        case JobFault::None: return "none";
        case JobFault::OpenFailed: return "open-failed";
        case JobFault::ReadFailed: return "read-failed";
        case JobFault::CommandRejected: return "command-rejected";
        case JobFault::Aborted: return "aborted";
    }
    return "unknown";
}

command::Error sd_job_command_error() {
    return s_job_command_error.load();
}

}  // namespace ray2::sd
