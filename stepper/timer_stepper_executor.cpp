#include "timer_stepper_executor.hpp"

#include <atomic>

namespace ray2::stepper {
namespace {

std::atomic<bool> s_initialized = false;

}  // namespace

esp_err_t timer_stepper_executor_init() {
    s_initialized.store(true);
    return ESP_OK;
}

TimerStepperDiagnostics timer_stepper_executor_diagnostics() {
    TimerStepperDiagnostics diagnostics = {};
    diagnostics.initialized = s_initialized.load();
    return diagnostics;
}

}  // namespace ray2::stepper
