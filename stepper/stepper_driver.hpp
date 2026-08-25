#pragma once

#include <array>
#include <cstdint>

#include "esp_err.h"

#include "raycube_machine.hpp"

namespace ray2::stepper {

enum class Direction : bool {
    Negative = false,
    Positive = true,
};

enum class PositionState {
    Unknown,
    Known,
    Invalidated,
};

enum class PositionInvalidationReason {
    None,
    Startup,
    Reset,
    RmtFailure,
    ExecutionFailure,
};

struct MachinePosition {
    std::array<std::int64_t, machine::kAxisCount> steps = {};
    std::array<double, machine::kAxisCount> millimeters = {};
    PositionState state = PositionState::Unknown;
    PositionInvalidationReason invalidation_reason = PositionInvalidationReason::Startup;
};

esp_err_t stepper_init();
esp_err_t stepper_set_enabled(machine::Axis axis, bool enabled);
esp_err_t stepper_set_direction(machine::Axis axis, Direction direction);
esp_err_t stepper_pulse(machine::Axis axis);

std::int64_t machine_position_steps(machine::Axis axis);
double machine_position_mm(machine::Axis axis);
MachinePosition machine_position();
PositionState machine_position_state();
esp_err_t stepper_set_machine_position_mm(const double position_mm[machine::kAxisCount]);

// Stepper execution backends call these only after a STEP pulse or RMT batch
// has actually completed. Higher layers must not modify executed position.
void stepper_record_executed_steps(machine::Axis axis, std::int32_t step_count);
void stepper_record_executed_steps_from_isr(machine::Axis axis, std::int32_t step_count);
void stepper_invalidate_position(
    PositionInvalidationReason reason = PositionInvalidationReason::ExecutionFailure);

}  // namespace ray2::stepper
