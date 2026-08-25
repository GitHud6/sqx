#include "stepper_driver.hpp"

#include <array>
#include <cmath>
#include <limits>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

namespace ray2::stepper {
namespace {

bool s_initialized = false;
std::array<Direction, machine::kAxisCount> s_directions = {
    Direction::Positive,
    Direction::Positive,
};
std::array<std::int64_t, machine::kAxisCount> s_machine_position_steps = {};
PositionState s_position_state = PositionState::Unknown;
PositionInvalidationReason s_position_invalidation_reason = PositionInvalidationReason::Startup;
portMUX_TYPE s_position_lock = portMUX_INITIALIZER_UNLOCKED;

esp_err_t set_output_level(int pin, bool high) {
    return gpio_set_level(static_cast<gpio_num_t>(pin), high ? 1 : 0);
}

const machine::AxisHardware* axis_hardware(machine::Axis axis) {
    const auto index = static_cast<std::size_t>(axis);
    if (index >= machine::kAxisCount) {
        return nullptr;
    }

    return &machine::raycube_axis(axis);
}

bool axis_index(machine::Axis axis, std::size_t& index) {
    index = static_cast<std::size_t>(axis);
    return index < machine::kAxisCount;
}

void record_steps_locked(std::size_t index, std::int32_t step_count) {
    const std::int64_t signed_steps = s_directions[index] == Direction::Positive
                                          ? static_cast<std::int64_t>(step_count)
                                          : -static_cast<std::int64_t>(step_count);
    s_machine_position_steps[index] += signed_steps;
}

}  // namespace

esp_err_t stepper_init() {
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t output_config = {};
    output_config.mode = GPIO_MODE_OUTPUT;

    for (std::size_t index = 0; index < machine::kAxisCount; ++index) {
        const auto& axis = machine::raycube_axis(static_cast<machine::Axis>(index));
        output_config.pin_bit_mask |= (1ULL << axis.step_pin);
        output_config.pin_bit_mask |= (1ULL << axis.direction_pin);
        output_config.pin_bit_mask |= (1ULL << axis.disable_pin);
    }

    esp_err_t result = gpio_config(&output_config);
    if (result != ESP_OK) {
        return result;
    }

    for (std::size_t index = 0; index < machine::kAxisCount; ++index) {
        const auto& axis = machine::raycube_axis(static_cast<machine::Axis>(index));

        result = set_output_level(axis.step_pin, false);
        if (result != ESP_OK) {
            return result;
        }

        result = set_output_level(axis.direction_pin, axis.direction_inverted);
        if (result != ESP_OK) {
            return result;
        }
        s_directions[index] = Direction::Positive;

        result = set_output_level(axis.disable_pin, axis.disable_active_high);
        if (result != ESP_OK) {
            return result;
        }
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t stepper_set_enabled(machine::Axis axis, bool enabled) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const auto* hardware = axis_hardware(axis);
    if (hardware == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool disable_level = enabled ? !hardware->disable_active_high : hardware->disable_active_high;
    return set_output_level(hardware->disable_pin, disable_level);
}

esp_err_t stepper_set_direction(machine::Axis axis, Direction direction) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const auto* hardware = axis_hardware(axis);
    if (hardware == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool direction_level =
        (direction == Direction::Positive) ^ hardware->direction_inverted;
    const esp_err_t result = set_output_level(hardware->direction_pin, direction_level);
    if (result == ESP_OK) {
        const std::size_t index = static_cast<std::size_t>(axis);
        portENTER_CRITICAL(&s_position_lock);
        s_directions[index] = direction;
        portEXIT_CRITICAL(&s_position_lock);
    }
    return result;
}

void stepper_set_direction_from_isr(const machine::Axis axis,
                                    const Direction direction) {
    const std::size_t index = static_cast<std::size_t>(axis);
    if (index >= machine::kAxisCount) {
        return;
    }
    const auto& hardware = machine::raycube_axis(axis);
    const bool direction_level =
        (direction == Direction::Positive) ^ hardware.direction_inverted;
    gpio_ll_set_level(&GPIO, hardware.direction_pin, direction_level ? 1U : 0U);
    portENTER_CRITICAL_ISR(&s_position_lock);
    s_directions[index] = direction;
    portEXIT_CRITICAL_ISR(&s_position_lock);
}

esp_err_t stepper_pulse(machine::Axis axis) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const auto* hardware = axis_hardware(axis);
    if (hardware == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = set_output_level(hardware->step_pin, true);
    if (result != ESP_OK) {
        return result;
    }

    stepper_record_executed_steps(axis, 1);

    esp_rom_delay_us(machine::kStepPulseMicroseconds);
    return set_output_level(hardware->step_pin, false);
}

std::int64_t machine_position_steps(machine::Axis axis) {
    std::size_t index = 0;
    if (!axis_index(axis, index)) {
        return 0;
    }

    portENTER_CRITICAL(&s_position_lock);
    const std::int64_t steps = s_machine_position_steps[index];
    portEXIT_CRITICAL(&s_position_lock);
    return steps;
}

double machine_position_mm(machine::Axis axis) {
    std::size_t index = 0;
    if (!axis_index(axis, index)) {
        return 0.0;
    }

    const float steps_per_mm = machine::raycube_axis(axis).steps_per_mm;
    if (steps_per_mm <= 0.0F) {
        return 0.0;
    }
    return static_cast<double>(machine_position_steps(axis)) / steps_per_mm;
}

MachinePosition machine_position() {
    MachinePosition position = {};
    portENTER_CRITICAL(&s_position_lock);
    position.steps = s_machine_position_steps;
    position.state = s_position_state;
    position.invalidation_reason = s_position_invalidation_reason;
    portEXIT_CRITICAL(&s_position_lock);

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const float steps_per_mm =
            machine::raycube_axis(static_cast<machine::Axis>(axis)).steps_per_mm;
        if (steps_per_mm > 0.0F) {
            position.millimeters[axis] = static_cast<double>(position.steps[axis]) / steps_per_mm;
        }
    }
    return position;
}

PositionState machine_position_state() {
    portENTER_CRITICAL(&s_position_lock);
    const PositionState state = s_position_state;
    portEXIT_CRITICAL(&s_position_lock);
    return state;
}

esp_err_t stepper_set_machine_position_mm(const double position_mm[machine::kAxisCount]) {
    if (position_mm == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<std::int64_t, machine::kAxisCount> steps = {};
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto current_axis = static_cast<machine::Axis>(axis);
        const auto& hardware = machine::raycube_axis(current_axis);
        if (!std::isfinite(position_mm[axis]) || position_mm[axis] < 0.0 ||
            position_mm[axis] > hardware.max_travel_mm || hardware.steps_per_mm <= 0.0F) {
            return ESP_ERR_INVALID_ARG;
        }

        const double requested_steps = position_mm[axis] * hardware.steps_per_mm;
        if (!std::isfinite(requested_steps) ||
            requested_steps > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
            return ESP_ERR_INVALID_ARG;
        }
        steps[axis] = std::llround(requested_steps);
    }

    portENTER_CRITICAL(&s_position_lock);
    s_machine_position_steps = steps;
    s_position_state = PositionState::Known;
    s_position_invalidation_reason = PositionInvalidationReason::None;
    portEXIT_CRITICAL(&s_position_lock);
    return ESP_OK;
}

void stepper_record_executed_steps(machine::Axis axis, std::int32_t step_count) {
    std::size_t index = 0;
    if (step_count <= 0 || !axis_index(axis, index)) {
        return;
    }

    portENTER_CRITICAL(&s_position_lock);
    record_steps_locked(index, step_count);
    portEXIT_CRITICAL(&s_position_lock);
}

void IRAM_ATTR stepper_record_executed_steps_from_isr(machine::Axis axis, std::int32_t step_count) {
    std::size_t index = 0;
    if (step_count <= 0 || !axis_index(axis, index)) return;
    portENTER_CRITICAL_ISR(&s_position_lock);
    record_steps_locked(index, step_count);
    portEXIT_CRITICAL_ISR(&s_position_lock);
}

void stepper_invalidate_position(const PositionInvalidationReason reason) {
    portENTER_CRITICAL(&s_position_lock);
    s_position_state = PositionState::Invalidated;
    s_position_invalidation_reason = reason;
    portEXIT_CRITICAL(&s_position_lock);
}

}  // namespace ray2::stepper
