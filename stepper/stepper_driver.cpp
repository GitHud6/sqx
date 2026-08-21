#include "stepper_driver.hpp"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

namespace ray2::stepper {
namespace {

bool s_initialized = false;

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
    return set_output_level(hardware->direction_pin, direction_level);
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

    esp_rom_delay_us(machine::kStepPulseMicroseconds);
    return set_output_level(hardware->step_pin, false);
}

}  // namespace ray2::stepper
