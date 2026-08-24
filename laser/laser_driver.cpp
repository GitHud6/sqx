#include "laser_driver.hpp"

#include <atomic>
#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "raycube_machine.hpp"

namespace ray2::laser {
namespace {

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kLedcDutyResolution = LEDC_TIMER_10_BIT;
constexpr std::uint32_t kLedcMaxDuty = (1U << kLedcDutyResolution) - 1U;
bool s_initialized = false;
std::atomic<bool> s_output_enabled = false;
std::atomic<std::uint32_t> s_current_duty = 0;

int disabled_enable_level() {
    return machine::raycube_laser().enable_active_high ? 0 : 1;
}

esp_err_t force_disabled_output() {
    const auto& laser = machine::raycube_laser();
    esp_err_t result = gpio_set_level(static_cast<gpio_num_t>(laser.enable_pin), disabled_enable_level());
    if (result != ESP_OK) {
        return result;
    }
    result = ledc_stop(kLedcMode, kLedcChannel, laser.pwm_inverted ? 1 : 0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }
    s_output_enabled.store(false);
    s_current_duty.store(0);
    return ESP_OK;
}

std::uint32_t duty_for_power(double power) {
    const auto max_power = static_cast<double>(machine::raycube_laser().max_power);
    return static_cast<std::uint32_t>(std::lround((power / max_power) * kLedcMaxDuty));
}

}  // namespace

esp_err_t laser_init() {
    if (s_initialized) {
        return ESP_OK;
    }

    const auto& laser = machine::raycube_laser();
    const auto pwm_pin = static_cast<gpio_num_t>(laser.pwm_pin);
    const auto enable_pin = static_cast<gpio_num_t>(laser.enable_pin);

    gpio_config_t enable_config = {};
    enable_config.pin_bit_mask = 1ULL << laser.enable_pin;
    enable_config.mode = GPIO_MODE_OUTPUT;
    enable_config.pull_up_en = GPIO_PULLUP_DISABLE;
    enable_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enable_config.intr_type = GPIO_INTR_DISABLE;
    esp_err_t result = gpio_config(&enable_config);
    if (result != ESP_OK) {
        return result;
    }
    if ((result = gpio_set_level(enable_pin, disabled_enable_level())) != ESP_OK) {
        return result;
    }

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kLedcMode;
    timer_config.duty_resolution = kLedcDutyResolution;
    timer_config.timer_num = kLedcTimer;
    timer_config.freq_hz = laser.pwm_frequency_hz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    if ((result = ledc_timer_config(&timer_config)) != ESP_OK) {
        return result;
    }

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = pwm_pin;
    channel_config.speed_mode = kLedcMode;
    channel_config.channel = kLedcChannel;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = kLedcTimer;
    channel_config.duty = 0;
    channel_config.hpoint = 0;
    channel_config.flags.output_invert = laser.pwm_inverted ? 1 : 0;
    if ((result = ledc_channel_config(&channel_config)) != ESP_OK) {
        return result;
    }

    if ((result = force_disabled_output()) != ESP_OK) {
        return result;
    }
    s_initialized = true;
    return ESP_OK;
}

esp_err_t laser_deinit() {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t result = laser_disable();
    if (result == ESP_OK) {
        s_initialized = false;
    }
    return result;
}

bool laser_is_initialized() {
    return s_initialized;
}

esp_err_t laser_set_power(double power) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const auto& laser = machine::raycube_laser();
    if (!std::isfinite(power) || power < 0.0 || power > laser.max_power) {
        laser_disable();
        return ESP_ERR_INVALID_ARG;
    }
    if (power == 0.0) {
        return laser_disable();
    }

    const std::uint32_t duty = duty_for_power(power);
    esp_err_t result = ledc_set_duty(kLedcMode, kLedcChannel, duty);
    if (result == ESP_OK) {
        result = ledc_update_duty(kLedcMode, kLedcChannel);
    }
    if (result != ESP_OK) {
        laser_disable();
        return result;
    }
    const int enabled_level = laser.enable_active_high ? 1 : 0;
    if ((result = gpio_set_level(static_cast<gpio_num_t>(laser.enable_pin), enabled_level)) != ESP_OK) {
        laser_disable();
        return result;
    }

    s_current_duty.store(duty);
    s_output_enabled.store(true);
    return ESP_OK;
}

esp_err_t laser_disable() {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return force_disabled_output();
}

bool laser_is_output_enabled() {
    return s_output_enabled.load();
}

std::uint32_t laser_current_duty() {
    return s_current_duty.load();
}

}  // namespace ray2::laser
