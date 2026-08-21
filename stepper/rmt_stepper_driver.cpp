#include "rmt_stepper_driver.hpp"

#include <limits>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

#include "raycube_machine.hpp"
#include "stepper_driver.hpp"

namespace ray2::stepper {
namespace {

rmt_channel_handle_t s_rmt_channels[machine::kAxisCount] = {};
rmt_sync_manager_handle_t s_sync_manager = nullptr;
rmt_encoder_handle_t s_copy_encoder = nullptr;
bool s_initialized = false;

esp_err_t release_sync_manager() {
    if (s_sync_manager == nullptr) {
        return ESP_OK;
    }

    const esp_err_t result = rmt_del_sync_manager(s_sync_manager);
    if (result == ESP_OK) {
        s_sync_manager = nullptr;
    }
    return result;
}

void release_rmt_resources() {
    release_sync_manager();
    if (s_copy_encoder != nullptr) {
        rmt_del_encoder(s_copy_encoder);
        s_copy_encoder = nullptr;
    }

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        if (s_rmt_channels[axis] == nullptr) {
            continue;
        }
        rmt_disable(s_rmt_channels[axis]);
        rmt_del_channel(s_rmt_channels[axis]);
        s_rmt_channels[axis] = nullptr;
    }
}

}  // namespace

esp_err_t rmt_stepper_init() {
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t result = stepper_init();
    if (result != ESP_OK) {
        return result;
    }

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const auto& hardware = machine::raycube_axis(static_cast<machine::Axis>(axis));
        const rmt_tx_channel_config_t config = {
            .gpio_num = static_cast<gpio_num_t>(hardware.step_pin),
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = kRmtStepResolutionHz,
            .mem_block_symbols = kRmtStepMemorySymbols,
            .trans_queue_depth = 1,
            .intr_priority = 0,
            .flags = {},
        };
        result = rmt_new_tx_channel(&config, &s_rmt_channels[axis]);
        if (result != ESP_OK) {
            release_rmt_resources();
            return result;
        }

        result = rmt_enable(s_rmt_channels[axis]);
        if (result != ESP_OK) {
            release_rmt_resources();
            return result;
        }
    }

    const rmt_sync_manager_config_t sync_config = {
        .tx_channel_array = s_rmt_channels,
        .array_size = machine::kAxisCount,
    };
    result = rmt_new_sync_manager(&sync_config, &s_sync_manager);
    if (result != ESP_OK) {
        release_rmt_resources();
        return result;
    }

    const rmt_copy_encoder_config_t encoder_config = {};
    result = rmt_new_copy_encoder(&encoder_config, &s_copy_encoder);
    if (result != ESP_OK) {
        release_rmt_resources();
        return result;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t rmt_stepper_deinit() {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    release_rmt_resources();
    s_initialized = false;
    return ESP_OK;
}

bool rmt_stepper_is_initialized() {
    return s_initialized;
}

bool rmt_stepper_can_execute(std::int32_t step_count, std::uint32_t step_interval_us) {
    return step_count > 0 && static_cast<std::size_t>(step_count) <= kRmtStepMemorySymbols &&
           step_interval_us > machine::kStepPulseMicroseconds &&
           step_interval_us <= kRmtStepMaxIntervalUs;
}

esp_err_t rmt_stepper_execute(machine::Axis axis,
                               std::int32_t step_count,
                               std::uint32_t step_interval_us) {
    const auto axis_index = static_cast<std::size_t>(axis);
    if (axis_index >= machine::kAxisCount ||
        !rmt_stepper_can_execute(step_count, step_interval_us)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = rmt_stepper_init();
    if (result != ESP_OK) {
        return result;
    }

    // A synchronized TX group requires both X and Y transactions. Single-axis
    // blocks must run on their channel alone until dual-axis RMT is introduced.
    result = release_sync_manager();
    if (result != ESP_OK) {
        return result;
    }

    rmt_symbol_word_t symbols[kRmtStepMemorySymbols] = {};
    const rmt_symbol_word_t step_symbol = {
        .duration0 = machine::kStepPulseMicroseconds,
        .level0 = 1,
        .duration1 = static_cast<std::uint16_t>(
            step_interval_us - machine::kStepPulseMicroseconds),
        .level1 = 0,
    };
    for (std::int32_t step = 0; step < step_count; ++step) {
        symbols[step] = step_symbol;
    }

    const rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };
    result = rmt_transmit(s_rmt_channels[axis_index], s_copy_encoder, symbols,
                          static_cast<std::size_t>(step_count) * sizeof(rmt_symbol_word_t),
                          &transmit_config);
    if (result != ESP_OK) {
        return result;
    }

    const std::uint64_t duration_us =
        static_cast<std::uint64_t>(step_count) * step_interval_us;
    const std::uint64_t timeout_ms = duration_us / 1000U + 100U;
    if (timeout_ms > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        return ESP_ERR_INVALID_ARG;
    }
    return rmt_tx_wait_all_done(s_rmt_channels[axis_index], static_cast<int>(timeout_ms));
}

}  // namespace ray2::stepper
