#include "rmt_stepper_driver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <limits>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "raycube_machine.hpp"
#include "stepper_driver.hpp"

namespace ray2::stepper {
namespace {

rmt_channel_handle_t s_rmt_channels[machine::kAxisCount] = {};
rmt_sync_manager_handle_t s_sync_manager = nullptr;
rmt_encoder_handle_t s_copy_encoder = nullptr;
std::atomic<bool> s_feed_hold_requested = false;
bool s_initialized = false;

struct SegmentPayload {
    rmt_symbol_word_t symbols[machine::kAxisCount][kRmtStepMemorySymbols] = {};
    std::size_t symbol_count = 0;
};

std::array<SegmentPayload, kRmtSegmentTransactionDepth> s_segment_payloads = {};

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

esp_err_t ensure_sync_manager() {
    if (s_sync_manager != nullptr) {
        return ESP_OK;
    }

    const rmt_sync_manager_config_t sync_config = {
        .tx_channel_array = s_rmt_channels,
        .array_size = machine::kAxisCount,
    };
    return rmt_new_sync_manager(&sync_config, &s_sync_manager);
}

rmt_symbol_word_t make_step_symbol(bool pulse, std::uint32_t interval_us) {
    if (pulse) {
        return {
            .duration0 = machine::kStepPulseMicroseconds,
            .level0 = 1,
            .duration1 = static_cast<std::uint16_t>(
                interval_us - machine::kStepPulseMicroseconds),
            .level1 = 0,
        };
    }

    const std::uint16_t first_half = static_cast<std::uint16_t>(interval_us / 2U);
    return {
        .duration0 = first_half,
        .level0 = 0,
        .duration1 = static_cast<std::uint16_t>(interval_us - first_half),
        .level1 = 0,
    };
}

esp_err_t wait_for_tx_done(machine::Axis axis,
                           std::int32_t symbol_count,
                           std::uint32_t interval_us) {
    const std::uint64_t duration_us =
        static_cast<std::uint64_t>(symbol_count) * interval_us;
    const std::uint64_t timeout_ms = duration_us / 1000U + 100U;
    if (timeout_ms > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        return ESP_ERR_INVALID_ARG;
    }
    return rmt_tx_wait_all_done(
        s_rmt_channels[static_cast<std::size_t>(axis)], static_cast<int>(timeout_ms));
}

const rmt_transmit_config_t kTransmitConfig = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0,
        .queue_nonblocking = 0,
    },
};

const rmt_transmit_config_t kQueuedTransmitConfig = {
    .loop_count = 0,
    .flags = {
        .eot_level = 0,
        .queue_nonblocking = 1,
    },
};

void wait_for_feed_hold_release() {
    while (s_feed_hold_requested.load()) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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

bool segment_interval(const StepSegment& segment, std::uint32_t& interval_us) {
    if (segment.major_steps <= 0 || segment.duration_us == 0) {
        return false;
    }
    interval_us = segment.duration_us / static_cast<std::uint32_t>(segment.major_steps);
    return interval_us > machine::kStepPulseMicroseconds && interval_us <= kRmtStepMaxIntervalUs;
}

void build_segment_payload(const StepSegment& segment, SegmentPayload& payload) {
    std::uint32_t interval_us = 0;
    segment_interval(segment, interval_us);
    payload.symbol_count = static_cast<std::size_t>(segment.major_steps);
    const std::int32_t x_steps = segment.axis_steps[0];
    const std::int32_t y_steps = segment.axis_steps[1];
    const bool x_is_major = x_steps >= y_steps;
    const std::int32_t minor_steps = x_is_major ? y_steps : x_steps;
    std::int64_t error = 0;
    for (std::int32_t step = 0; step < segment.major_steps; ++step) {
        bool x_pulse = x_is_major;
        bool y_pulse = !x_is_major;
        error += minor_steps;
        if (error * 2 >= segment.major_steps) {
            if (x_is_major) {
                y_pulse = true;
            } else {
                x_pulse = true;
            }
            error -= segment.major_steps;
        }
        payload.symbols[0][step] = make_step_symbol(x_pulse, interval_us);
        payload.symbols[1][step] = make_step_symbol(y_pulse, interval_us);
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
            .trans_queue_depth = kRmtSegmentTransactionDepth,
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

    result = ensure_sync_manager();
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
    return step_count > 0 && step_interval_us > machine::kStepPulseMicroseconds &&
           step_interval_us <= kRmtStepMaxIntervalUs;
}

bool rmt_stepper_can_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                std::uint32_t step_interval_us) {
    return steps != nullptr && rmt_stepper_can_execute(steps[0], step_interval_us) &&
           rmt_stepper_can_execute(steps[1], step_interval_us);
}

bool rmt_stepper_can_execute_segments(const StepSegment* segments, std::size_t segment_count) {
    if (segments == nullptr || segment_count == 0 ||
        segment_count > kRmtSegmentTransactionDepth) {
        return false;
    }
    for (std::size_t index = 0; index < segment_count; ++index) {
        std::uint32_t interval_us = 0;
        if (!segment_interval(segments[index], interval_us) ||
            segments[index].major_steps > static_cast<std::int32_t>(kRmtStepMemorySymbols) ||
            segments[index].axis_steps[0] < 0 || segments[index].axis_steps[1] < 0 ||
            std::max(segments[index].axis_steps[0], segments[index].axis_steps[1]) !=
                segments[index].major_steps) {
            return false;
        }
    }
    return true;
}

esp_err_t rmt_stepper_execute_segments(const StepSegment* segments, std::size_t segment_count) {
    if (!rmt_stepper_can_execute_segments(segments, segment_count)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = rmt_stepper_init();
    if (result != ESP_OK) {
        return result;
    }
    wait_for_feed_hold_release();

    bool x_active = false;
    bool y_active = false;
    std::int32_t emitted_steps[machine::kAxisCount] = {};
    for (std::size_t index = 0; index < segment_count; ++index) {
        x_active = x_active || segments[index].axis_steps[0] > 0;
        y_active = y_active || segments[index].axis_steps[1] > 0;
        emitted_steps[0] += segments[index].axis_steps[0];
        emitted_steps[1] += segments[index].axis_steps[1];
        build_segment_payload(segments[index], s_segment_payloads[index]);
    }

    if (!x_active && !y_active) {
        return ESP_ERR_INVALID_ARG;
    }
    if (x_active && y_active) {
        result = ensure_sync_manager();
        if (result != ESP_OK) {
            return result;
        }
        result = rmt_sync_reset(s_sync_manager);
        if (result != ESP_OK) {
            return result;
        }
    } else {
        result = release_sync_manager();
        if (result != ESP_OK) {
            return result;
        }
    }

    for (std::size_t index = 0; index < segment_count; ++index) {
        SegmentPayload& payload = s_segment_payloads[index];
        if (x_active) {
            result = rmt_transmit(s_rmt_channels[0], s_copy_encoder, payload.symbols[0],
                                  payload.symbol_count * sizeof(rmt_symbol_word_t),
                                  &kQueuedTransmitConfig);
            if (result != ESP_OK) {
                return result;
            }
        }
        if (y_active) {
            result = rmt_transmit(s_rmt_channels[1], s_copy_encoder, payload.symbols[1],
                                  payload.symbol_count * sizeof(rmt_symbol_word_t),
                                  &kQueuedTransmitConfig);
            if (result != ESP_OK) {
                return result;
            }
        }
    }

    if (x_active) {
        result = rmt_tx_wait_all_done(s_rmt_channels[0], -1);
        if (result != ESP_OK) {
            stepper_invalidate_position();
            return result;
        }
        stepper_record_executed_steps(machine::Axis::X, emitted_steps[0]);
    }
    if (y_active) {
        result = rmt_tx_wait_all_done(s_rmt_channels[1], -1);
        if (result != ESP_OK) {
            stepper_invalidate_position();
            return result;
        }
        stepper_record_executed_steps(machine::Axis::Y, emitted_steps[1]);
    }
    return result;
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

    std::int32_t remaining_steps = step_count;
    const rmt_symbol_word_t step_symbol = make_step_symbol(true, step_interval_us);
    while (remaining_steps > 0) {
        wait_for_feed_hold_release();
        const std::int32_t chunk_steps = std::min(
            remaining_steps, static_cast<std::int32_t>(kRmtStepMemorySymbols));
        rmt_symbol_word_t symbols[kRmtStepMemorySymbols] = {};
        for (std::int32_t step = 0; step < chunk_steps; ++step) {
            symbols[step] = step_symbol;
        }

        result = rmt_transmit(s_rmt_channels[axis_index], s_copy_encoder, symbols,
                              static_cast<std::size_t>(chunk_steps) * sizeof(rmt_symbol_word_t),
                              &kTransmitConfig);
        if (result != ESP_OK) {
            return result;
        }
        result = wait_for_tx_done(axis, chunk_steps, step_interval_us);
        if (result != ESP_OK) {
            stepper_invalidate_position();
            return result;
        }
        stepper_record_executed_steps(axis, chunk_steps);
        remaining_steps -= chunk_steps;
    }
    return ESP_OK;
}

esp_err_t rmt_stepper_execute_xy(const std::int32_t steps[machine::kAxisCount],
                                 std::uint32_t step_interval_us) {
    if (steps == nullptr || steps[0] <= 0 || steps[1] <= 0 ||
        step_interval_us <= machine::kStepPulseMicroseconds ||
        step_interval_us > kRmtStepMaxIntervalUs) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = rmt_stepper_init();
    if (result != ESP_OK) {
        return result;
    }
    result = ensure_sync_manager();
    if (result != ESP_OK) {
        return result;
    }
    const std::int32_t major_steps = std::max(steps[0], steps[1]);
    const std::int32_t minor_steps = std::min(steps[0], steps[1]);
    const bool x_is_major = steps[0] >= steps[1];
    std::int32_t remaining_major_steps = major_steps;
    std::int64_t error = 0;

    while (remaining_major_steps > 0) {
        wait_for_feed_hold_release();
        const std::int32_t chunk_steps = std::min(
            remaining_major_steps, static_cast<std::int32_t>(kRmtStepMemorySymbols));
        rmt_symbol_word_t x_symbols[kRmtStepMemorySymbols] = {};
        rmt_symbol_word_t y_symbols[kRmtStepMemorySymbols] = {};
        std::int32_t x_emitted_steps = 0;
        std::int32_t y_emitted_steps = 0;

        for (std::int32_t step = 0; step < chunk_steps; ++step) {
            bool x_pulse = x_is_major;
            bool y_pulse = !x_is_major;
            error += minor_steps;
            if (error * 2 >= major_steps) {
                if (x_is_major) {
                    y_pulse = true;
                } else {
                    x_pulse = true;
                }
                error -= major_steps;
            }
            x_symbols[step] = make_step_symbol(x_pulse, step_interval_us);
            y_symbols[step] = make_step_symbol(y_pulse, step_interval_us);
            x_emitted_steps += x_pulse ? 1 : 0;
            y_emitted_steps += y_pulse ? 1 : 0;
        }

        result = rmt_sync_reset(s_sync_manager);
        if (result != ESP_OK) {
            return result;
        }
        result = rmt_transmit(s_rmt_channels[0], s_copy_encoder, x_symbols,
                              static_cast<std::size_t>(chunk_steps) * sizeof(rmt_symbol_word_t),
                              &kTransmitConfig);
        if (result != ESP_OK) {
            return result;
        }
        result = rmt_transmit(s_rmt_channels[1], s_copy_encoder, y_symbols,
                              static_cast<std::size_t>(chunk_steps) * sizeof(rmt_symbol_word_t),
                              &kTransmitConfig);
        if (result != ESP_OK) {
            return result;
        }
        result = wait_for_tx_done(machine::Axis::X, chunk_steps, step_interval_us);
        if (result != ESP_OK) {
            stepper_invalidate_position();
            return result;
        }
        stepper_record_executed_steps(machine::Axis::X, x_emitted_steps);
        result = wait_for_tx_done(machine::Axis::Y, chunk_steps, step_interval_us);
        if (result != ESP_OK) {
            stepper_invalidate_position();
            return result;
        }
        stepper_record_executed_steps(machine::Axis::Y, y_emitted_steps);
        remaining_major_steps -= chunk_steps;
    }
    return ESP_OK;
}

void rmt_stepper_request_feed_hold() {
    s_feed_hold_requested.store(true);
}

void rmt_stepper_resume() {
    s_feed_hold_requested.store(false);
}

esp_err_t rmt_stepper_stop() {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_error = ESP_OK;
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const esp_err_t result = rmt_disable(s_rmt_channels[axis]);
        if (result != ESP_OK && first_error == ESP_OK) {
            first_error = result;
        }
    }
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        const esp_err_t result = rmt_enable(s_rmt_channels[axis]);
        if (result != ESP_OK && first_error == ESP_OK) {
            first_error = result;
        }
    }
    return first_error;
}

}  // namespace ray2::stepper
