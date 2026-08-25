#include "rmt_step_pulse_driver.hpp"

#include <atomic>
#include <cstdint>

#include "esp_rom_gpio.h"
#include "hal/rmt_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"

#include "raycube_machine.hpp"

namespace ray2::driver {
namespace {

constexpr std::uint32_t kRmtClockDivider = 80;
constexpr std::uint32_t kRmtPulseSymbol = 3U | (1U << 15) | (1U << 16);

std::atomic<bool> s_initialized = false;

void configure_axis(const std::size_t axis_index) {
    const auto& axis = machine::raycube_axis(static_cast<machine::Axis>(axis_index));
    rmt_ll_tx_stop(&RMT, axis_index);
    rmt_ll_tx_set_channel_clock_div(&RMT, axis_index, kRmtClockDivider);
    rmt_ll_tx_set_mem_blocks(&RMT, axis_index, 1);
    rmt_ll_tx_enable_wrap(&RMT, axis_index, false);
    rmt_ll_tx_enable_loop(&RMT, axis_index, false);
    rmt_ll_tx_fix_idle_level(&RMT, axis_index, 0, true);
    rmt_ll_tx_enable_carrier_modulation(&RMT, axis_index, false);
    rmt_ll_tx_reset_pointer(&RMT, axis_index);
    RMT.chndata[axis_index].chndata = kRmtPulseSymbol;
    RMT.chndata[axis_index].chndata = 0;
    esp_rom_gpio_connect_out_signal(axis.step_pin, RMT_SIG_OUT0_IDX + axis_index, false, false);
}

}  // namespace

esp_err_t rmt_step_pulse_driver_init() {
    if (s_initialized.load()) {
        return ESP_OK;
    }

    rmt_ll_enable_group_clock(&RMT, true);
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        configure_axis(axis);
    }
    s_initialized.store(true);
    return ESP_OK;
}

void rmt_step_pulse_driver_stop() {
    if (!s_initialized.load()) {
        return;
    }

    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        rmt_ll_tx_stop(&RMT, axis);
        rmt_ll_tx_reset_pointer(&RMT, axis);
    }
}

bool rmt_step_pulse_driver_is_initialized() {
    return s_initialized.load();
}

void IRAM_ATTR rmt_step_pulse_driver_trigger(const std::size_t axis_index) {
    rmt_ll_tx_reset_pointer(&RMT, axis_index);
    rmt_ll_tx_start(&RMT, axis_index);
}

}  // namespace ray2::driver
