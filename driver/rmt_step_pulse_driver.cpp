#include "rmt_step_pulse_driver.hpp"

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_private/periph_ctrl.h"
#include "hal/rmt_ll.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"
#include "soc/soc_caps.h"

#include "raycube_machine.hpp"

namespace ray2::driver {
namespace {

// The RMT memory is a separate APB-mapped region. Keep this small local view
// instead of pulling in the deprecated RMT driver types or API.
struct RmtMemoryChannel {
    volatile std::uint32_t data[SOC_RMT_MEM_WORDS_PER_CHANNEL];
};

struct RmtMemory {
    RmtMemoryChannel channel[SOC_RMT_CHANNELS_PER_GROUP];
};

extern "C" RmtMemory RMTMEM;

// Match ray2-main: 80 MHz / 20 yields a 4 MHz RMT clock. The first
// 0.25 us holds the idle level, followed by the 3 us active STEP pulse.
constexpr std::uint32_t kRmtClockDivider = 20;
constexpr std::uint32_t kRmtPulseSymbol = 1U | (12U << 16U) | (1U << 31U);

std::atomic<bool> s_initialized = false;
std::array<std::atomic<std::uint64_t>, machine::kAxisCount> s_trigger_count = {};
std::array<std::atomic<std::uint64_t>, machine::kAxisCount> s_active_retrigger_count = {};
std::array<std::atomic<std::uint32_t>, machine::kAxisCount> s_last_status_word = {};
std::atomic<std::uint64_t> s_invalid_trigger_count = 0;

void configure_axis(const std::size_t axis_index) {
    const auto& axis = machine::raycube_axis(static_cast<machine::Axis>(axis_index));
    rmt_ll_tx_stop(&RMT, axis_index);
    rmt_ll_tx_set_channel_clock_div(&RMT, axis_index, kRmtClockDivider);
    rmt_ll_tx_set_mem_blocks(&RMT, axis_index, 2);
    // Legacy rmt_config() always enables TX ping-pong access. This is also
    // required when the channel owns two memory blocks.
    rmt_ll_tx_enable_wrap(&RMT, axis_index, true);
    rmt_ll_tx_enable_loop(&RMT, axis_index, false);
    rmt_ll_tx_fix_idle_level(&RMT, axis_index, 0, true);
    rmt_ll_tx_enable_carrier_modulation(&RMT, axis_index, false);
    rmt_ll_tx_reset_pointer(&RMT, axis_index);
    // Non-FIFO mode exposes the channel RAM through RMTMEM. chndata is the
    // APB FIFO register and does not populate the direct memory used here.
    RMTMEM.channel[axis_index].data[0] = kRmtPulseSymbol;
    RMTMEM.channel[axis_index].data[1] = 0;
    gpio_set_direction(static_cast<gpio_num_t>(axis.step_pin), GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(axis.step_pin, RMT_SIG_OUT0_IDX + axis_index, false, false);
}

}  // namespace

esp_err_t rmt_step_pulse_driver_init() {
    if (s_initialized.load()) {
        return ESP_OK;
    }

    // Reproduce the module-level setup performed by legacy rmt_config().
    // Channel registers alone are insufficient: the RMT RAM must be visible
    // through APB and the group must be clocked from APB before its symbols
    // can generate STEP output.
    PERIPH_RCC_ATOMIC() {
        rmt_ll_enable_bus_clock(0, true);
        rmt_ll_reset_register(0);
    }
    rmt_ll_mem_power_by_pmu(&RMT);
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);
    rmt_ll_set_group_clock_src(&RMT, 0, RMT_CLK_SRC_APB, 1, 0, 0);
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

void rmt_step_pulse_driver_trigger(const std::size_t axis_index) {
    if (axis_index >= machine::kAxisCount || !s_initialized.load(std::memory_order_acquire)) {
        s_invalid_trigger_count.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const std::uint32_t status_word = RMT.chnstatus[axis_index].val;
    const std::uint32_t fsm_state = (status_word >> 22U) & 0x7U;
    s_last_status_word[axis_index].store(status_word, std::memory_order_relaxed);
    s_trigger_count[axis_index].fetch_add(1, std::memory_order_relaxed);
    if (fsm_state != 0) {
        s_active_retrigger_count[axis_index].fetch_add(1, std::memory_order_relaxed);
    }
    rmt_ll_tx_reset_pointer(&RMT, axis_index);
    rmt_ll_tx_start(&RMT, axis_index);
}

RmtStepPulseDiagnostics rmt_step_pulse_driver_diagnostics() {
    RmtStepPulseDiagnostics diagnostics = {};
    for (std::size_t axis = 0; axis < machine::kAxisCount; ++axis) {
        diagnostics.trigger_count[axis] = s_trigger_count[axis].load(std::memory_order_relaxed);
        diagnostics.active_retrigger_count[axis] =
            s_active_retrigger_count[axis].load(std::memory_order_relaxed);
        diagnostics.last_status_word[axis] = s_last_status_word[axis].load(std::memory_order_relaxed);
    }
    diagnostics.invalid_trigger_count = s_invalid_trigger_count.load(std::memory_order_relaxed);
    return diagnostics;
}

}  // namespace ray2::driver
