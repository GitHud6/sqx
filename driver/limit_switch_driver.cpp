#include "limit_switch_driver.hpp"

#include <atomic>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "runtime_affinity.hpp"

#include "machine_profile.hpp"

namespace laserctl::driver {
namespace {

constexpr TickType_t kDebounceDelay = pdMS_TO_TICKS(5);

std::atomic<bool> s_initialized = false;
std::atomic<std::uint32_t> s_triggered_mask = 0;
std::atomic<std::uint32_t> s_debounce_events = 0;
TaskHandle_t s_debounce_task = nullptr;
LimitSwitchIsrCallback s_isr_callback = nullptr;
void* s_isr_callback_user_data = nullptr;
LimitSwitchStateCallback s_state_callback = nullptr;
void* s_state_callback_user_data = nullptr;

std::uint32_t read_triggered_mask() {
    std::uint32_t mask = 0;
    for (std::size_t index = 0; index < machine::kAxisCount; ++index) {
        const auto& axis = machine::machine_axis(static_cast<machine::Axis>(index));
        const int level = gpio_get_level(static_cast<gpio_num_t>(axis.limit_pin));
        const bool triggered = axis.limit_active_low ? level == 0 : level != 0;
        if (triggered) mask |= 1U << index;
    }
    return mask;
}

void IRAM_ATTR limit_switch_isr(void* user_data) {
    const std::size_t axis = reinterpret_cast<std::uintptr_t>(user_data);
    BaseType_t task_woken = pdFALSE;
    if (s_isr_callback != nullptr) {
        s_isr_callback(axis, s_isr_callback_user_data);
    }
    if (s_debounce_task != nullptr) {
        vTaskNotifyGiveFromISR(s_debounce_task, &task_woken);
    }
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void limit_switch_debounce_task(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(kDebounceDelay);
        const std::uint32_t mask = read_triggered_mask();
        s_triggered_mask.store(mask, std::memory_order_release);
        s_debounce_events.fetch_add(1, std::memory_order_relaxed);
        if (s_state_callback != nullptr) {
            const LimitSwitchState state = {
                .triggered = {(mask & 1U) != 0, (mask & 2U) != 0},
                .debounce_events = s_debounce_events.load(std::memory_order_relaxed),
            };
            s_state_callback(state, s_state_callback_user_data);
        }
    }
}

}  // namespace

esp_err_t limit_switch_driver_init() {
    if (s_initialized.load(std::memory_order_acquire)) return ESP_OK;

    for (std::size_t index = 0; index < machine::kAxisCount; ++index) {
        const auto& axis = machine::machine_axis(static_cast<machine::Axis>(index));
        const gpio_config_t config = {
            .pin_bit_mask = 1ULL << axis.limit_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        const esp_err_t result = gpio_config(&config);
        if (result != ESP_OK) return result;
    }

    if (xTaskCreatePinnedToCore(limit_switch_debounce_task, "limit_debounce", 2048, nullptr, 5,
                                &s_debounce_task, runtime::kNonRealtimeCore) != pdPASS) {
        s_debounce_task = nullptr;
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t service_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (service_result != ESP_OK && service_result != ESP_ERR_INVALID_STATE) {
        vTaskDelete(s_debounce_task);
        s_debounce_task = nullptr;
        return service_result;
    }
    for (std::size_t index = 0; index < machine::kAxisCount; ++index) {
        const auto& axis = machine::machine_axis(static_cast<machine::Axis>(index));
        const esp_err_t result = gpio_isr_handler_add(
            static_cast<gpio_num_t>(axis.limit_pin), limit_switch_isr,
            reinterpret_cast<void*>(static_cast<std::uintptr_t>(index)));
        if (result != ESP_OK) return result;
    }

    s_triggered_mask.store(read_triggered_mask(), std::memory_order_release);
    s_initialized.store(true, std::memory_order_release);
    return ESP_OK;
}

bool limit_switch_driver_is_initialized() {
    return s_initialized.load(std::memory_order_acquire);
}

LimitSwitchState limit_switch_driver_state() {
    const std::uint32_t mask = s_triggered_mask.load(std::memory_order_acquire);
    return {
        .triggered = {(mask & 1U) != 0, (mask & 2U) != 0},
        .debounce_events = s_debounce_events.load(std::memory_order_relaxed),
    };
}

void limit_switch_driver_set_isr_callback(const LimitSwitchIsrCallback callback, void* user_data) {
    s_isr_callback = callback;
    s_isr_callback_user_data = user_data;
}

void limit_switch_driver_set_state_callback(const LimitSwitchStateCallback callback, void* user_data) {
    s_state_callback = callback;
    s_state_callback_user_data = user_data;
}

}  // namespace laserctl::driver
