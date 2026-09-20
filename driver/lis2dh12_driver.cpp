#include "lis2dh12_driver.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

namespace laserctl::driver {
namespace {

constexpr char kLogTag[] = "lis2dh12";
constexpr i2c_port_t kLis2dh12I2cPort = I2C_NUM_0;
constexpr std::uint32_t kLis2dh12I2cFrequencyHz = 400000;
constexpr std::uint32_t kLis2dh12TimeoutMs = 100;

constexpr std::uint8_t kWhoAmIRegister = 0x0F;
constexpr std::uint8_t kExpectedWhoAmI = 0x33;
constexpr std::uint8_t kCtrlReg1 = 0x20;
constexpr std::uint8_t kCtrlReg3 = 0x22;
constexpr std::uint8_t kCtrlReg4 = 0x23;
constexpr std::uint8_t kOutputXLowRegister = 0x28;

i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_device = nullptr;
bool s_initialized = false;

esp_err_t write_register(const std::uint8_t reg, const std::uint8_t value) {
    const std::uint8_t command[] = {reg, value};
    return i2c_master_transmit(s_device, command, sizeof(command), kLis2dh12TimeoutMs);
}

esp_err_t read_register(const std::uint8_t reg, std::uint8_t& value) {
    return i2c_master_transmit_receive(
        s_device, &reg, sizeof(reg), &value, sizeof(value), kLis2dh12TimeoutMs);
}

void release_resources() {
    if (s_device != nullptr) {
        i2c_master_bus_rm_device(s_device);
        s_device = nullptr;
    }
    if (s_bus != nullptr) {
        i2c_del_master_bus(s_bus);
        s_bus = nullptr;
    }
}

}  // namespace

bool lis2dh12_read_raw(Lis2dh12RawSample& sample) {
    if (!s_initialized || s_device == nullptr) return false;

    std::uint8_t data[6] = {};
    const std::uint8_t start_register = kOutputXLowRegister | 0x80U;
    if (i2c_master_transmit_receive(s_device, &start_register, sizeof(start_register), data,
                                     sizeof(data), kLis2dh12TimeoutMs) != ESP_OK) {
        return false;
    }

    const auto decode = [](const std::uint8_t low, const std::uint8_t high) {
        return static_cast<std::int16_t>(static_cast<std::int16_t>(
            (static_cast<std::uint16_t>(high) << 8U) | low) >> 4);
    };
    sample.raw_x = decode(data[0], data[1]);
    sample.raw_y = decode(data[2], data[3]);
    sample.raw_z = decode(data[4], data[5]);
    return true;
}

esp_err_t lis2dh12_init() {
    if (s_initialized) return ESP_OK;

    const gpio_config_t int_config = {
        .pin_bit_mask = 1ULL << kLis2dh12Int1Pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t result = gpio_config(&int_config);
    if (result != ESP_OK) return result;

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = kLis2dh12I2cPort,
        .sda_io_num = static_cast<gpio_num_t>(kLis2dh12SdaPin),
        .scl_io_num = static_cast<gpio_num_t>(kLis2dh12SclPin),
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    result = i2c_new_master_bus(&bus_config, &s_bus);
    if (result != ESP_OK) return result;

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = kLis2dh12I2cAddress,
        .scl_speed_hz = kLis2dh12I2cFrequencyHz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };
    result = i2c_master_bus_add_device(s_bus, &device_config, &s_device);
    if (result != ESP_OK) {
        release_resources();
        return result;
    }

    std::uint8_t who_am_i = 0;
    result = read_register(kWhoAmIRegister, who_am_i);
    if (result != ESP_OK) {
        release_resources();
        return result;
    }
    if (who_am_i != kExpectedWhoAmI) {
        ESP_LOGE(kLogTag, "unexpected WHO_AM_I: 0x%02X", who_am_i);
        release_resources();
        return ESP_ERR_NOT_FOUND;
    }

    // 400 Hz, high-resolution mode, all axes enabled.
    result = write_register(kCtrlReg1, 0x77);
    if (result == ESP_OK) {
        // Route XYZ data-ready to INT1. The current sampling task is timed;
        // the pin remains available for a later interrupt-driven mode.
        result = write_register(kCtrlReg3, 0x10);
    }
    if (result == ESP_OK) {
        // BDU enabled, +/-2 g full scale, high-resolution enabled.
        result = write_register(kCtrlReg4, 0x88);
    }
    if (result != ESP_OK) {
        release_resources();
        return result;
    }

    s_initialized = true;
    ESP_LOGI(kLogTag, "initialized: addr=0x%02X, sda=%d, scl=%d, int1=%d",
             kLis2dh12I2cAddress, kLis2dh12SdaPin, kLis2dh12SclPin, kLis2dh12Int1Pin);
    return ESP_OK;
}

bool lis2dh12_is_initialized() {
    return s_initialized;
}

}  // namespace laserctl::driver
