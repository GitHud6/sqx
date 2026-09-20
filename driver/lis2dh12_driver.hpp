#pragma once

#include <cstdint>

#include "esp_err.h"

namespace laserctl::driver {

constexpr int kLis2dh12SdaPin = 38;
constexpr int kLis2dh12SclPin = 39;
constexpr int kLis2dh12Int1Pin = 40;
constexpr std::uint8_t kLis2dh12I2cAddress = 0x19;

struct Lis2dh12RawSample {
    std::int16_t raw_x = 0;
    std::int16_t raw_y = 0;
    std::int16_t raw_z = 0;
};

esp_err_t lis2dh12_init();
bool lis2dh12_is_initialized();
bool lis2dh12_read_raw(Lis2dh12RawSample& sample);

}  // namespace laserctl::driver
