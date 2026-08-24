#pragma once

#include "esp_err.h"

namespace ray2::driver {

constexpr int kSdCardDetectPin = 14;
constexpr int kSdCardCmdPin = 11;
constexpr int kSdCardClkPin = 12;
constexpr int kSdCardD0Pin = 13;
constexpr char kSdCardMountPoint[] = "/sd";

// Mounts the RayCube 1-bit SDMMC interface at kSdCardMountPoint.
esp_err_t sdcard_start();

// Unmounts the card if it is currently mounted.
esp_err_t sdcard_stop();

bool sdcard_is_mounted();

}  // namespace ray2::driver
