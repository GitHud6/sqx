#include "sdcard_driver.hpp"

#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace ray2::driver {
namespace {

sdmmc_card_t* s_card = nullptr;

}  // namespace

esp_err_t sdcard_start() {
    if (s_card != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.slot = SDMMC_HOST_SLOT_1;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = static_cast<gpio_num_t>(kSdCardClkPin);
    slot_config.cmd = static_cast<gpio_num_t>(kSdCardCmdPin);
    slot_config.d0 = static_cast<gpio_num_t>(kSdCardD0Pin);
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 0,
        .disk_status_check_enable = false,
    };

    return esp_vfs_fat_sdmmc_mount(
        kSdCardMountPoint, &host, &slot_config, &mount_config, &s_card);
}

esp_err_t sdcard_stop() {
    if (s_card == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t result = esp_vfs_fat_sdcard_unmount(kSdCardMountPoint, s_card);
    if (result == ESP_OK) {
        s_card = nullptr;
    }
    return result;
}

bool sdcard_is_mounted() {
    return s_card != nullptr;
}

}  // namespace ray2::driver
