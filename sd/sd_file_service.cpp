#include "sd_file_service.hpp"

#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

#include "file_reader.hpp"
#include "sdcard_driver.hpp"

namespace ray2::sd {

bool list_root(ListEntryCallback callback, void* user_data) {
    if (callback == nullptr || !ray2::driver::sdcard_is_mounted()) {
        return false;
    }

    DIR* directory = opendir(ray2::driver::kSdCardMountPoint);
    if (directory == nullptr) {
        return false;
    }

    bool success = true;
    while (dirent* entry = readdir(directory)) {
        if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        char path[ray2::sd::kFilePathCapacity] = {};
        const int path_length = std::snprintf(path, sizeof(path), "/%s", entry->d_name);
        if (path_length <= 0 || static_cast<std::size_t>(path_length) >= sizeof(path)) {
            success = false;
            break;
        }

        char full_path[sizeof(path) + sizeof(ray2::driver::kSdCardMountPoint)] = {};
        const int full_path_length = std::snprintf(
            full_path, sizeof(full_path), "%s%s", ray2::driver::kSdCardMountPoint, path);
        if (full_path_length <= 0 || static_cast<std::size_t>(full_path_length) >= sizeof(full_path)) {
            success = false;
            break;
        }

        struct stat file_stat = {};
        if (stat(full_path, &file_stat) != 0 ||
            !callback(path, static_cast<std::uintmax_t>(file_stat.st_size),
                      S_ISDIR(file_stat.st_mode), user_data)) {
            success = false;
            break;
        }
    }

    closedir(directory);
    return success;
}

}  // namespace ray2::sd
