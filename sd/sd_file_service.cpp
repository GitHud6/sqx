#include "sd_file_service.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_log.h"

#include "file_reader.hpp"
#include "sdcard_driver.hpp"
#include "sd_job_service.hpp"

namespace laserctl::sd {
namespace {

constexpr char kLogTag[] = "sd_file";
std::atomic<bool> s_file_access_claimed = false;

bool path_segment_is_valid(const char* const begin, const std::size_t length) {
    if (length == 0) return false;
    if (length == 1 && begin[0] == '.') return false;
    return !(length == 2 && begin[0] == '.' && begin[1] == '.');
}

bool acquire_file_access() {
    if (!laserctl::driver::sdcard_is_mounted()) {
        ESP_LOGW(kLogTag, "access denied reason=not-mounted");
        return false;
    }
    if (laserctl::sd::sd_job_is_active()) {
        ESP_LOGW(kLogTag, "access denied reason=sd-job-active");
        return false;
    }

    bool expected = false;
    if (!s_file_access_claimed.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                                        std::memory_order_acquire)) {
        ESP_LOGW(kLogTag, "access denied reason=file-access-held");
        return false;
    }
    if (!laserctl::driver::sdcard_is_mounted() || laserctl::sd::sd_job_is_active()) {
        s_file_access_claimed.store(false, std::memory_order_release);
        ESP_LOGW(kLogTag, "access released during claim mounted=%d job=%d",
                 laserctl::driver::sdcard_is_mounted(), laserctl::sd::sd_job_is_active());
        return false;
    }
    ESP_LOGI(kLogTag, "access acquired");
    return true;
}

void release_file_access() {
    const bool was_claimed = s_file_access_claimed.exchange(false, std::memory_order_acq_rel);
    if (was_claimed) ESP_LOGI(kLogTag, "access released");
}

FileResult unavailable_result() {
    if (!laserctl::driver::sdcard_is_mounted()) return FileResult::NotMounted;
    return FileResult::Busy;
}

bool build_full_path(const char* const logical_path, char* const full_path,
                     const std::size_t full_path_capacity) {
    if (logical_path == nullptr || full_path == nullptr || full_path_capacity == 0) return false;
    const int length = std::snprintf(full_path, full_path_capacity, "%s%s",
                                     laserctl::driver::kSdCardMountPoint, logical_path);
    return length > 0 && static_cast<std::size_t>(length) < full_path_capacity;
}

bool build_sibling_path(const char* const source, const char* const suffix, char* const target,
                        const std::size_t target_capacity) {
    if (source == nullptr || suffix == nullptr || target == nullptr) return false;
    const int length = std::snprintf(target, target_capacity, "%s%s", source, suffix);
    return length > 0 && static_cast<std::size_t>(length) < target_capacity;
}

void clear_read_session(FileReadSession& session) {
    session.file = nullptr;
}

void clear_upload_session(FileUploadSession& session) {
    session.file = nullptr;
    session.target_path[0] = '\0';
    session.temporary_path[0] = '\0';
    session.backup_path[0] = '\0';
}

}  // namespace

FileResult normalize_path(const char* const source, char* const target, const std::size_t target_capacity) {
    if (source == nullptr || target == nullptr || target_capacity < 2 || source[0] != '/') {
        return FileResult::InvalidPath;
    }

    std::size_t write = 0;
    std::size_t segment_start = 1;
    target[write++] = '/';
    for (std::size_t read = 1;; ++read) {
        const char value = source[read];
        if (value == '\0' || value == '/') {
            if (!path_segment_is_valid(source + segment_start, read - segment_start)) {
                if (value == '\0' && read == 1) {
                    target[write] = '\0';
                    return FileResult::Ok;
                }
                return FileResult::InvalidPath;
            }
            if (write != 1) {
                if (write + 1 >= target_capacity) return FileResult::InvalidPath;
                target[write++] = '/';
            }
            for (std::size_t index = segment_start; index < read; ++index) {
                const unsigned char character = static_cast<unsigned char>(source[index]);
                if (character < 0x20U || source[index] == '\\' || write + 1 >= target_capacity) {
                    return FileResult::InvalidPath;
                }
                target[write++] = source[index];
            }
            if (value == '\0') {
                target[write] = '\0';
                return FileResult::Ok;
            }
            segment_start = read + 1;
        } else if (static_cast<unsigned char>(value) < 0x20U || value == '\\') {
            return FileResult::InvalidPath;
        }
    }
}

FileResult list_directory(const char* const path, ListEntryCallback callback, void* const user_data) {
    if (callback == nullptr) return FileResult::InvalidPath;
    ESP_LOGI(kLogTag, "list begin path=%s", path == nullptr ? "<null>" : path);

    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok) return normalized;
    if (!acquire_file_access()) return unavailable_result();

    char full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    if (!build_full_path(normalized_path, full_path, sizeof(full_path))) {
        release_file_access();
        return FileResult::InvalidPath;
    }
    DIR* directory = opendir(full_path);
    if (directory == nullptr) {
        struct stat path_stat = {};
        const FileResult result =
            stat(full_path, &path_stat) == 0 ? FileResult::NotDirectory : FileResult::NotFound;
        release_file_access();
        return result;
    }

    FileResult result = FileResult::Ok;
    while (dirent* entry = readdir(directory)) {
        if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) continue;

        char entry_path[kFilePathCapacity] = {};
        const int entry_path_length = std::snprintf(
            entry_path, sizeof(entry_path), "%s%s%s", normalized_path,
            std::strcmp(normalized_path, "/") == 0 ? "" : "/", entry->d_name);
        if (entry_path_length <= 0 || static_cast<std::size_t>(entry_path_length) >= sizeof(entry_path)) {
            result = FileResult::IoError;
            break;
        }

        char entry_full_path[sizeof(full_path)] = {};
        if (!build_full_path(entry_path, entry_full_path, sizeof(entry_full_path))) {
            result = FileResult::IoError;
            break;
        }
        struct stat file_stat = {};
        if (stat(entry_full_path, &file_stat) != 0 ||
            !callback(entry_path, static_cast<std::uintmax_t>(file_stat.st_size),
                      S_ISDIR(file_stat.st_mode), user_data)) {
            result = FileResult::IoError;
            break;
        }
    }
    closedir(directory);
    release_file_access();
    ESP_LOGI(kLogTag, "list end path=%s result=%s", normalized_path, file_result_text(result));
    return result;
}

FileResult validate_directory(const char* const path) {
    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok) return normalized;
    if (!acquire_file_access()) return unavailable_result();

    char full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    if (!build_full_path(normalized_path, full_path, sizeof(full_path))) {
        release_file_access();
        return FileResult::InvalidPath;
    }
    struct stat path_stat = {};
    const FileResult result = stat(full_path, &path_stat) != 0
                                  ? FileResult::NotFound
                                  : (S_ISDIR(path_stat.st_mode) ? FileResult::Ok
                                                                 : FileResult::NotDirectory);
    release_file_access();
    return result;
}

FileResult file_read_open(const char* const path, FileReadSession& session, std::uintmax_t& size) {
    file_read_close(session);
    size = 0;
    ESP_LOGI(kLogTag, "read open path=%s", path == nullptr ? "<null>" : path);
    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok || std::strcmp(normalized_path, "/") == 0) {
        return normalized == FileResult::Ok ? FileResult::IsDirectory : normalized;
    }
    if (!acquire_file_access()) return unavailable_result();

    char full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    struct stat path_stat = {};
    if (!build_full_path(normalized_path, full_path, sizeof(full_path))) {
        release_file_access();
        return FileResult::InvalidPath;
    }
    if (stat(full_path, &path_stat) != 0) {
        release_file_access();
        return FileResult::NotFound;
    }
    if (S_ISDIR(path_stat.st_mode)) {
        release_file_access();
        return FileResult::IsDirectory;
    }
    std::FILE* const file = std::fopen(full_path, "rb");
    if (file == nullptr) {
        release_file_access();
        return FileResult::IoError;
    }
    session.file = file;
    size = static_cast<std::uintmax_t>(path_stat.st_size);
    ESP_LOGI(kLogTag, "read opened path=%s bytes=%llu", normalized_path,
             static_cast<unsigned long long>(size));
    return FileResult::Ok;
}

FileResult file_read_next(FileReadSession& session, void* const buffer, const std::size_t capacity,
                          std::size_t& length) {
    length = 0;
    if (session.file == nullptr || buffer == nullptr || capacity == 0) return FileResult::IoError;
    if (!laserctl::driver::sdcard_is_mounted()) return FileResult::NotMounted;
    length = std::fread(buffer, 1, capacity, static_cast<std::FILE*>(session.file));
    return std::ferror(static_cast<std::FILE*>(session.file)) == 0 ? FileResult::Ok
                                                                    : FileResult::IoError;
}

void file_read_close(FileReadSession& session) {
    if (session.file != nullptr) {
        std::fclose(static_cast<std::FILE*>(session.file));
        clear_read_session(session);
        release_file_access();
        ESP_LOGI(kLogTag, "read closed");
    }
}

FileResult file_upload_begin(const char* const path, FileUploadSession& session) {
    file_upload_abort(session);
    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok || std::strcmp(normalized_path, "/") == 0) {
        return normalized == FileResult::Ok ? FileResult::InvalidPath : normalized;
    }
    if (!build_sibling_path(normalized_path, ".upload-part", session.temporary_path,
                            sizeof(session.temporary_path)) ||
        !build_sibling_path(normalized_path, ".upload-backup", session.backup_path,
                            sizeof(session.backup_path))) {
        clear_upload_session(session);
        return FileResult::InvalidPath;
    }
    std::strcpy(session.target_path, normalized_path);
    if (!acquire_file_access()) {
        clear_upload_session(session);
        return unavailable_result();
    }

    char temporary_full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    if (!build_full_path(session.temporary_path, temporary_full_path, sizeof(temporary_full_path))) {
        clear_upload_session(session);
        release_file_access();
        return FileResult::InvalidPath;
    }
    (void)std::remove(temporary_full_path);
    session.file = std::fopen(temporary_full_path, "wb");
    if (session.file == nullptr) {
        clear_upload_session(session);
        release_file_access();
        return FileResult::IoError;
    }
    ESP_LOGI(kLogTag, "upload opened target=%s temporary=%s", session.target_path,
             session.temporary_path);
    return FileResult::Ok;
}

FileResult file_upload_write(FileUploadSession& session, const void* const data,
                             const std::size_t length) {
    if (session.file == nullptr || data == nullptr || length == 0) return FileResult::IoError;
    if (!laserctl::driver::sdcard_is_mounted()) return FileResult::NotMounted;
    return std::fwrite(data, 1, length, static_cast<std::FILE*>(session.file)) == length
               ? FileResult::Ok
               : FileResult::IoError;
}

FileResult file_upload_commit(FileUploadSession& session) {
    if (session.file == nullptr) return FileResult::IoError;
    ESP_LOGI(kLogTag, "upload commit target=%s", session.target_path);
    std::FILE* const file = static_cast<std::FILE*>(session.file);
    const bool flushed = std::fflush(file) == 0;
    const bool closed = std::fclose(file) == 0;
    session.file = nullptr;
    if (!flushed || !closed || !laserctl::driver::sdcard_is_mounted()) {
        file_upload_abort(session);
        return laserctl::driver::sdcard_is_mounted() ? FileResult::IoError : FileResult::NotMounted;
    }

    char target_full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    char temporary_full_path[sizeof(target_full_path)] = {};
    char backup_full_path[sizeof(target_full_path)] = {};
    if (!build_full_path(session.target_path, target_full_path, sizeof(target_full_path)) ||
        !build_full_path(session.temporary_path, temporary_full_path, sizeof(temporary_full_path)) ||
        !build_full_path(session.backup_path, backup_full_path, sizeof(backup_full_path))) {
        file_upload_abort(session);
        return FileResult::InvalidPath;
    }

    struct stat target_stat = {};
    const bool target_exists = stat(target_full_path, &target_stat) == 0;
    if (target_exists && S_ISDIR(target_stat.st_mode)) {
        file_upload_abort(session);
        return FileResult::IsDirectory;
    }
    if (target_exists) {
        (void)std::remove(backup_full_path);
        if (std::rename(target_full_path, backup_full_path) != 0) {
            file_upload_abort(session);
            return FileResult::IoError;
        }
    }
    if (std::rename(temporary_full_path, target_full_path) != 0) {
        if (target_exists) (void)std::rename(backup_full_path, target_full_path);
        file_upload_abort(session);
        return FileResult::IoError;
    }
    if (target_exists) (void)std::remove(backup_full_path);
    clear_upload_session(session);
    release_file_access();
    ESP_LOGI(kLogTag, "upload committed");
    return FileResult::Ok;
}

void file_upload_abort(FileUploadSession& session) {
    const bool had_session = session.target_path[0] != '\0' || session.file != nullptr;
    if (had_session) ESP_LOGW(kLogTag, "upload abort target=%s", session.target_path);
    if (session.file != nullptr) {
        std::fclose(static_cast<std::FILE*>(session.file));
        session.file = nullptr;
    }
    if (session.temporary_path[0] != '\0') {
        char temporary_full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
        if (build_full_path(session.temporary_path, temporary_full_path, sizeof(temporary_full_path))) {
            (void)std::remove(temporary_full_path);
        }
    }
    const bool held = session.target_path[0] != '\0';
    clear_upload_session(session);
    if (held) release_file_access();
}

FileResult file_delete(const char* const path) {
    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok || std::strcmp(normalized_path, "/") == 0) {
        return normalized == FileResult::Ok ? FileResult::InvalidPath : normalized;
    }
    if (!acquire_file_access()) return unavailable_result();
    char full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    if (!build_full_path(normalized_path, full_path, sizeof(full_path))) {
        release_file_access();
        return FileResult::InvalidPath;
    }
    struct stat path_stat = {};
    FileResult result = FileResult::Ok;
    if (stat(full_path, &path_stat) != 0) {
        result = FileResult::NotFound;
    } else if (std::remove(full_path) != 0) {
        result = FileResult::IoError;
    }
    release_file_access();
    return result;
}

FileResult directory_create(const char* const path) {
    char normalized_path[kFilePathCapacity] = {};
    const FileResult normalized = normalize_path(path, normalized_path, sizeof(normalized_path));
    if (normalized != FileResult::Ok || std::strcmp(normalized_path, "/") == 0) {
        return normalized == FileResult::Ok ? FileResult::AlreadyExists : normalized;
    }
    if (!acquire_file_access()) return unavailable_result();
    char full_path[kFilePathCapacity + sizeof(laserctl::driver::kSdCardMountPoint)] = {};
    if (!build_full_path(normalized_path, full_path, sizeof(full_path))) {
        release_file_access();
        return FileResult::InvalidPath;
    }
    struct stat path_stat = {};
    FileResult result = FileResult::Ok;
    if (stat(full_path, &path_stat) == 0) {
        result = FileResult::AlreadyExists;
    } else if (mkdir(full_path, 0775) != 0) {
        result = FileResult::IoError;
    }
    release_file_access();
    return result;
}

bool list_root(ListEntryCallback callback, void* user_data) {
    return list_directory("/", callback, user_data) == FileResult::Ok;
}

bool file_service_acquire_job_access() {
    bool expected = false;
    const bool acquired = s_file_access_claimed.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel, std::memory_order_acquire);
    ESP_LOGI(kLogTag, "job access %s", acquired ? "acquired" : "denied");
    return acquired;
}

void file_service_release_job_access() {
    ESP_LOGI(kLogTag, "job access release");
    release_file_access();
}

bool file_service_is_busy() {
    return s_file_access_claimed.load(std::memory_order_acquire);
}

const char* file_result_text(const FileResult result) {
    switch (result) {
        case FileResult::Ok: return "ok";
        case FileResult::InvalidPath: return "invalid-path";
        case FileResult::NotMounted: return "sd-not-mounted";
        case FileResult::Busy: return "sd-busy";
        case FileResult::NotFound: return "not-found";
        case FileResult::NotDirectory: return "not-directory";
        case FileResult::IsDirectory: return "is-directory";
        case FileResult::AlreadyExists: return "already-exists";
        case FileResult::IoError: return "io-error";
    }
    return "unknown";
}

}  // namespace laserctl::sd
