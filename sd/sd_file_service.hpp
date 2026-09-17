#pragma once

#include <cstddef>
#include <cstdint>

#include "file_reader.hpp"

namespace ray2::sd {

enum class FileResult : std::uint8_t {
    Ok,
    InvalidPath,
    NotMounted,
    Busy,
    NotFound,
    NotDirectory,
    IsDirectory,
    AlreadyExists,
    IoError,
};

using ListEntryCallback = bool (*)(const char* path,
                                    std::uintmax_t size,
                                    bool is_directory,
                                    void* user_data);

struct FileReadSession {
    void* file = nullptr;
};

struct FileUploadSession {
    void* file = nullptr;
    char target_path[kFilePathCapacity] = {};
    char temporary_path[kFilePathCapacity] = {};
    char backup_path[kFilePathCapacity] = {};
};

// Logical paths are rooted at the SD volume, for example "/job.gc". They are
// never host filesystem paths and cannot escape the mounted /sd volume.
FileResult normalize_path(const char* source, char* target, std::size_t target_capacity);

FileResult validate_directory(const char* path);
// Directory enumeration takes a short exclusive storage lease. A callback must
// return quickly and must not invoke another sd_file_service operation.
FileResult list_directory(const char* path, ListEntryCallback callback, void* user_data);
bool list_root(ListEntryCallback callback, void* user_data);

FileResult file_read_open(const char* path, FileReadSession& session, std::uintmax_t& size);
FileResult file_read_next(FileReadSession& session, void* buffer, std::size_t capacity,
                          std::size_t& length);
void file_read_close(FileReadSession& session);

// Uploads are first written to a sibling temporary file. Commit replaces the
// target only after the complete body was flushed and closed.
FileResult file_upload_begin(const char* path, FileUploadSession& session);
FileResult file_upload_write(FileUploadSession& session, const void* data, std::size_t length);
FileResult file_upload_commit(FileUploadSession& session);
void file_upload_abort(FileUploadSession& session);

FileResult file_delete(const char* path);
FileResult directory_create(const char* path);

// The SD job owns this lease while its reader is open. HTTP file operations
// use the same lease, so a job cannot race a file write, download or listing.
bool file_service_acquire_job_access();
void file_service_release_job_access();
bool file_service_is_busy();

const char* file_result_text(FileResult result);

}  // namespace ray2::sd
