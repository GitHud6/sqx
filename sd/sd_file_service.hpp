#pragma once

#include <cstddef>
#include <cstdint>

namespace ray2::sd {

using ListEntryCallback = bool (*)(const char* path,
                                    std::uintmax_t size,
                                    bool is_directory,
                                    void* user_data);

bool list_root(ListEntryCallback callback, void* user_data);

}  // namespace ray2::sd
