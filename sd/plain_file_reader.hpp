#pragma once

#include "file_reader.hpp"

namespace ray2::sd {

class PlainFileReader final : public FileReader {
public:
    bool open(const char* path) override;
    bool read_line(char* buffer, std::size_t capacity) override;
    bool eof() const override;
    bool has_error() const override;
    void close() override;

private:
    void* file_ = nullptr;
    bool eof_ = false;
    bool error_ = false;
};

}  // namespace ray2::sd
