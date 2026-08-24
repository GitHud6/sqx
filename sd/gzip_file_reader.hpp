#pragma once

#include <cstdint>
#include <cstdio>

#include "file_reader.hpp"
#include "miniz.h"

namespace ray2::sd {

class GzipFileReader final : public FileReader {
public:
    bool open(const char* path) override;
    bool read_line(char* buffer, std::size_t capacity) override;
    bool eof() const override;
    bool has_error() const override;
    void close() override;

private:
    bool read_header();
    bool fill_output();
    bool read_byte(unsigned char& value);
    bool read_compressed_byte(unsigned char& value);
    bool finish_stream();

    std::FILE* file_ = nullptr;
    tinfl_decompressor inflater_ = {};
    unsigned char input_[1024] = {};
    unsigned char dictionary_[TINFL_LZ_DICT_SIZE] = {};
    std::size_t input_offset_ = 0;
    std::size_t input_length_ = 0;
    std::size_t dictionary_write_offset_ = 0;
    std::size_t output_offset_ = 0;
    std::size_t output_end_ = 0;
    mz_ulong crc_ = MZ_CRC32_INIT;
    std::uint32_t output_size_ = 0;
    bool initialized_ = false;
    bool stream_finished_ = false;
    bool eof_ = false;
    bool error_ = false;
};

}  // namespace ray2::sd
