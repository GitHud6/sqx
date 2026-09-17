#include "gzip_file_reader.hpp"

#include <cstdint>
#include <cstring>

#include "sdcard_driver.hpp"

namespace ray2::sd {
namespace {

constexpr char kMountPointPrefix[] = "/sd";
constexpr unsigned char kGzipMagic0 = 0x1f;
constexpr unsigned char kGzipMagic1 = 0x8b;
constexpr unsigned char kGzipDeflateMethod = 8;

bool valid_path(const char* path) {
    return path != nullptr && path[0] == '/' && std::strstr(path, "..") == nullptr;
}

bool read_exact(std::FILE* file, unsigned char* data, std::size_t length) {
    return file != nullptr && std::fread(data, 1, length, file) == length;
}

std::uint32_t little_u32(const unsigned char* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

}  // namespace

bool GzipFileReader::open(const char* path) {
    close();
    if (!ray2::driver::sdcard_is_mounted() || !valid_path(path)) {
        error_ = true;
        return false;
    }

    char full_path[kFilePathCapacity + sizeof(kMountPointPrefix)] = {};
    const int length = std::snprintf(full_path, sizeof(full_path), "%s%s", kMountPointPrefix, path);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(full_path)) {
        error_ = true;
        return false;
    }
    file_ = std::fopen(full_path, "rb");
    if (file_ == nullptr) {
        error_ = true;
        return false;
    }
    if (std::fseek(file_, -4, SEEK_END) != 0) {
        close();
        error_ = true;
        return false;
    }
    unsigned char uncompressed_size[4] = {};
    if (!read_exact(file_, uncompressed_size, sizeof(uncompressed_size)) ||
        std::fseek(file_, 0, SEEK_SET) != 0 || !read_header()) {
        close();
        error_ = true;
        return false;
    }

    bytes_consumed_ = 0;
    total_bytes_ = little_u32(uncompressed_size);
    tinfl_init(&inflater_);
    initialized_ = true;
    stream_finished_ = false;
    eof_ = false;
    error_ = false;
    return true;
}

bool GzipFileReader::read_header() {
    unsigned char header[10] = {};
    if (!read_exact(file_, header, sizeof(header)) || header[0] != kGzipMagic0 ||
        header[1] != kGzipMagic1 || header[2] != kGzipDeflateMethod) return false;

    const unsigned char flags = header[3];
    if ((flags & 0xe0U) != 0) return false;
    if ((flags & 0x04U) != 0) {
        unsigned char extra_length[2] = {};
        if (!read_exact(file_, extra_length, sizeof(extra_length))) return false;
        std::size_t length = extra_length[0] | (static_cast<std::size_t>(extra_length[1]) << 8);
        while (length > 0) {
            unsigned char discard[32] = {};
            const std::size_t chunk = length > sizeof(discard) ? sizeof(discard) : length;
            if (!read_exact(file_, discard, chunk)) return false;
            length -= chunk;
        }
    }
    if ((flags & 0x08U) != 0) {
        unsigned char value = 0;
        do {
            if (!read_exact(file_, &value, 1)) return false;
        } while (value != 0);
    }
    if ((flags & 0x10U) != 0) {
        unsigned char value = 0;
        do {
            if (!read_exact(file_, &value, 1)) return false;
        } while (value != 0);
    }
    if ((flags & 0x02U) != 0) {
        unsigned char header_crc[2] = {};
        if (!read_exact(file_, header_crc, sizeof(header_crc))) return false;
    }
    return true;
}

bool GzipFileReader::fill_output() {
    if (stream_finished_ || file_ == nullptr || !initialized_) return false;
    if (input_offset_ == input_length_) {
        input_length_ = std::fread(input_, 1, sizeof(input_), file_);
        input_offset_ = 0;
        if (input_length_ == 0) {
            error_ = true;
            return false;
        }
    }

    const std::size_t output_capacity = TINFL_LZ_DICT_SIZE - dictionary_write_offset_;
    std::size_t input_size = input_length_ - input_offset_;
    std::size_t output_size = output_capacity;
    const bool source_has_more = !std::feof(file_) || input_size > 8;
    const tinfl_status result = tinfl_decompress(
        &inflater_, input_ + input_offset_, &input_size, dictionary_,
        dictionary_ + dictionary_write_offset_, &output_size,
        source_has_more ? TINFL_FLAG_HAS_MORE_INPUT : 0);
    input_offset_ += input_size;
    output_offset_ = dictionary_write_offset_;
    output_end_ = dictionary_write_offset_ + output_size;
    dictionary_write_offset_ = (dictionary_write_offset_ + output_size) % TINFL_LZ_DICT_SIZE;
    if (output_size > 0) {
        crc_ = mz_crc32(crc_, dictionary_ + output_offset_, output_size);
        output_size_ += static_cast<std::uint32_t>(output_size);
    }
    if (result == TINFL_STATUS_DONE) {
        stream_finished_ = true;
    } else if (result < 0 || (result == TINFL_STATUS_NEEDS_MORE_INPUT && input_offset_ == input_length_ &&
                              std::feof(file_))) {
        error_ = true;
        return false;
    }
    return output_size > 0 || stream_finished_;
}

bool GzipFileReader::read_line(char* buffer, std::size_t capacity) {
    if (buffer == nullptr || capacity == 0 || error_ || eof_) return false;
    std::size_t length = 0;
    while (true) {
        unsigned char value = 0;
        if (!read_byte(value)) {
            if (length != 0 && !error_) {
                buffer[length] = '\0';
                return true;
            }
            return false;
        }
        if (value == '\n') {
            buffer[length] = '\0';
            return true;
        }
        if (value == '\r') continue;
        if (length + 1 >= capacity) {
            error_ = true;
            buffer[0] = '\0';
            return false;
        }
        buffer[length++] = static_cast<char>(value);
    }
}

bool GzipFileReader::read_byte(unsigned char& value) {
    while (output_offset_ >= output_end_) {
        output_offset_ = 0;
        output_end_ = 0;
        if (stream_finished_) return finish_stream();
        if (!fill_output()) return false;
    }
    value = dictionary_[output_offset_++];
    ++bytes_consumed_;
    return true;
}

bool GzipFileReader::read_compressed_byte(unsigned char& value) {
    if (input_offset_ < input_length_) {
        value = input_[input_offset_++];
        return true;
    }
    return read_exact(file_, &value, 1);
}

bool GzipFileReader::finish_stream() {
    unsigned char trailer[8] = {};
    for (unsigned char& value : trailer) {
        if (!read_compressed_byte(value)) {
            error_ = true;
            return false;
        }
    }
    if (little_u32(trailer) != static_cast<std::uint32_t>(crc_) || little_u32(trailer + 4) != output_size_) {
        error_ = true;
        return false;
    }
    eof_ = true;
    return false;
}

bool GzipFileReader::eof() const { return eof_; }
bool GzipFileReader::has_error() const { return error_; }

std::uint64_t GzipFileReader::bytes_consumed() const { return bytes_consumed_; }
std::uint64_t GzipFileReader::total_bytes() const { return total_bytes_; }

void GzipFileReader::close() {
    if (file_ != nullptr) std::fclose(file_);
    file_ = nullptr;
    inflater_ = {};
    input_offset_ = 0;
    input_length_ = 0;
    dictionary_write_offset_ = 0;
    output_offset_ = 0;
    output_end_ = 0;
    crc_ = MZ_CRC32_INIT;
    output_size_ = 0;
    bytes_consumed_ = 0;
    total_bytes_ = 0;
    initialized_ = false;
    stream_finished_ = false;
    eof_ = false;
    error_ = false;
}

}  // namespace ray2::sd
