#pragma once

/// @file cdr.hpp
/// @brief CDR (Common Data Representation) serialization for nano-ros
///
/// Header-only CDR encoder/decoder that is wire-compatible with Rust nano-ros-serdes
/// and ROS 2 DDS implementations.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace nano_ros {

/// CDR little-endian encapsulation header
/// Format: [0x00, 0x01, 0x00, 0x00]
/// - Byte 0: CDR representation identifier (0x00 for CDR)
/// - Byte 1: Options - 0x01 for little-endian
/// - Bytes 2-3: Reserved (0x00, 0x00)
inline constexpr uint8_t CDR_LE_HEADER[4] = {0x00, 0x01, 0x00, 0x00};

/// @brief CDR writer for serialization
///
/// Handles alignment and little-endian encoding for CDR format.
/// Wire-compatible with Rust nano-ros-serdes.
///
/// Example:
/// @code
/// nano_ros::CdrWriter writer;
/// writer.write_encapsulation();
/// writer.write_i32(42);
/// writer.write_string("hello");
/// auto bytes = writer.take_buffer();
/// @endcode
class CdrWriter {
public:
    /// @brief Construct a CDR writer with default buffer size
    CdrWriter() {
        buffer_.reserve(256);
    }

    /// @brief Construct a CDR writer with pre-allocated buffer size
    explicit CdrWriter(size_t initial_capacity) {
        buffer_.reserve(initial_capacity);
    }

    /// @brief Write CDR encapsulation header (little-endian)
    ///
    /// Must be called before writing any data if encapsulation is needed.
    /// Sets the origin for alignment calculations.
    void write_encapsulation() {
        buffer_.insert(buffer_.end(), CDR_LE_HEADER, CDR_LE_HEADER + 4);
        origin_ = 4;
    }

    /// @brief Get current position in buffer
    size_t position() const {
        return buffer_.size();
    }

    /// @brief Get the serialized data
    const uint8_t* data() const {
        return buffer_.data();
    }

    /// @brief Get the size of serialized data
    size_t size() const {
        return buffer_.size();
    }

    /// @brief Get a copy of the buffer
    std::vector<uint8_t> buffer() const {
        return buffer_;
    }

    /// @brief Move the buffer out of the writer
    std::vector<uint8_t> take_buffer() {
        return std::move(buffer_);
    }

    // ========================================================================
    // Primitive write methods
    // ========================================================================

    /// @brief Write a single byte without alignment
    void write_u8(uint8_t value) {
        buffer_.push_back(value);
    }

    /// @brief Write a signed byte without alignment
    void write_i8(int8_t value) {
        write_u8(static_cast<uint8_t>(value));
    }

    /// @brief Write a boolean (serialized as single byte: 0 = false, 1 = true)
    void write_bool(bool value) {
        write_u8(value ? 1 : 0);
    }

    /// @brief Write u16 with 2-byte alignment (little-endian)
    void write_u16(uint16_t value) {
        align(2);
        buffer_.push_back(static_cast<uint8_t>(value & 0xFF));
        buffer_.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }

    /// @brief Write i16 with 2-byte alignment (little-endian)
    void write_i16(int16_t value) {
        write_u16(static_cast<uint16_t>(value));
    }

    /// @brief Write u32 with 4-byte alignment (little-endian)
    void write_u32(uint32_t value) {
        align(4);
        buffer_.push_back(static_cast<uint8_t>(value & 0xFF));
        buffer_.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        buffer_.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        buffer_.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    }

    /// @brief Write i32 with 4-byte alignment (little-endian)
    void write_i32(int32_t value) {
        write_u32(static_cast<uint32_t>(value));
    }

    /// @brief Write u64 with 8-byte alignment (little-endian)
    void write_u64(uint64_t value) {
        align(8);
        for (int i = 0; i < 8; ++i) {
            buffer_.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
        }
    }

    /// @brief Write i64 with 8-byte alignment (little-endian)
    void write_i64(int64_t value) {
        write_u64(static_cast<uint64_t>(value));
    }

    /// @brief Write f32 with 4-byte alignment (little-endian)
    void write_f32(float value) {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        write_u32(bits);
    }

    /// @brief Write f64 with 8-byte alignment (little-endian)
    void write_f64(double value) {
        uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        write_u64(bits);
    }

    /// @brief Write raw bytes without alignment
    void write_bytes(const uint8_t* data, size_t len) {
        buffer_.insert(buffer_.end(), data, data + len);
    }

    // ========================================================================
    // String and sequence methods
    // ========================================================================

    /// @brief Write a CDR string (4-byte length including null + data + null terminator)
    void write_string(const std::string& s) {
        // Length includes null terminator
        auto len = static_cast<uint32_t>(s.size() + 1);
        write_u32(len);
        write_bytes(reinterpret_cast<const uint8_t*>(s.data()), s.size());
        write_u8(0);  // Null terminator
    }

    /// @brief Write a sequence length (used before writing sequence elements)
    void write_sequence_length(uint32_t len) {
        write_u32(len);
    }

    /// @brief Write a sequence of primitives
    template <typename T>
    void write_sequence(const std::vector<T>& vec);

private:
    /// @brief Align to the given boundary (relative to origin)
    void align(size_t alignment) {
        size_t offset = buffer_.size() - origin_;
        size_t padding = (alignment - (offset % alignment)) % alignment;
        for (size_t i = 0; i < padding; ++i) {
            buffer_.push_back(0);
        }
    }

    std::vector<uint8_t> buffer_;
    size_t origin_ = 0;
};

// Template specializations for write_sequence
template <>
inline void CdrWriter::write_sequence(const std::vector<uint8_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    write_bytes(vec.data(), vec.size());
}

template <>
inline void CdrWriter::write_sequence(const std::vector<int8_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    write_bytes(reinterpret_cast<const uint8_t*>(vec.data()), vec.size());
}

template <>
inline void CdrWriter::write_sequence(const std::vector<bool>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (bool v : vec) {
        write_bool(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<uint16_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (uint16_t v : vec) {
        write_u16(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<int16_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (int16_t v : vec) {
        write_i16(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<uint32_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (uint32_t v : vec) {
        write_u32(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<int32_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (int32_t v : vec) {
        write_i32(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<uint64_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (uint64_t v : vec) {
        write_u64(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<int64_t>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (int64_t v : vec) {
        write_i64(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<float>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (float v : vec) {
        write_f32(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<double>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (double v : vec) {
        write_f64(v);
    }
}

template <>
inline void CdrWriter::write_sequence(const std::vector<std::string>& vec) {
    write_sequence_length(static_cast<uint32_t>(vec.size()));
    for (const auto& v : vec) {
        write_string(v);
    }
}

/// @brief CDR reader for deserialization
///
/// Handles alignment and little-endian decoding for CDR format.
/// Wire-compatible with Rust nano-ros-serdes.
///
/// Example:
/// @code
/// nano_ros::CdrReader reader(data.data(), data.size());
/// reader.read_encapsulation();
/// int32_t value = reader.read_i32();
/// std::string name = reader.read_string();
/// @endcode
class CdrReader {
public:
    /// @brief Construct a CDR reader from a buffer
    /// @param data Pointer to the CDR data
    /// @param size Size of the data in bytes
    CdrReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

    /// @brief Construct a CDR reader from a vector
    explicit CdrReader(const std::vector<uint8_t>& buffer)
        : data_(buffer.data()), size_(buffer.size()) {}

    /// @brief Read and validate CDR encapsulation header
    ///
    /// Must be called before reading data if encapsulation was used.
    /// Sets the origin for alignment calculations.
    /// @throws std::runtime_error if header is invalid
    void read_encapsulation() {
        if (remaining() < 4) {
            throw std::runtime_error("CDR: unexpected end of buffer reading header");
        }
        // Check for valid CDR header (we only support little-endian)
        if (data_[pos_] != 0x00 || (data_[pos_ + 1] != 0x00 && data_[pos_ + 1] != 0x01)) {
            throw std::runtime_error("CDR: invalid encapsulation header");
        }
        pos_ = 4;
        origin_ = 4;
    }

    /// @brief Get current position in buffer
    size_t position() const {
        return pos_;
    }

    /// @brief Get remaining bytes
    size_t remaining() const {
        return size_ > pos_ ? size_ - pos_ : 0;
    }

    /// @brief Check if reader is at end of buffer
    bool is_empty() const {
        return remaining() == 0;
    }

    // ========================================================================
    // Primitive read methods
    // ========================================================================

    /// @brief Read a single byte without alignment
    uint8_t read_u8() {
        check_remaining(1);
        return data_[pos_++];
    }

    /// @brief Read a signed byte without alignment
    int8_t read_i8() {
        return static_cast<int8_t>(read_u8());
    }

    /// @brief Read a boolean (deserialized from a single byte)
    bool read_bool() {
        return read_u8() != 0;
    }

    /// @brief Read u16 with 2-byte alignment (little-endian)
    uint16_t read_u16() {
        align(2);
        check_remaining(2);
        uint16_t value =
            static_cast<uint16_t>(data_[pos_]) | (static_cast<uint16_t>(data_[pos_ + 1]) << 8);
        pos_ += 2;
        return value;
    }

    /// @brief Read i16 with 2-byte alignment (little-endian)
    int16_t read_i16() {
        return static_cast<int16_t>(read_u16());
    }

    /// @brief Read u32 with 4-byte alignment (little-endian)
    uint32_t read_u32() {
        align(4);
        check_remaining(4);
        uint32_t value = static_cast<uint32_t>(data_[pos_]) |
                         (static_cast<uint32_t>(data_[pos_ + 1]) << 8) |
                         (static_cast<uint32_t>(data_[pos_ + 2]) << 16) |
                         (static_cast<uint32_t>(data_[pos_ + 3]) << 24);
        pos_ += 4;
        return value;
    }

    /// @brief Read i32 with 4-byte alignment (little-endian)
    int32_t read_i32() {
        return static_cast<int32_t>(read_u32());
    }

    /// @brief Read u64 with 8-byte alignment (little-endian)
    uint64_t read_u64() {
        align(8);
        check_remaining(8);
        uint64_t value = 0;
        for (int i = 0; i < 8; ++i) {
            value |= static_cast<uint64_t>(data_[pos_ + i]) << (i * 8);
        }
        pos_ += 8;
        return value;
    }

    /// @brief Read i64 with 8-byte alignment (little-endian)
    int64_t read_i64() {
        return static_cast<int64_t>(read_u64());
    }

    /// @brief Read f32 with 4-byte alignment (little-endian)
    float read_f32() {
        uint32_t bits = read_u32();
        float value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    /// @brief Read f64 with 8-byte alignment (little-endian)
    double read_f64() {
        uint64_t bits = read_u64();
        double value = 0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    /// @brief Read raw bytes without alignment
    /// @param len Number of bytes to read
    /// @return Vector containing the bytes
    std::vector<uint8_t> read_bytes(size_t len) {
        check_remaining(len);
        std::vector<uint8_t> result(data_ + pos_, data_ + pos_ + len);
        pos_ += len;
        return result;
    }

    // ========================================================================
    // String and sequence methods
    // ========================================================================

    /// @brief Read a CDR string (4-byte length including null + data + null terminator)
    /// @return The string (without null terminator)
    /// @throws std::runtime_error if string is malformed
    std::string read_string() {
        uint32_t len = read_u32();
        if (len == 0) {
            throw std::runtime_error("CDR: invalid string length");
        }
        check_remaining(len);
        // Length includes null terminator, so actual string is len - 1 bytes
        std::string result(reinterpret_cast<const char*>(data_ + pos_), len - 1);
        pos_ += len;
        return result;
    }

    /// @brief Read a sequence length
    uint32_t read_sequence_length() {
        return read_u32();
    }

    /// @brief Read a sequence of primitives
    template <typename T>
    std::vector<T> read_sequence();

private:
    /// @brief Align to the given boundary (relative to origin)
    void align(size_t alignment) {
        size_t offset = pos_ - origin_;
        size_t padding = (alignment - (offset % alignment)) % alignment;
        if (remaining() < padding) {
            throw std::runtime_error("CDR: unexpected end of buffer during alignment");
        }
        pos_ += padding;
    }

    /// @brief Check if there are enough bytes remaining
    void check_remaining(size_t needed) const {
        if (remaining() < needed) {
            throw std::runtime_error("CDR: unexpected end of buffer");
        }
    }

    const uint8_t* data_;
    size_t size_;
    size_t pos_ = 0;
    size_t origin_ = 0;
};

// Template specializations for read_sequence
template <>
inline std::vector<uint8_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    return read_bytes(len);
}

template <>
inline std::vector<int8_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    check_remaining(len);
    std::vector<int8_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_i8();
    }
    return result;
}

template <>
inline std::vector<bool> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<bool> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_bool();
    }
    return result;
}

template <>
inline std::vector<uint16_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<uint16_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_u16();
    }
    return result;
}

template <>
inline std::vector<int16_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<int16_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_i16();
    }
    return result;
}

template <>
inline std::vector<uint32_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<uint32_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_u32();
    }
    return result;
}

template <>
inline std::vector<int32_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<int32_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_i32();
    }
    return result;
}

template <>
inline std::vector<uint64_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<uint64_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_u64();
    }
    return result;
}

template <>
inline std::vector<int64_t> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<int64_t> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_i64();
    }
    return result;
}

template <>
inline std::vector<float> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<float> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_f32();
    }
    return result;
}

template <>
inline std::vector<double> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<double> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_f64();
    }
    return result;
}

template <>
inline std::vector<std::string> CdrReader::read_sequence() {
    uint32_t len = read_sequence_length();
    std::vector<std::string> result(len);
    for (uint32_t i = 0; i < len; ++i) {
        result[i] = read_string();
    }
    return result;
}

}  // namespace nano_ros
