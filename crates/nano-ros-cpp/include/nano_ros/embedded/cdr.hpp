#pragma once

/// @file cdr.hpp
/// @brief CDR serialization for embedded systems
///
/// This header provides CDR (Common Data Representation) serialization
/// that works without exceptions or dynamic allocation. Uses error codes
/// and fixed-size buffers.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <nano_ros/embedded/types.hpp>

namespace nano_ros {

// =============================================================================
// EmbeddedCdrWriter - CDR serialization with fixed buffer
// =============================================================================

/// CDR Writer for embedded systems
///
/// Uses a fixed external buffer, returns error codes instead of throwing.
/// All data is written in little-endian format (CDR_LE).
///
/// Example:
/// @code
/// uint8_t buffer[256];
/// EmbeddedCdrWriter writer(buffer, sizeof(buffer));
/// writer.write_encapsulation();
/// writer.write_i32(42);
/// writer.write_string("hello");
/// if (writer.is_ok()) {
///     // Use writer.data() and writer.size()
/// }
/// @endcode
class EmbeddedCdrWriter {
public:
    /// Initialize with external buffer
    /// @param buffer Pointer to buffer for serialized data
    /// @param capacity Maximum size of buffer
    EmbeddedCdrWriter(uint8_t* buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {}

    // Non-copyable, non-movable (references external buffer)
    EmbeddedCdrWriter(const EmbeddedCdrWriter&) = delete;
    EmbeddedCdrWriter& operator=(const EmbeddedCdrWriter&) = delete;

    /// Check if writer is in error state
    bool has_error() const {
        return error_ != ErrorCode::Ok;
    }

    /// Check if writer is OK
    bool is_ok() const {
        return error_ == ErrorCode::Ok;
    }

    /// Get error code
    ErrorCode error() const {
        return error_;
    }

    /// Get pointer to serialized data
    const uint8_t* data() const {
        return buffer_;
    }

    /// Get size of serialized data
    size_t size() const {
        return pos_;
    }

    /// Get remaining capacity
    size_t remaining() const {
        return capacity_ > pos_ ? capacity_ - pos_ : 0;
    }

    /// Reset writer to beginning
    void reset() {
        pos_ = 0;
        error_ = ErrorCode::Ok;
    }

    // =========================================================================
    // CDR Header
    // =========================================================================

    /// Write CDR encapsulation header (little-endian)
    void write_encapsulation() {
        write_u8(0x00);  // CDR_LE representation
        write_u8(0x01);  // Version
        write_u8(0x00);  // Options
        write_u8(0x00);  // Options
    }

    // =========================================================================
    // Primitive Types
    // =========================================================================

    void write_bool(bool v) {
        uint8_t b = v ? 1 : 0;
        write_raw(&b, 1);
    }

    void write_u8(uint8_t v) {
        write_raw(&v, 1);
    }
    void write_i8(int8_t v) {
        write_raw(&v, 1);
    }

    void write_u16(uint16_t v) {
        align(2);
        write_raw(&v, 2);
    }

    void write_i16(int16_t v) {
        align(2);
        write_raw(&v, 2);
    }

    void write_u32(uint32_t v) {
        align(4);
        write_raw(&v, 4);
    }

    void write_i32(int32_t v) {
        align(4);
        write_raw(&v, 4);
    }

    void write_u64(uint64_t v) {
        align(8);
        write_raw(&v, 8);
    }

    void write_i64(int64_t v) {
        align(8);
        write_raw(&v, 8);
    }

    void write_f32(float v) {
        align(4);
        write_raw(&v, 4);
    }

    void write_f64(double v) {
        align(8);
        write_raw(&v, 8);
    }

    // =========================================================================
    // String Types
    // =========================================================================

    /// Write string (with length prefix and null terminator)
    void write_string(StringView s) {
        auto len = static_cast<uint32_t>(s.size() + 1);  // Include null
        write_u32(len);
        write_raw(s.data(), s.size());
        write_u8(0);  // Null terminator
    }

    /// Write string from C string
    void write_string(const char* s) {
        size_t len = 0;
        while (s[len] != '\0') {
            ++len;
        }
        write_string(StringView(s, len));
    }

    // =========================================================================
    // Array and Sequence Types
    // =========================================================================

    /// Write fixed-size array
    template <typename T, size_t N>
    void write_array(const T (&arr)[N]) {
        for (size_t i = 0; i < N && is_ok(); ++i) {
            write_value(arr[i]);
        }
    }

    /// Write sequence (dynamic array with length prefix)
    template <typename T, size_t N>
    void write_sequence(const Vector<T, N>& vec) {
        write_u32(static_cast<uint32_t>(vec.size()));
        for (size_t i = 0; i < vec.size() && is_ok(); ++i) {
            write_value(vec[i]);
        }
    }

    /// Write raw bytes
    void write_bytes(const uint8_t* data, size_t len) {
        write_u32(static_cast<uint32_t>(len));
        write_raw(data, len);
    }

private:
    /// Align write position to boundary
    void align(size_t alignment) {
        if (has_error()) {
            return;
        }
        size_t padding = (alignment - (pos_ % alignment)) % alignment;
        if (pos_ + padding > capacity_) {
            error_ = ErrorCode::BufferTooSmall;
            return;
        }
        while (padding != 0) {
            buffer_[pos_++] = 0;
            --padding;
        }
    }

    /// Write raw bytes to buffer
    void write_raw(const void* data, size_t len) {
        if (has_error())
            return;
        if (pos_ + len > capacity_) {
            error_ = ErrorCode::BufferTooSmall;
            return;
        }
        std::memcpy(buffer_ + pos_, data, len);
        pos_ += len;
    }

    /// Write a single value (specialized for primitives)
    template <typename T>
    void write_value(const T& v);

    uint8_t* buffer_;
    size_t capacity_;
    size_t pos_{0};
    ErrorCode error_{ErrorCode::Ok};
};

// Specializations for write_value
template <>
inline void EmbeddedCdrWriter::write_value(const bool& v) {
    write_bool(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const uint8_t& v) {
    write_u8(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const int8_t& v) {
    write_i8(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const uint16_t& v) {
    write_u16(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const int16_t& v) {
    write_i16(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const uint32_t& v) {
    write_u32(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const int32_t& v) {
    write_i32(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const uint64_t& v) {
    write_u64(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const int64_t& v) {
    write_i64(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const float& v) {
    write_f32(v);
}
template <>
inline void EmbeddedCdrWriter::write_value(const double& v) {
    write_f64(v);
}

// =============================================================================
// EmbeddedCdrReader - CDR deserialization with fixed buffer
// =============================================================================

/// CDR Reader for embedded systems
///
/// Reads from a fixed buffer, returns error codes instead of throwing.
///
/// Example:
/// @code
/// EmbeddedCdrReader reader(data, size);
/// reader.read_encapsulation();
/// int32_t value = reader.read_i32();
/// String<64> str;
/// reader.read_string(str);
/// if (reader.is_ok()) {
///     // Use deserialized values
/// }
/// @endcode
class EmbeddedCdrReader {
public:
    /// Initialize with data buffer
    /// @param data Pointer to CDR-serialized data
    /// @param size Size of data in bytes
    EmbeddedCdrReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

    // Non-copyable
    EmbeddedCdrReader(const EmbeddedCdrReader&) = delete;
    EmbeddedCdrReader& operator=(const EmbeddedCdrReader&) = delete;

    /// Check if reader is in error state
    bool has_error() const {
        return error_ != ErrorCode::Ok;
    }

    /// Check if reader is OK
    bool is_ok() const {
        return error_ == ErrorCode::Ok;
    }

    /// Get error code
    ErrorCode error() const {
        return error_;
    }

    /// Get remaining bytes
    size_t remaining() const {
        return size_ > pos_ ? size_ - pos_ : 0;
    }

    /// Get current position
    size_t position() const {
        return pos_;
    }

    /// Reset reader to beginning
    void reset() {
        pos_ = 0;
        error_ = ErrorCode::Ok;
    }

    // =========================================================================
    // CDR Header
    // =========================================================================

    /// Read and validate CDR encapsulation header
    void read_encapsulation() {
        read_u8();  // Representation (0x00 = CDR_LE)
        read_u8();  // Version (0x01)
        read_u8();  // Options
        read_u8();  // Options
    }

    // =========================================================================
    // Primitive Types
    // =========================================================================

    bool read_bool() {
        return read_u8() != 0;
    }

    uint8_t read_u8() {
        return read_primitive<uint8_t>(1);
    }
    int8_t read_i8() {
        return read_primitive<int8_t>(1);
    }

    uint16_t read_u16() {
        align(2);
        return read_primitive<uint16_t>(2);
    }

    int16_t read_i16() {
        align(2);
        return read_primitive<int16_t>(2);
    }

    uint32_t read_u32() {
        align(4);
        return read_primitive<uint32_t>(4);
    }

    int32_t read_i32() {
        align(4);
        return read_primitive<int32_t>(4);
    }

    uint64_t read_u64() {
        align(8);
        return read_primitive<uint64_t>(8);
    }

    int64_t read_i64() {
        align(8);
        return read_primitive<int64_t>(8);
    }

    float read_f32() {
        align(4);
        return read_primitive<float>(4);
    }

    double read_f64() {
        align(8);
        return read_primitive<double>(8);
    }

    // =========================================================================
    // String Types
    // =========================================================================

    /// Read string into fixed-size buffer
    template <size_t N>
    void read_string(String<N>& out) {
        uint32_t len = read_u32();
        if (has_error())
            return;

        // Length includes null terminator
        if (len > 0)
            len--;

        if (len > N - 1) {
            error_ = ErrorCode::BufferTooSmall;
            return;
        }

        if (pos_ + len + 1 > size_) {
            error_ = ErrorCode::InvalidArgument;
            return;
        }

        out.assign(reinterpret_cast<const char*>(data_ + pos_), len);
        pos_ += len + 1;  // Include null terminator
    }

    /// Skip string (don't store value)
    void skip_string() {
        uint32_t len = read_u32();
        if (has_error())
            return;
        if (pos_ + len > size_) {
            error_ = ErrorCode::InvalidArgument;
            return;
        }
        pos_ += len;
    }

    // =========================================================================
    // Array and Sequence Types
    // =========================================================================

    /// Read fixed-size array
    template <typename T, size_t N>
    void read_array(T (&arr)[N]) {
        for (size_t i = 0; i < N && is_ok(); ++i) {
            read_value(arr[i]);
        }
    }

    /// Read sequence into fixed-capacity vector
    template <typename T, size_t N>
    void read_sequence(Vector<T, N>& out) {
        uint32_t len = read_u32();
        if (has_error())
            return;

        if (len > N) {
            error_ = ErrorCode::BufferTooSmall;
            return;
        }

        out.clear();
        for (uint32_t i = 0; i < len && is_ok(); ++i) {
            T v;
            read_value(v);
            if (is_ok()) {
                out.push_back(v);
            }
        }
    }

    /// Read raw bytes
    void read_bytes(uint8_t* out, size_t max_len, size_t& actual_len) {
        uint32_t len = read_u32();
        if (has_error())
            return;

        if (len > max_len) {
            error_ = ErrorCode::BufferTooSmall;
            return;
        }

        if (pos_ + len > size_) {
            error_ = ErrorCode::InvalidArgument;
            return;
        }

        std::memcpy(out, data_ + pos_, len);
        pos_ += len;
        actual_len = len;
    }

private:
    /// Align read position to boundary
    void align(size_t alignment) {
        if (has_error())
            return;
        size_t padding = (alignment - (pos_ % alignment)) % alignment;
        if (pos_ + padding > size_) {
            error_ = ErrorCode::InvalidArgument;
            return;
        }
        pos_ += padding;
    }

    /// Read primitive value
    template <typename T>
    T read_primitive(size_t size) {
        if (has_error())
            return T{};
        if (pos_ + size > size_) {
            error_ = ErrorCode::InvalidArgument;
            return T{};
        }
        T v;
        std::memcpy(&v, data_ + pos_, size);
        pos_ += size;
        return v;
    }

    /// Read a single value (specialized for primitives)
    template <typename T>
    void read_value(T& v);

    const uint8_t* data_;
    size_t size_;
    size_t pos_{0};
    ErrorCode error_{ErrorCode::Ok};
};

// Specializations for read_value
template <>
inline void EmbeddedCdrReader::read_value(bool& v) {
    v = read_bool();
}
template <>
inline void EmbeddedCdrReader::read_value(uint8_t& v) {
    v = read_u8();
}
template <>
inline void EmbeddedCdrReader::read_value(int8_t& v) {
    v = read_i8();
}
template <>
inline void EmbeddedCdrReader::read_value(uint16_t& v) {
    v = read_u16();
}
template <>
inline void EmbeddedCdrReader::read_value(int16_t& v) {
    v = read_i16();
}
template <>
inline void EmbeddedCdrReader::read_value(uint32_t& v) {
    v = read_u32();
}
template <>
inline void EmbeddedCdrReader::read_value(int32_t& v) {
    v = read_i32();
}
template <>
inline void EmbeddedCdrReader::read_value(uint64_t& v) {
    v = read_u64();
}
template <>
inline void EmbeddedCdrReader::read_value(int64_t& v) {
    v = read_i64();
}
template <>
inline void EmbeddedCdrReader::read_value(float& v) {
    v = read_f32();
}
template <>
inline void EmbeddedCdrReader::read_value(double& v) {
    v = read_f64();
}

}  // namespace nano_ros
