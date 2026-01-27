#pragma once

/// @file types.hpp
/// @brief Platform-abstracted types for embedded and desktop builds
///
/// This header provides type aliases that work across platforms:
/// - Desktop (NANO_ROS_EMBEDDED=0): Uses std::string, std::vector, etc.
/// - Embedded with ETL (NANO_ROS_USE_ETL=1): Uses etl::string, etl::vector, etc.
/// - Embedded freestanding: Uses minimal custom implementations

#include <cstddef>
#include <cstdint>

// =============================================================================
// Platform Detection and Includes
// =============================================================================

#if defined(NANO_ROS_USE_ETL)
// ETL-based embedded build
#include <etl/delegate.h>
#include <etl/optional.h>
#include <etl/span.h>
#include <etl/string.h>
#include <etl/string_view.h>
#include <etl/vector.h>

#elif defined(NANO_ROS_EMBEDDED)
// Freestanding embedded build - minimal implementations below

#else
// Desktop build with full std library
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#endif

namespace nano_ros {

// =============================================================================
// Error Codes (used instead of exceptions in embedded mode)
// =============================================================================

/// Error codes for embedded builds (replaces exceptions)
enum class ErrorCode : int32_t {
    Ok = 0,
    InvalidArgument = -1,
    OutOfMemory = -2,
    Timeout = -3,
    NotFound = -4,
    BufferTooSmall = -5,
    NotInitialized = -6,
    AlreadyExists = -7,
    TransportError = -8,
    SerializationError = -9,
};

/// Check if error code indicates success
inline bool is_ok(ErrorCode err) {
    return err == ErrorCode::Ok;
}

// =============================================================================
// Configuration Constants
// =============================================================================

/// Default buffer and container sizes for embedded builds
struct EmbeddedConfig {
    static constexpr size_t MAX_NODE_NAME = 64;
    static constexpr size_t MAX_TOPIC_NAME = 128;
    static constexpr size_t MAX_NAMESPACE = 64;
    static constexpr size_t MAX_STRING = 256;
    static constexpr size_t MAX_PUBLISHERS = 8;
    static constexpr size_t MAX_SUBSCRIBERS = 8;
    static constexpr size_t MAX_SERVICES = 4;
    static constexpr size_t DEFAULT_BUFFER_SIZE = 512;
    static constexpr size_t MAX_SEQUENCE_LENGTH = 64;
};

// =============================================================================
// Type Definitions - ETL Build
// =============================================================================

#if defined(NANO_ROS_USE_ETL)

/// String view type
using StringView = etl::string_view;

/// Fixed-capacity string
template <size_t N = EmbeddedConfig::MAX_STRING>
using String = etl::string<N>;

/// Fixed-capacity vector
template <typename T, size_t N = EmbeddedConfig::MAX_SEQUENCE_LENGTH>
using Vector = etl::vector<T, N>;

/// Span (view into contiguous memory)
template <typename T>
using Span = etl::span<T>;

/// Optional value
template <typename T>
using Optional = etl::optional<T>;

/// Callback/delegate type
template <typename Signature>
using Function = etl::delegate<Signature>;

// =============================================================================
// Type Definitions - Freestanding Embedded Build
// =============================================================================

#elif defined(NANO_ROS_EMBEDDED)

/// Minimal string view implementation
class StringView {
public:
    constexpr StringView() : data_(nullptr), size_(0) {}
    constexpr StringView(const char* str, size_t len) : data_(str), size_(len) {}
    StringView(const char* str);  // Implemented in embedded.cpp

    constexpr const char* data() const {
        return data_;
    }
    constexpr size_t size() const {
        return size_;
    }
    constexpr size_t length() const {
        return size_;
    }
    constexpr bool empty() const {
        return size_ == 0;
    }

    constexpr char operator[](size_t i) const {
        return data_[i];
    }

    constexpr const char* begin() const {
        return data_;
    }
    constexpr const char* end() const {
        return data_ + size_;
    }

private:
    const char* data_;
    size_t size_;
};

/// Fixed-capacity string (stack allocated)
template <size_t N = EmbeddedConfig::MAX_STRING>
class String {
public:
    String() : size_(0) {
        data_[0] = '\0';
    }

    String(const char* str) : size_(0) {
        while (str[size_] && size_ < N - 1) {
            data_[size_] = str[size_];
            ++size_;
        }
        data_[size_] = '\0';
    }

    String(const char* str, size_t len) : size_(len < N ? len : N - 1) {
        for (size_t i = 0; i < size_; ++i) {
            data_[i] = str[i];
        }
        data_[size_] = '\0';
    }

    void assign(const char* str, size_t len) {
        size_ = len < N ? len : N - 1;
        for (size_t i = 0; i < size_; ++i) {
            data_[i] = str[i];
        }
        data_[size_] = '\0';
    }

    void clear() {
        size_ = 0;
        data_[0] = '\0';
    }

    const char* c_str() const {
        return data_;
    }
    const char* data() const {
        return data_;
    }
    size_t size() const {
        return size_;
    }
    size_t length() const {
        return size_;
    }
    size_t capacity() const {
        return N - 1;
    }
    bool empty() const {
        return size_ == 0;
    }

    char& operator[](size_t i) {
        return data_[i];
    }
    char operator[](size_t i) const {
        return data_[i];
    }

    operator StringView() const {
        return StringView(data_, size_);
    }

private:
    char data_[N];
    size_t size_;
};

/// Fixed-capacity vector (stack allocated)
template <typename T, size_t N = EmbeddedConfig::MAX_SEQUENCE_LENGTH>
class Vector {
public:
    Vector() : size_(0) {}

    void push_back(const T& value) {
        if (size_ < N) {
            data_[size_++] = value;
        }
    }

    void pop_back() {
        if (size_ > 0) {
            --size_;
        }
    }

    void clear() {
        size_ = 0;
    }

    T& operator[](size_t i) {
        return data_[i];
    }
    const T& operator[](size_t i) const {
        return data_[i];
    }

    T* data() {
        return data_;
    }
    const T* data() const {
        return data_;
    }

    size_t size() const {
        return size_;
    }
    size_t capacity() const {
        return N;
    }
    bool empty() const {
        return size_ == 0;
    }
    bool full() const {
        return size_ >= N;
    }

    T* begin() {
        return data_;
    }
    T* end() {
        return data_ + size_;
    }
    const T* begin() const {
        return data_;
    }
    const T* end() const {
        return data_ + size_;
    }

private:
    T data_[N];
    size_t size_;
};

/// Span (non-owning view into contiguous memory)
template <typename T>
class Span {
public:
    constexpr Span() : data_(nullptr), size_(0) {}
    constexpr Span(T* data, size_t size) : data_(data), size_(size) {}

    template <size_t N>
    constexpr Span(T (&arr)[N]) : data_(arr), size_(N) {}

    T* data() const {
        return data_;
    }
    size_t size() const {
        return size_;
    }
    bool empty() const {
        return size_ == 0;
    }

    T& operator[](size_t i) const {
        return data_[i];
    }

    T* begin() const {
        return data_;
    }
    T* end() const {
        return data_ + size_;
    }

private:
    T* data_;
    size_t size_;
};

/// Optional value (simplified)
template <typename T>
class Optional {
public:
    Optional() : has_value_(false) {}
    Optional(const T& value) : has_value_(true) {
        new (&storage_) T(value);
    }

    ~Optional() {
        if (has_value_) {
            reinterpret_cast<T*>(&storage_)->~T();
        }
    }

    Optional(const Optional& other) : has_value_(other.has_value_) {
        if (has_value_) {
            new (&storage_) T(*other);
        }
    }

    Optional& operator=(const Optional& other) {
        if (this != &other) {
            reset();
            if (other.has_value_) {
                new (&storage_) T(*other);
                has_value_ = true;
            }
        }
        return *this;
    }

    bool has_value() const {
        return has_value_;
    }
    explicit operator bool() const {
        return has_value_;
    }

    T& operator*() {
        return *reinterpret_cast<T*>(&storage_);
    }
    const T& operator*() const {
        return *reinterpret_cast<const T*>(&storage_);
    }

    T* operator->() {
        return reinterpret_cast<T*>(&storage_);
    }
    const T* operator->() const {
        return reinterpret_cast<const T*>(&storage_);
    }

    T& value() {
        return **this;
    }
    const T& value() const {
        return **this;
    }

    void reset() {
        if (has_value_) {
            reinterpret_cast<T*>(&storage_)->~T();
            has_value_ = false;
        }
    }

private:
    alignas(T) char storage_[sizeof(T)];
    bool has_value_;
};

/// Function pointer type (no std::function in freestanding)
template <typename Signature>
using Function = Signature*;

// =============================================================================
// Type Definitions - Desktop Build
// =============================================================================

#else  // Desktop with std

/// String view type
using StringView = std::string_view;

/// Dynamic string
template <size_t N = 0>
using String = std::string;

/// Dynamic vector
template <typename T, size_t N = 0>
using Vector = std::vector<T>;

/// Span (using simple implementation for C++17 compatibility)
template <typename T>
class Span {
public:
    constexpr Span() : data_(nullptr), size_(0) {}
    constexpr Span(T* data, size_t size) : data_(data), size_(size) {}

    template <size_t N>
    constexpr Span(T (&arr)[N]) : data_(arr), size_(N) {}

    T* data() const {
        return data_;
    }
    size_t size() const {
        return size_;
    }
    bool empty() const {
        return size_ == 0;
    }

    T& operator[](size_t i) const {
        return data_[i];
    }

    T* begin() const {
        return data_;
    }
    T* end() const {
        return data_ + size_;
    }

private:
    T* data_;
    size_t size_;
};

/// Optional
template <typename T>
using Optional = std::optional<T>;

/// Callable wrapper
template <typename Signature>
using Function = std::function<Signature>;

#endif  // Platform selection

// =============================================================================
// Smart Pointer Alternatives for Embedded
// =============================================================================

#if defined(NANO_ROS_EMBEDDED) && !defined(NANO_ROS_USE_ETL)

/// Non-owning pointer wrapper (embedded alternative to shared_ptr)
/// In embedded, memory is typically statically allocated
template <typename T>
class Ptr {
public:
    Ptr() : ptr_(nullptr) {}
    explicit Ptr(T* ptr) : ptr_(ptr) {}

    T* get() const {
        return ptr_;
    }
    T* operator->() const {
        return ptr_;
    }
    T& operator*() const {
        return *ptr_;
    }
    explicit operator bool() const {
        return ptr_ != nullptr;
    }

    void reset(T* ptr = nullptr) {
        ptr_ = ptr;
    }

private:
    T* ptr_;
};

#else

/// Use std::shared_ptr on desktop
template <typename T>
using Ptr = std::shared_ptr<T>;

#endif

// =============================================================================
// Result Type for Error Handling
// =============================================================================

/// Result type that holds either a value or an error code
template <typename T>
class Result {
public:
    /// Create success result
    static Result ok(const T& value) {
        Result r;
        r.value_ = value;
        r.error_ = ErrorCode::Ok;
        return r;
    }

    /// Create error result
    static Result err(ErrorCode error) {
        Result r;
        r.error_ = error;
        return r;
    }

    bool is_ok() const {
        return error_ == ErrorCode::Ok;
    }
    bool is_err() const {
        return error_ != ErrorCode::Ok;
    }

    ErrorCode error() const {
        return error_;
    }

    T& value() {
        return value_;
    }
    const T& value() const {
        return value_;
    }

    T value_or(const T& default_value) const {
        return is_ok() ? value_ : default_value;
    }

private:
    T value_{};
    ErrorCode error_{ErrorCode::Ok};
};

/// Specialization for void result
template <>
class Result<void> {
public:
    static Result ok() {
        Result r;
        r.error_ = ErrorCode::Ok;
        return r;
    }

    static Result err(ErrorCode error) {
        Result r;
        r.error_ = error;
        return r;
    }

    bool is_ok() const {
        return error_ == ErrorCode::Ok;
    }
    bool is_err() const {
        return error_ != ErrorCode::Ok;
    }

    ErrorCode error() const {
        return error_;
    }

private:
    ErrorCode error_{ErrorCode::Ok};
};

}  // namespace nano_ros
