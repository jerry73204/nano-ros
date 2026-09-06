// nros-cpp: Lightweight non-owning view types
//
// Provides nros::Span<T> and nros::StringView as zero-overhead alternatives
// to std::span (C++20) and std::string_view (C++17). Compatible with GCC 5+,
// Clang 3.5+, and all embedded toolchains.
//
// Written in the freestanding C++14 subset, which is a property of this
// FILE and not the project's minimum: nano-ros declares C++17 (issue 1118 —
// `target_compile_features(nros-cpp-headers INTERFACE cxx_std_17)`), because
// `component_node.hpp` needs `if constexpr`. `just check cpp` still parses
// this header at `-std=c++14 -ffreestanding`, so the subset is enforced and
// not merely claimed.
//
// These types are used by generated borrowed message structs to reference
// variable-length data in the CDR receive buffer without copying.

/**
 * @file span.hpp
 * @ingroup grp_support
 * @brief `nros::Span<T>` and `nros::StringView` — non-owning views.
 */

#ifndef NROS_CPP_SPAN_HPP
#define NROS_CPP_SPAN_HPP

#include <cstddef>
#include <cstdint>
#include <string.h>

namespace nros {

/// Non-owning view over a contiguous sequence of `T` values.
///
/// Same semantics as `std::span<const T>`, without needing C++20 for it.
/// The data pointer is valid only for the lifetime of the source buffer
/// (typically the subscription callback scope).
template <typename T> struct Span {
    /// Pointer to the first element. Borrowed — caller-owned storage.
    const T* ptr;
    /// Number of elements in the view.
    size_t len;

    /// Pointer to the underlying storage.
    constexpr const T* data() const { return ptr; }
    /// Number of elements (alias for `len`).
    constexpr size_t size() const { return len; }
    /// True if the view contains zero elements.
    constexpr bool empty() const { return len == 0; }
    /// Element access; no bounds check.
    constexpr const T& operator[](size_t i) const { return ptr[i]; }
    /// Iterator to the first element.
    constexpr const T* begin() const { return ptr; }
    /// Iterator past the last element.
    constexpr const T* end() const { return ptr + len; }
};

/// Non-owning view over a UTF-8 string (not null-terminated).
///
/// Same semantics as `std::string_view`, without needing C++17's <string_view>.
/// The data pointer is valid only for the lifetime of the source buffer.
struct StringView {
    /// Pointer to the first byte. Not null-terminated.
    const char* ptr;
    /// Number of bytes in the view.
    size_t len;

    /// Pointer to the underlying bytes.
    constexpr const char* data() const { return ptr; }
    /// Number of bytes.
    constexpr size_t size() const { return len; }
    /// True if the view contains zero bytes.
    constexpr bool empty() const { return len == 0; }
    /// Byte access; no bounds check.
    constexpr char operator[](size_t i) const { return ptr[i]; }
    /// Iterator to the first byte.
    constexpr const char* begin() const { return ptr; }
    /// Iterator past the last byte.
    constexpr const char* end() const { return ptr + len; }

    /// True if `cstr` (null-terminated) has the same length and bytes.
    bool equals(const char* cstr) const {
        size_t clen = strlen(cstr);
        return clen == len && memcmp(ptr, cstr, len) == 0;
    }
};

/// Alignment-agnostic view over a little-endian numeric sequence in the CDR
/// receive buffer (RFC-0033 borrowed mode — the C++ analogue of Rust's
/// `nros_core::LeSliceView` and C's `nros_le_slice_view_*`).
///
/// `T` is a fixed-width numeric (`uint16_t`, `float`, …). The raw LE bytes are
/// borrowed zero-copy; `operator[]` decodes one element by value (no `T*` is
/// ever formed into the unaligned buffer), so the buffer base need not be
/// `T`-aligned. The pointer is valid only while the source buffer lives.
template <typename T> struct LeSpan {
    /// Pointer to the first element's little-endian bytes. Borrowed.
    const uint8_t* bytes;
    /// Number of elements.
    size_t count;

    /// Number of elements.
    constexpr size_t size() const { return count; }
    /// True if the view contains zero elements.
    constexpr bool empty() const { return count == 0; }

    /// Decode element `i` (little-endian → host); no bounds check, no alignment
    /// assumption.
    T operator[](size_t i) const {
        const uint8_t* p = bytes + i * sizeof(T);
        unsigned char tmp[sizeof(T)];
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) &&                                    \
    __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        for (size_t b = 0; b < sizeof(T); ++b)
            tmp[b] = p[sizeof(T) - 1 - b];
#else
        for (size_t b = 0; b < sizeof(T); ++b)
            tmp[b] = p[b];
#endif
        T out;
        memcpy(&out, tmp, sizeof(T));
        return out;
    }
};

} // namespace nros

#endif // NROS_CPP_SPAN_HPP
