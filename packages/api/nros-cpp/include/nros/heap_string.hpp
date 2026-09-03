// nros-cpp: HeapString — heap-backed (alloc) growable string container
// Freestanding C++14 — no exceptions, no STL required
//
// Memory layout is repr(C) compatible: identical to
// `struct { char* data; size_t size; size_t capacity; }` (rclc's
// `rosidl_runtime_c__String` shape). Used by codegen-generated message structs
// for `mode = "heap"` string fields (RFC-0033) — the bridgeable analog of
// rclcpp's `std::string` (whose layout is not repr(C), so it cannot cross the
// Rust↔C++ FFI directly).
//
// `data` is kept NUL-terminated, `size` is the string length, and `capacity`
// includes the NUL (`size + 1`) — matching `rosidl_runtime_c__String`. Allocation
// goes through the C-ABI nros_platform_malloc/free so the SAME allocator spans
// both FFI sides (the Rust deserializer allocates; this destructor frees).

/**
 * @file heap_string.hpp
 * @ingroup grp_support
 * @brief `nros::HeapString` — heap-backed growable string container.
 */

#ifndef NROS_CPP_HEAP_STRING_HPP
#define NROS_CPP_HEAP_STRING_HPP

#include <cstddef>
#include <string.h> // strcmp/strlen — `<cstring>` isn't in Zephyr's minimal libcpp

#include <nros/platform.h>

// phase-417 W1.c — `std::string` interop, gated exactly as `fixed_string.hpp`
// gates it (`__has_include`, not `__STDC_HOSTED__` — issue 0112).
#if defined(NROS_CPP_STD)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#elif defined(__has_include)
#if __has_include(<string>)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#endif
#endif

namespace nros {

/// Heap-backed string container (`mode = "heap"`, RFC-0033).
///
/// Layout `{ char* data; size_t size; size_t capacity; }` — C-ABI compatible
/// with the runtime and the Rust FFI mirror. Owns its buffer (freed in the
/// destructor via `nros_platform_free`); non-copyable, movable. `data` is
/// NUL-terminated when non-null.
struct HeapString {
    char* data;
    size_t size;
    size_t capacity;

    HeapString() : data(nullptr), size(0), capacity(0) {}
    ~HeapString() { nros_platform_free(data); }

    HeapString(const HeapString&) = delete;
    HeapString& operator=(const HeapString&) = delete;
    HeapString(HeapString&& o) noexcept : data(o.data), size(o.size), capacity(o.capacity) {
        o.data = nullptr;
        o.size = 0;
        o.capacity = 0;
    }
    HeapString& operator=(HeapString&& o) noexcept {
        if (this != &o) {
            nros_platform_free(data);
            data = o.data;
            size = o.size;
            capacity = o.capacity;
            o.data = nullptr;
            o.size = 0;
            o.capacity = 0;
        }
        return *this;
    }

    /// NUL-terminated contents (`""` when empty).
    const char* c_str() const { return data ? data : ""; }
    size_t length() const { return size; }
    bool empty() const { return size == 0; }

    /// Copy `n` bytes from `src` (which need not be NUL-terminated), storing a
    /// fresh NUL-terminated buffer. Returns false on alloc failure.
    bool assign(const char* src, size_t n) {
        char* fresh = static_cast<char*>(nros_platform_malloc(n + 1));
        if (fresh == nullptr) return false;
        for (size_t i = 0; i < n; ++i)
            fresh[i] = src[i];
        fresh[n] = '\0';
        nros_platform_free(data);
        data = fresh;
        size = n;
        capacity = n + 1;
        return true;
    }

    void clear() {
        nros_platform_free(data);
        data = nullptr;
        size = 0;
        capacity = 0;
    }

    /// Assign from a NUL-terminated C string. Returns false on alloc failure,
    /// in which case the string is left EMPTY rather than holding its previous
    /// value — a stale read is the one outcome a caller cannot detect.
    bool assign(const char* src) {
        if (src == nullptr) {
            clear();
            return true;
        }
        if (!assign(src, strlen(src))) {
            clear();
            return false;
        }
        return true;
    }

    /// Compare with a C string.
    bool operator==(const char* s) const {
        if (s == nullptr) return size == 0;
        return strcmp(c_str(), s) == 0;
    }
    bool operator!=(const char* s) const { return !(*this == s); }

#ifdef NROS_CPP_HAS_STD_STRING
    // phase-417 W1.c — `std::string` interop, the same conversions
    // `FixedString<N>` carries, so a ported assignment reads identically
    // whichever string mode (RFC-0033) codegen picked for the field.
    //
    // These COPY and call through to `assign()`; no data member is added, so
    // the `{ char* data; size_t size; size_t capacity; }` repr(C) layout this
    // type exists for is unchanged.
    //
    // NOTE the divergence from `std::string`, which is why `assign()` stays
    // the verb for anything that must handle failure: `std::string` THROWS on
    // allocation failure and nano-ros has no exceptions (RFC-0018), so
    // `operator=` cannot report one. It leaves the string EMPTY rather than
    // stale, which `empty()` can see.

    /// Assign from a `std::string`. Empty on alloc failure — see the note
    /// above; use `assign()` when the failure must be handled.
    HeapString& operator=(const std::string& s) {
        if (!assign(s.data(), s.size())) clear();
        return *this;
    }

    /// Assign from a C string. Empty on alloc failure — see `operator=(const
    /// std::string&)`.
    HeapString& operator=(const char* s) {
        (void)assign(s);
        return *this;
    }

    /// Copy out as a `std::string`. Implicit, so `std::string s = msg.data;`
    /// works the same as it does for `FixedString<N>`.
    // NOLINTNEXTLINE(google-explicit-constructor)
    operator std::string() const { return std::string(c_str(), size); }

    /// Explicit spelling of the conversion above.
    std::string to_string() const { return std::string(c_str(), size); }

    bool operator==(const std::string& s) const {
        return size == s.size() && memcmp(c_str(), s.data(), size) == 0;
    }
    bool operator!=(const std::string& s) const { return !(*this == s); }
#endif
};

} // namespace nros

#endif // NROS_CPP_HEAP_STRING_HPP
