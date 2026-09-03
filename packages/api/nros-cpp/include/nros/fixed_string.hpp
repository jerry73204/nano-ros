// nros-cpp: FixedString — fixed-capacity null-terminated string
// Freestanding C++14 — no exceptions, no STL required
//
// Memory layout is repr(C) compatible: identical to char[N].
// Used by codegen-generated message structs for string fields.

/**
 * @file fixed_string.hpp
 * @ingroup grp_support
 * @brief `nros::FixedString<N>` — fixed-capacity null-terminated string.
 */

#ifndef NROS_CPP_FIXED_STRING_HPP
#define NROS_CPP_FIXED_STRING_HPP

#include <cstddef>
#include <string.h>

// phase-417 W1.c — `std::string` interop. `<string>` is absent from a minimal
// freestanding libcpp, so gate on the declared std flavour, else ASK THE
// COMPILER. `__STDC_HOSTED__` is the wrong question (issue 0112): the Zephyr
// XRCE C++ leaves compile `-fno-freestanding -nostdinc++`, i.e. hosted with no
// `<string>`. Same `__has_include` idiom as `declared_qos.hpp:80`; the longer
// rationale is in `publisher.hpp`.
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

/// Fixed-capacity null-terminated string.
///
/// Wraps a `char[N]` buffer with safe assignment and query methods.
/// Memory layout is identical to `char[N]`, so the type is C-ABI
/// compatible and can be passed across the runtime boundary by value.
///
/// Usage:
/// ```cpp
/// nros::FixedString<256> name;
/// name = "hello world";
/// printf("%s (len=%zu)\n", name.c_str(), name.length());
/// ```
template <size_t N> struct FixedString {
    char data[N];

    /// Default constructor — empty string.
    FixedString() { data[0] = '\0'; }

    /// Assign from a C string. Truncates if longer than capacity.
    FixedString& operator=(const char* s) {
        if (s == nullptr) {
            data[0] = '\0';
            return *this;
        }
        size_t i = 0;
        for (; i < N - 1 && s[i] != '\0'; ++i) {
            data[i] = s[i];
        }
        data[i] = '\0';
        return *this;
    }

    /// Get a pointer to the null-terminated string.
    const char* c_str() const { return data; }

    /// Get the length of the string (up to N-1).
    size_t length() const {
        size_t len = 0;
        while (len < N && data[len] != '\0')
            ++len;
        return len;
    }

    /// Maximum number of characters (excluding null terminator).
    static constexpr size_t capacity() { return N - 1; }

    /// Number of characters, excluding the terminator — the `std::string`
    /// spelling of `length()`. phase-417 W1.c; ungated, no `<string>` needed.
    size_t size() const { return length(); }

    /// True when the string holds no characters. phase-417 W1.c; ungated.
    bool empty() const { return data[0] == '\0'; }

    /// Compare with a C string.
    bool operator==(const char* s) const {
        if (s == nullptr) return data[0] == '\0';
        return strncmp(data, s, N) == 0;
    }

    bool operator!=(const char* s) const { return !(*this == s); }

#ifdef NROS_CPP_HAS_STD_STRING
    // phase-417 W1.c — `std::string` interop.
    //
    // Codegen emits a message's string field as `FixedString<N>` where
    // upstream rosidl emits `std::string`, so the very first line of a ported
    // node —
    //
    //     message.data = "Hello, world! " + std::to_string(count_++);
    //
    // did not compile: the only assignment was from `const char*`. These
    // members COPY and call through to the existing `const char*` paths; they
    // add no data member and no second code path (RFC-0087 §"Who implements an
    // adopted name" — container/string conversions that copy and delegate are
    // wrapper-side ergonomics), so the `char[N]` layout the FFI relies on is
    // unchanged.
    //
    // Truncation is the same bounded-capacity contract `operator=(const
    // char*)` has always had: a longer source is cut at `N - 1` characters.

    /// Assign from a `std::string`. Truncates if longer than capacity.
    FixedString& operator=(const std::string& s) { return *this = s.c_str(); }

    /// Copy out as a `std::string`. Implicit, so `std::string s = msg.data;`
    /// and passing a field to a `const std::string&` parameter both work.
    // NOLINTNEXTLINE(google-explicit-constructor)
    operator std::string() const { return std::string(data, length()); }

    /// Explicit spelling of the conversion above, for call sites where an
    /// implicit conversion would be ambiguous.
    std::string to_string() const { return std::string(data, length()); }

    bool operator==(const std::string& s) const { return *this == s.c_str(); }
    bool operator!=(const std::string& s) const { return !(*this == s.c_str()); }
#endif
};

} // namespace nros

#endif // NROS_CPP_FIXED_STRING_HPP
