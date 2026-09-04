// nros-cpp: Result type for error handling
// Freestanding C++ — no exceptions, no STL required

/**
 * @file result.hpp
 * @ingroup grp_errors
 * @brief `nros::Result`, `nros::ErrorCode`, and the `NROS_TRY` macro.
 *
 * See @ref error_codes for the full code table and recovery guidance.
 */

#ifndef NROS_CPP_RESULT_HPP
#define NROS_CPP_RESULT_HPP

#include <cstdint>
#include <utility>
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
#include <cstdio>
#endif

namespace nros {

/// Error codes returned by nros-cpp functions.
///
/// Values match the C `nros_cpp_ret_t` enum in `<nros/nros_cpp_generated.h>`.
/// Issue #229 — value-identical to the C `NROS_RET_*` codes AND the
/// `NROS_CPP_RET_*` FFI codes (one numbering across all three spaces), so
/// `Result(<any C-ABI return>)` is correct by identity. The static_assert
/// pin tables below and in parameter.hpp fail the build on re-divergence.
enum class ErrorCode : int32_t {
    /// Success.
    Ok = 0,
    /// Generic failure not covered by a more specific code.
    Error = -1,
    /// Operation deadline elapsed before completion.
    Timeout = -2,
    /// Null pointer, empty topic name, or out-of-range value.
    InvalidArgument = -3,
    /// Entity not found (topic, parameter, service…).
    NotFound = -4,
    /// Already exists (duplicate declare/register).
    AlreadyExists = -5,
    /// Static pool exhausted (executor slots, subscription buffers, …).
    Full = -6,
    /// `nros::init()` was never called or the entity is in a default
    /// state. See `is_valid()` on entity classes.
    NotInitialized = -7,
    /// Operation invalid in the current state (bad call sequence).
    BadSequence = -8,
    /// Service request/reply failed.
    ServiceFailed = -9,
    /// Publish failed.
    PublishFailed = -10,
    /// Subscription create/take failed.
    SubscriptionFailed = -11,
    /// Operation not allowed for this entity/backend.
    NotAllowed = -12,
    /// Request was rejected — the peer considered it and declined.
    /// A goal rejected by an action server, or a QoS/ABI
    /// incompatibility. Distinct from `Error`, which means the
    /// request never got that far (issue 0868).
    Rejected = -13,
    /// Transient — no data ready yet (non-blocking take). Retry later.
    TryAgain = -14,
    /// A blocking call was made from inside a callback.
    Reentrant = -15,
    /// Operation not implemented by the active backend.
    Unsupported = -16,
    /// Underlying zenoh-pico / DDS transport rejected the operation.
    TransportError = -100,
};

// Issue #229 pin (self-consistency half): the values above ARE the shared
// numbering. The cross-space asserts against the real C constants live in
// parameter.hpp (vs NROS_RET_*) and node.hpp (vs NROS_CPP_RET_*), where
// those headers are visible.
static_assert(static_cast<int32_t>(ErrorCode::NotFound) == -4 &&
                  static_cast<int32_t>(ErrorCode::AlreadyExists) == -5 &&
                  static_cast<int32_t>(ErrorCode::Full) == -6 &&
                  static_cast<int32_t>(ErrorCode::NotInitialized) == -7 &&
                  static_cast<int32_t>(ErrorCode::TryAgain) == -14 &&
                  static_cast<int32_t>(ErrorCode::Reentrant) == -15 &&
                  static_cast<int32_t>(ErrorCode::Unsupported) == -16,
              "ErrorCode numbering must match nros_ret_t (issue #229)");

/// Result type for fallible operations.
///
/// This replaces exceptions in freestanding C++. Use the NROS_TRY macro
/// for early return on error.
class Result {
  public:
    /// Default-construct a success.
    constexpr Result() : code_(ErrorCode::Ok) {}
    /// Construct from a typed code.
    constexpr Result(ErrorCode code) : code_(code) {}
    /// Construct from a raw FFI return value (`int32_t`).
    constexpr Result(int32_t raw) : code_(static_cast<ErrorCode>(raw)) {}

    /// Returns true if the operation succeeded.
    bool ok() const { return code_ == ErrorCode::Ok; }

    /// Explicit bool conversion — allows `if (result) { ... }`.
    explicit operator bool() const { return ok(); }

    /// Get the underlying error code.
    ErrorCode code() const { return code_; }

    /// Get the raw integer code (for FFI interop).
    int32_t raw() const { return static_cast<int32_t>(code_); }

    /// Named constructors.
    static constexpr Result success() { return Result(ErrorCode::Ok); }

  private:
    ErrorCode code_;
};

/// Early-return macro for error propagation (replaces try/catch).
///
/// Usage:
/// ```cpp
/// nros::Result do_stuff() {
///     NROS_TRY(nros::init());
///     NROS_TRY(node.create_publisher(pub, "/topic"));
///     return nros::Result::success();
/// }
/// ```
#define NROS_TRY(expr)                                                                             \
    do {                                                                                           \
        ::nros::Result _nros_r = (expr);                                                           \
        if (!_nros_r.ok()) return _nros_r;                                                         \
    } while (0)

/// Like NROS_TRY but for callers that need a custom return value
/// (e.g. `int main` examples returning 1 on failure).
///
/// Phase 123.B.1 — when `NROS_CPP_STD` is defined (POSIX / Zephyr
/// native_sim / threadx-linux + any host with `<cstdio>`), the
/// default logger writes `[nros] <file>:<line> <expr> -> <ret>` to
/// `stderr` so first-time users see failures immediately. Embedded
/// builds without stdio fall through to the silent default.
///
/// Override `NROS_TRY_LOG(file, line, expr, ret)` before including
/// this header to attach a custom logger (Zephyr's `LOG_ERR`,
/// semihosting, defmt, etc.). Opt out entirely with
/// `#define NROS_TRY_LOG(file, line, expr, ret) ((void)0)`.
#ifndef NROS_TRY_LOG
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    ::std::fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))
#else
#define NROS_TRY_LOG(file, line, expr, ret) ((void)(file), (void)(line), (void)(expr), (void)(ret))
#endif
#endif

#define NROS_TRY_RET(expr, retval)                                                                 \
    do {                                                                                           \
        ::nros::Result _nros_r = (expr);                                                           \
        if (!_nros_r.ok()) {                                                                       \
            NROS_TRY_LOG(__FILE__, __LINE__, #expr, _nros_r.raw());                                \
            return (retval);                                                                       \
        }                                                                                          \
    } while (0)

/// Like NROS_TRY but for void-returning callers (RTOS `app_main(void)`,
/// task entry points, …). Logs the failure via the same `NROS_TRY_LOG`
/// hook as `NROS_TRY_RET` and bails with a bare `return;`.
#define NROS_CHECK(expr)                                                                           \
    do {                                                                                           \
        ::nros::Result _nros_r = (expr);                                                           \
        if (!_nros_r.ok()) {                                                                       \
            NROS_TRY_LOG(__FILE__, __LINE__, #expr, _nros_r.raw());                                \
            return;                                                                                \
        }                                                                                          \
    } while (0)

/// Phase 123.B.4 — templated value-or-error wrapper.
///
/// Lets factory functions return constructed entities by value
/// instead of forcing the out-param + Result idiom. Trade-off:
/// requires `T` to be default-constructible and move-constructible
/// (Node, Publisher, Subscription all satisfy both today). Storage
/// is direct (the value lives inline, no allocation) — when the
/// result holds an error the value member is default-constructed
/// and idle.
///
/// Usage:
/// ```cpp
/// auto node_r = nros::Node::make("my_node");
/// if (!node_r.ok()) return node_r.error_as_result();
/// auto& node = node_r.value();
/// ```
///
/// Out-param `create_node(node, "name")` remains the canonical
/// zero-cost API for embedded / strictly-no-alloc code; `make()`
/// is a hosted-friendly convenience that closes the rclcpp /
/// rclrs idiom gap.
template <typename T> class Expected {
  public:
    static Expected ok(T value) {
        Expected e;
        e.ok_ = true;
        e.value_ = ::std::move(value);
        return e;
    }
    static Expected error(ErrorCode code) {
        Expected e;
        e.ok_ = false;
        e.error_ = code;
        return e;
    }
    static Expected error(const Result& r) { return error(r.code()); }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    T& value() & { return value_; }
    const T& value() const& { return value_; }
    T&& value() && { return ::std::move(value_); }

    ErrorCode error() const { return error_; }
    Result error_as_result() const { return Result(error_); }

  private:
    Expected() : ok_(false), error_(ErrorCode::Error), value_() {}

    bool ok_;
    ErrorCode error_;
    T value_;
};

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0087 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0087 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// `rclcpp::Result` is NOT an upstream rclcpp name — it is a convenience the
// compat header carried, kept here because step A must not lose anything. It
// names `nros::Result` exactly; RFC-0018 forbids exceptions, so there is no
// upstream error type to adopt in its place.
namespace rclcpp {
using ::nros::Result;
} // namespace rclcpp

#endif // NROS_CPP_RESULT_HPP
