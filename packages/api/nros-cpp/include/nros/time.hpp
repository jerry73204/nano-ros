// nros-cpp: Time — a point on a clock's timeline
// Freestanding C++ — no exceptions, no STL required

/**
 * @file time.hpp
 * @ingroup grp_clock
 * @brief `nros::Time` — a timestamp, mirroring `rclcpp::Time`.
 *
 * Issue 0789. `nros_time_t` and the `nros_time_*` arithmetic have existed in C
 * since the C API shipped; this is the C++ face over them, so
 * `msg.header.stamp` can be filled from `node.now()` instead of by hand.
 */

#ifndef NROS_CPP_TIME_HPP
#define NROS_CPP_TIME_HPP

#include <cstdint>
#include <stdint.h>

#include "nros/duration.hpp"

// `nros_clock_type_t` and `nros_time_from_nanoseconds` — the C surface this
// wraps. `nros_generated.h` carries its own `extern "C"` guard.
#include "nros/clock.h"

namespace nros {

/// A point in time, held as nanoseconds on a named clock.
///
/// Mirrors `rclcpp::Time`. The clock TYPE travels with the value, as it does in
/// rclcpp, so a steady-clock reading and a system-clock reading are
/// distinguishable after the fact.
///
/// Divergence from rclcpp, forced by `-fno-exceptions` (RFC-0018): rclcpp
/// throws `std::runtime_error` when two `Time`s of different clock types are
/// compared or subtracted. There is no way to report that from an operator
/// here, so the comparison is on nanoseconds alone. Check `get_clock_type()`
/// when the distinction matters.
///
/// Usage:
/// ```cpp
/// nros::Time stamp = node.now();
/// stamp.to_msg(msg.header.stamp);
/// nros::Duration age = node.now() - stamp;
/// ```
class Time {
  public:
    /// Nanoseconds since the clock's epoch. Both arguments default, so
    /// `Time()` is the zero point on the system clock — the
    /// `rclcpp::Time(int64_t = 0, rcl_clock_type_t = RCL_SYSTEM_TIME)` shape.
    constexpr Time(int64_t nanoseconds = 0, nros_clock_type_t clock_type = NROS_CLOCK_SYSTEM_TIME)
        : ns_(nanoseconds), clock_type_(clock_type) {}

    /// Seconds + nanoseconds, the `rclcpp::Time(int32_t, uint32_t,
    /// rcl_clock_type_t)` shape.
    constexpr Time(int32_t seconds, uint32_t nanoseconds,
                   nros_clock_type_t clock_type = NROS_CLOCK_SYSTEM_TIME)
        : ns_(static_cast<int64_t>(seconds) * NANOSECONDS_PER_SECOND +
              static_cast<int64_t>(nanoseconds)),
          clock_type_(clock_type) {}

    /// The largest representable time — `rclcpp::Time::max()`'s value.
    static constexpr Time max() { return Time(INT32_MAX, 999999999u); }

    /// Nanoseconds since the clock's epoch.
    constexpr int64_t nanoseconds() const { return ns_; }

    /// (Fractional) seconds since the clock's epoch.
    constexpr double seconds() const {
        return static_cast<double>(ns_) / static_cast<double>(NANOSECONDS_PER_SECOND);
    }

    /// Which clock this value was read from.
    constexpr nros_clock_type_t get_clock_type() const { return clock_type_; }

    // -- Arithmetic -------------------------------------------------------

    constexpr Time operator+(const Duration& rhs) const {
        return Time(ns_ + rhs.nanoseconds(), clock_type_);
    }
    constexpr Time operator-(const Duration& rhs) const {
        return Time(ns_ - rhs.nanoseconds(), clock_type_);
    }
    /// The span between two instants. See the clock-type note above.
    constexpr Duration operator-(const Time& rhs) const {
        return Duration::from_nanoseconds(ns_ - rhs.ns_);
    }

    Time& operator+=(const Duration& rhs) {
        ns_ += rhs.nanoseconds();
        return *this;
    }
    Time& operator-=(const Duration& rhs) {
        ns_ -= rhs.nanoseconds();
        return *this;
    }

    // -- Comparison -------------------------------------------------------

    constexpr bool operator==(const Time& rhs) const { return ns_ == rhs.ns_; }
    constexpr bool operator!=(const Time& rhs) const { return ns_ != rhs.ns_; }
    constexpr bool operator<(const Time& rhs) const { return ns_ < rhs.ns_; }
    constexpr bool operator<=(const Time& rhs) const { return ns_ <= rhs.ns_; }
    constexpr bool operator>(const Time& rhs) const { return ns_ > rhs.ns_; }
    constexpr bool operator>=(const Time& rhs) const { return ns_ >= rhs.ns_; }

    /// Fill a generated `builtin_interfaces/msg/Time` — anything with `sec` /
    /// `nanosec` members, which is the shape rosidl-codegen emits. This is the
    /// call a publisher actually needs: stamping a header.
    ///
    /// rclcpp spells it as an implicit `operator builtin_interfaces::msg::
    /// Time()`. nros-cpp is header-only and message types are generated per
    /// user package, so the client library cannot name one; a template that
    /// binds to the generated struct is the same conversion without the
    /// dependency. (`rust:Time::to_ros_msg` is the same call one language over,
    /// still recorded as a gap.)
    ///
    /// ```cpp
    /// node.now().to_msg(msg.header.stamp);
    /// ```
    template <typename TimeMsgT> void to_msg(TimeMsgT& out) const {
        // The C decomposition floors and keeps `nanosec` in `[0, 1e9)`, which
        // is what `Time.msg` specifies — for a time before the epoch as much
        // as after it (issue 0799). `Duration::to_msg` is the same call on the
        // duration entry point.
        nros_time_t t = nros_time_from_nanoseconds(ns_);
        out.sec = t.sec;
        out.nanosec = t.nanosec;
    }

  private:
    int64_t ns_;
    nros_clock_type_t clock_type_;
};

/// `duration + time`, so the sum reads either way round. The one free operator
/// `rclcpp` declares for these types; every other one is a member on both
/// sides.
constexpr Time operator+(const Duration& lhs, const Time& rhs) {
    return rhs + lhs;
}

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0087 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0087 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// phase-417 W1.d — `rclcpp::Time stamp = node->now();` is what a ported
// publisher writes to stamp a header. The nano-ros type UNCHANGED, not a
// wrapper over it.
namespace rclcpp {
using Time = ::nros::Time;
} // namespace rclcpp

#endif // NROS_CPP_TIME_HPP
