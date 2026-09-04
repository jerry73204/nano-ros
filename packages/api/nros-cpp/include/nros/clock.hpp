// nros-cpp: Clock — a time source
// Freestanding C++ — no exceptions, no STL required

/**
 * @file clock.hpp
 * @ingroup grp_clock
 * @brief `nros::Clock` — reads the current time, mirroring `rclcpp::Clock`.
 *
 * Issue 0789. A thin C++ face over the `nros_clock_*` C surface (RFC-0073
 * defines the platform clock contract underneath it); this header invents no
 * capability of its own.
 */

#ifndef NROS_CPP_CLOCK_HPP
#define NROS_CPP_CLOCK_HPP

#include <cstdint>
#include <stdint.h>

#include "nros/time.hpp"

// The `nros_clock_*` entry points. `nros_generated.h` carries its own
// `extern "C"` guard.
#include "nros/clock.h"

namespace nros {

/// A time source: system, steady, or ROS time.
///
/// Mirrors `rclcpp::Clock`. The clock type is fixed at construction and
/// readable afterwards with `get_clock_type()`.
///
/// Usage:
/// ```cpp
/// nros::Clock steady(NROS_CLOCK_STEADY_TIME);
/// nros::Time t0 = steady.now();
/// // ...
/// nros::Duration elapsed = steady.now() - t0;
/// ```
///
/// Copyable and trivially destructible on purpose. `nros_clock_t` owns no
/// allocation and no handle — `nros_clock_fini` only marks the struct shut
/// down — so unlike `rclcpp::Clock` (which finalises an `rcl_clock_t` holding
/// an allocator) there is nothing for a destructor to release.
///
/// A `NROS_CLOCK_ROS_TIME` clock can be driven by a simulator's or a bag
/// player's `/clock` (issue 0789): `ros_time_is_active()` reports whether it
/// is, `started()` whether a sample has arrived. The switches that INSTALL a
/// time are the C ones (`nros_set_ros_time_override` and friends) rather than
/// members here, because the override is process-global — one simulated clock
/// per image, as in Rust — and hanging a global's setter off an instance would
/// read as per-clock state.
class Clock {
  public:
    /// Construct a clock of the given type.
    ///
    /// Defaults to `NROS_CLOCK_SYSTEM_TIME`, matching
    /// `rclcpp::Clock(rcl_clock_type_t = RCL_SYSTEM_TIME)`. (A `Node`'s own
    /// clock is ROS time, also as in rclcpp — see `Node::get_clock()`.)
    ///
    /// The clock type is the C enum `nros_clock_type_t` rather than a second
    /// C++ spelling of the same four values: rclcpp likewise takes rcl's
    /// `rcl_clock_type_t`, and one vocabulary across the two languages is the
    /// point of issue 0789.
    explicit Clock(nros_clock_type_t clock_type = NROS_CLOCK_SYSTEM_TIME)
        : clock_(nros_clock_get_zero_initialized()) {
        nros_clock_init(&clock_, clock_type);
    }

    /// The current time.
    ///
    /// Infallible, following the C surface: `nros_clock_get_now_ns` cannot fail
    /// once the clock is valid, because the platform seam (RFC-0073) either has
    /// a clock or the image did not boot. An INVALID clock — one built with
    /// `NROS_CLOCK_UNINITIALIZED` — reads zero; `is_valid()` is how that is
    /// detected, not a `Result` on every timestamp.
    Time now() const {
        int64_t nanoseconds = 0;
        if (nros_clock_get_now_ns(&clock_, &nanoseconds) != 0) {
            nanoseconds = 0;
        }
        return Time(nanoseconds, get_clock_type());
    }

    /// Which kind of clock this is.
    nros_clock_type_t get_clock_type() const { return nros_clock_get_type(&clock_); }

    /// Whether the clock initialised. False for a clock built with an invalid
    /// type; the same predicate as `Node::is_valid()` / `Timer::is_valid()`.
    bool is_valid() const { return nros_clock_is_valid(&clock_); }

    /// Whether a `/clock` source is driving this clock — `rclcpp::Clock::
    /// ros_time_is_active()`.
    ///
    /// False for anything but a `NROS_CLOCK_ROS_TIME` clock, and false for a
    /// ROS-time clock with no override installed (it is reading the system
    /// clock). Infallible for the reason `now()` is: the report is a
    /// predicate, not a status the caller has to unpack.
    bool ros_time_is_active() const {
        bool enabled = false;
        if (rcl_is_enabled_ros_time_override(&clock_, &enabled) != 0) {
            return false;
        }
        return enabled;
    }

    /// Whether the clock has produced a time yet — `rclcpp::Clock::started()`.
    ///
    /// For a ROS-time clock this is "the simulator has published at least one
    /// `/clock` sample"; a system or steady clock has started as soon as it is
    /// valid. (`rclcpp::Clock::wait_until_started` is deliberately absent —
    /// RFC-0021 forbids a blocking helper that does not drive the executor.)
    bool started() const { return rcl_clock_time_started(&clock_); }

  private:
    nros_clock_t clock_;
};

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0089 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0089 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// phase-417 W1.d — `rclcpp::Clock` is the nano-ros type UNCHANGED.
// `rclcpp::Node::get_clock()` (`nros/nros.hpp`) hands back a pointer to the
// node's own, because there is no allocator here to hand back a `SharedPtr`
// from (RFC-0022); the `node->get_clock()->now()` spelling is unchanged.
namespace rclcpp {
using Clock = ::nros::Clock;
} // namespace rclcpp

#endif // NROS_CPP_CLOCK_HPP
