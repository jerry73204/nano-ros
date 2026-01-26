#pragma once

/// @file clock.hpp
/// @brief Clock class for nano-ros (rclcpp-compatible API)

#include "nano_ros/time.hpp"

#include <memory>

namespace nano_ros {

namespace ffi {
struct RustClock;
}  // namespace ffi

/// @brief Clock type enumeration
enum class ClockType {
    /// System clock (wall time, can jump forward/backward)
    SystemTime,
    /// Steady clock (monotonic, never jumps)
    SteadyTime,
    /// ROS time (simulation time when available)
    RosTime,
};

/// @brief Clock for time measurement
///
/// This class provides an rclcpp-compatible API for accessing time.
/// Three types of clocks are supported:
/// - SystemTime: Wall clock time, can be adjusted
/// - SteadyTime: Monotonic clock, never jumps
/// - RosTime: Simulation time (uses system time when not in simulation)
///
/// Example:
/// @code
/// auto clock = nano_ros::Clock::create_system();
/// auto now = clock->now();
/// std::cout << "Current time: " << now.seconds() << "s" << std::endl;
/// @endcode
class Clock {
public:
    /// @brief Create a system clock (wall time)
    /// @return Shared pointer to the clock
    static std::shared_ptr<Clock> create_system();

    /// @brief Create a steady clock (monotonic)
    /// @return Shared pointer to the clock
    static std::shared_ptr<Clock> create_steady();

    /// @brief Create a ROS time clock
    /// @return Shared pointer to the clock
    static std::shared_ptr<Clock> create_ros();

    /// @brief Get the clock type
    /// @return Clock type
    ClockType get_clock_type() const;

    /// @brief Get the current time
    /// @return Current time
    Time now() const;

    ~Clock();

    // Non-copyable
    Clock(const Clock&) = delete;
    Clock& operator=(const Clock&) = delete;

    // Movable
    Clock(Clock&&) noexcept;
    Clock& operator=(Clock&&) noexcept;

private:
    Clock();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace nano_ros
