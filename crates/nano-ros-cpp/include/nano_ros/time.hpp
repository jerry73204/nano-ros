#pragma once

/// @file time.hpp
/// @brief Time class for nano-ros (rclcpp-compatible API)

#include "nano_ros/duration.hpp"

#include <chrono>
#include <cstdint>

namespace nano_ros {

/// @brief Represents a point in time
///
/// This class provides an rclcpp-compatible API for representing time.
/// Internally uses seconds and nanoseconds to match builtin_interfaces/msg/Time.
///
/// Example:
/// @code
/// auto now = nano_ros::Time::from_nanoseconds(clock_gettime_ns());
/// auto later = now + nano_ros::Duration::from_seconds(1.0);
/// auto diff = later - now;  // Returns Duration
/// @endcode
class Time {
public:
    /// @brief Construct a zero time (epoch)
    Time();

    /// @brief Construct from seconds and nanoseconds since epoch
    /// @param seconds Seconds since epoch
    /// @param nanoseconds Nanoseconds component (0-999999999)
    Time(int32_t seconds, uint32_t nanoseconds);

    /// @brief Create a Time from nanoseconds since epoch
    /// @param nanoseconds Nanoseconds since epoch
    /// @return Time object
    static Time from_nanoseconds(int64_t nanoseconds);

    /// @brief Create a zero time (epoch)
    /// @return Zero Time
    static Time zero();

    /// @brief Get the seconds component
    /// @return Seconds since epoch
    int32_t sec() const {
        return sec_;
    }

    /// @brief Get the nanoseconds component
    /// @return Nanoseconds (0-999999999)
    uint32_t nanosec() const {
        return nanosec_;
    }

    /// @brief Convert to total nanoseconds since epoch
    /// @return Total nanoseconds
    int64_t nanoseconds() const;

    /// @brief Convert to floating-point seconds since epoch
    /// @return Seconds as double
    double seconds() const;

    /// @brief Check if time is zero (epoch)
    bool is_zero() const {
        return sec_ == 0 && nanosec_ == 0;
    }

    // Comparison operators
    bool operator==(const Time& rhs) const;
    bool operator!=(const Time& rhs) const;
    bool operator<(const Time& rhs) const;
    bool operator<=(const Time& rhs) const;
    bool operator>(const Time& rhs) const;
    bool operator>=(const Time& rhs) const;

    // Arithmetic operators
    Time operator+(const Duration& rhs) const;
    Time operator-(const Duration& rhs) const;
    Duration operator-(const Time& rhs) const;

    Time& operator+=(const Duration& rhs);
    Time& operator-=(const Duration& rhs);

private:
    int32_t sec_;
    uint32_t nanosec_;
};

}  // namespace nano_ros
