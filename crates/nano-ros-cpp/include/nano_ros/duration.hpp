#pragma once

/// @file duration.hpp
/// @brief Duration class for nano-ros (rclcpp-compatible API)

#include <chrono>
#include <cstdint>

namespace nano_ros {

/// @brief Represents a duration of time
///
/// This class provides an rclcpp-compatible API for representing durations.
/// Internally uses seconds and nanoseconds to match builtin_interfaces/msg/Duration.
///
/// Example:
/// @code
/// auto d1 = nano_ros::Duration::from_seconds(1.5);
/// auto d2 = nano_ros::Duration::from_nanoseconds(500000000);
/// auto sum = d1 + d2;
/// std::cout << "Total: " << sum.seconds() << " seconds" << std::endl;
/// @endcode
class Duration {
public:
    /// @brief Construct a zero duration
    Duration();

    /// @brief Construct from seconds and nanoseconds
    /// @param seconds Seconds component (can be negative)
    /// @param nanoseconds Nanoseconds component (0-999999999)
    Duration(int32_t seconds, uint32_t nanoseconds);

    /// @brief Construct from std::chrono::nanoseconds
    explicit Duration(std::chrono::nanoseconds ns);

    /// @brief Construct from any std::chrono::duration
    template <class Rep, class Period>
    Duration(const std::chrono::duration<Rep, Period>& duration)
        : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration)) {}

    /// @brief Create a Duration from floating-point seconds
    /// @param seconds Duration in seconds
    /// @return Duration object
    static Duration from_seconds(double seconds);

    /// @brief Create a Duration from nanoseconds
    /// @param nanoseconds Duration in nanoseconds
    /// @return Duration object
    static Duration from_nanoseconds(int64_t nanoseconds);

    /// @brief Create a Duration from milliseconds
    /// @param milliseconds Duration in milliseconds
    /// @return Duration object
    static Duration from_milliseconds(int64_t milliseconds);

    /// @brief Create a zero duration
    /// @return Zero Duration
    static Duration zero();

    /// @brief Get the seconds component
    /// @return Seconds (can be negative)
    int32_t sec() const {
        return sec_;
    }

    /// @brief Get the nanoseconds component
    /// @return Nanoseconds (0-999999999)
    uint32_t nanosec() const {
        return nanosec_;
    }

    /// @brief Convert to total nanoseconds
    /// @return Total nanoseconds
    int64_t nanoseconds() const;

    /// @brief Convert to floating-point seconds
    /// @return Seconds as double
    double seconds() const;

    /// @brief Convert to std::chrono::nanoseconds
    std::chrono::nanoseconds to_chrono() const;

    /// @brief Check if duration is zero
    bool is_zero() const {
        return sec_ == 0 && nanosec_ == 0;
    }

    // Comparison operators
    bool operator==(const Duration& rhs) const;
    bool operator!=(const Duration& rhs) const;
    bool operator<(const Duration& rhs) const;
    bool operator<=(const Duration& rhs) const;
    bool operator>(const Duration& rhs) const;
    bool operator>=(const Duration& rhs) const;

    // Arithmetic operators
    Duration operator+(const Duration& rhs) const;
    Duration operator-(const Duration& rhs) const;
    Duration operator-() const;

    Duration& operator+=(const Duration& rhs);
    Duration& operator-=(const Duration& rhs);

private:
    int32_t sec_{0};
    uint32_t nanosec_{0};
};

}  // namespace nano_ros
