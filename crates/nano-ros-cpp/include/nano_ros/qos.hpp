#pragma once

/// @file qos.hpp
/// @brief Quality of Service settings for nano-ros (rclcpp-compatible API)

#include <cstddef>
#include <cstdint>

namespace nano_ros {

/// @brief QoS History policy
enum class HistoryPolicy {
    /// Keep last N samples
    KeepLast,
    /// Keep all samples (until resource limits)
    KeepAll,
};

/// @brief QoS Reliability policy
enum class ReliabilityPolicy {
    /// Best effort delivery (may drop messages)
    BestEffort,
    /// Reliable delivery (retransmit if needed)
    Reliable,
};

/// @brief QoS Durability policy
enum class DurabilityPolicy {
    /// Only deliver to currently connected subscribers
    Volatile,
    /// Store messages for late-joining subscribers
    TransientLocal,
};

/// @brief Quality of Service settings
///
/// This class provides an rclcpp-compatible API for configuring QoS settings.
/// Uses a builder pattern for fluent configuration.
///
/// Example:
/// @code
/// // Using depth constructor (like rclcpp)
/// auto qos = nano_ros::QoS(10).reliable().transient_local();
///
/// // Using presets
/// auto sensor_qos = nano_ros::QoS::sensor_data();
/// auto service_qos = nano_ros::QoS::services();
/// @endcode
class QoS {
public:
    /// @brief Construct QoS with keep_last history and given depth
    /// @param depth History depth (for keep_last)
    explicit QoS(size_t depth);

    /// @brief Default QoS (reliable, volatile, keep_last(10))
    static QoS default_qos();

    /// @brief Sensor data QoS preset (best_effort, volatile, keep_last(5))
    static QoS sensor_data();

    /// @brief Services QoS preset (reliable, volatile, keep_last(10))
    static QoS services();

    /// @brief Parameters QoS preset (reliable, volatile, keep_last(1000))
    static QoS parameters();

    // ========================================================================
    // Builder methods (return reference for chaining)
    // ========================================================================

    /// @brief Set history policy
    QoS& history(HistoryPolicy policy);

    /// @brief Set to keep_last with given depth
    QoS& keep_last(size_t depth);

    /// @brief Set to keep_all
    QoS& keep_all();

    /// @brief Set reliability policy
    QoS& reliability(ReliabilityPolicy policy);

    /// @brief Set to reliable
    QoS& reliable();

    /// @brief Set to best effort
    QoS& best_effort();

    /// @brief Set durability policy
    QoS& durability(DurabilityPolicy policy);

    /// @brief Set to volatile (cannot use 'volatile' as method name)
    QoS& durability_volatile();

    /// @brief Set to transient local
    QoS& transient_local();

    // ========================================================================
    // Accessors
    // ========================================================================

    /// @brief Get history policy
    HistoryPolicy get_history() const {
        return history_;
    }

    /// @brief Get history depth
    size_t get_depth() const {
        return depth_;
    }

    /// @brief Get reliability policy
    ReliabilityPolicy get_reliability() const {
        return reliability_;
    }

    /// @brief Get durability policy
    DurabilityPolicy get_durability() const {
        return durability_;
    }

private:
    HistoryPolicy history_{HistoryPolicy::KeepLast};
    size_t depth_{10};
    ReliabilityPolicy reliability_{ReliabilityPolicy::Reliable};
    DurabilityPolicy durability_{DurabilityPolicy::Volatile};
};

/// @brief Helper to create QoS with keep_last history
/// @param depth History depth
/// @return QoS with keep_last(depth)
inline QoS KeepLast(size_t depth) {
    return QoS(depth);
}

/// @brief Helper to create QoS with keep_all history
/// @return QoS with keep_all
inline QoS KeepAll() {
    return QoS(0).keep_all();
}

}  // namespace nano_ros
