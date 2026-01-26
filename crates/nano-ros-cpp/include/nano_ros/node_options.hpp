#pragma once

/// @file node_options.hpp
/// @brief NodeOptions class for nano-ros (rclcpp-compatible API)

#include "nano_ros/clock.hpp"

#include <string>
#include <vector>

namespace nano_ros {

/// @brief Options for node creation
///
/// This class provides an rclcpp-compatible API for configuring node options.
/// Uses a builder pattern for fluent configuration.
///
/// Example:
/// @code
/// auto options = nano_ros::NodeOptions()
///     .node_name("my_node")
///     .namespace_("/my_robot")
///     .use_clock_type(nano_ros::ClockType::SteadyTime);
///
/// auto node = context->create_node(options);
/// @endcode
class NodeOptions {
public:
    /// @brief Construct default NodeOptions
    NodeOptions();

    /// @brief Construct NodeOptions with node name
    /// @param name Node name
    explicit NodeOptions(std::string name);

    // ========================================================================
    // Builder methods (return reference for chaining)
    // ========================================================================

    /// @brief Set the node name
    /// @param name Node name (must be a valid ROS identifier)
    /// @return Reference to this for chaining
    NodeOptions& node_name(const std::string& name);

    /// @brief Set the node namespace
    /// @param ns Node namespace (e.g., "/my_robot")
    /// @return Reference to this for chaining
    NodeOptions& namespace_(const std::string& ns);

    /// @brief Set the clock type for the node
    /// @param clock_type Clock type (SystemTime, SteadyTime, or RosTime)
    /// @return Reference to this for chaining
    NodeOptions& use_clock_type(ClockType clock_type);

    /// @brief Enable or disable intra-process communication
    /// @param enable Whether to enable intra-process comms
    /// @return Reference to this for chaining
    ///
    /// @note Not yet implemented in nano-ros, stored for API compatibility
    NodeOptions& use_intra_process_comms(bool enable);

    /// @brief Enable or disable parameter services
    /// @param enable Whether to start parameter services
    /// @return Reference to this for chaining
    ///
    /// @note Not yet implemented in nano-ros, stored for API compatibility
    NodeOptions& start_parameter_services(bool enable);

    // ========================================================================
    // Accessors
    // ========================================================================

    /// @brief Get the node name
    const std::string& get_node_name() const {
        return name_;
    }

    /// @brief Get the namespace
    const std::string& get_namespace() const {
        return ns_;
    }

    /// @brief Get the clock type
    ClockType get_clock_type() const {
        return clock_type_;
    }

    /// @brief Check if intra-process comms is enabled
    bool get_use_intra_process_comms() const {
        return use_intra_process_comms_;
    }

    /// @brief Check if parameter services are enabled
    bool get_start_parameter_services() const {
        return start_parameter_services_;
    }

private:
    std::string name_;
    std::string ns_;
    ClockType clock_type_{ClockType::SystemTime};
    bool use_intra_process_comms_{false};
    bool start_parameter_services_{true};
};

}  // namespace nano_ros
