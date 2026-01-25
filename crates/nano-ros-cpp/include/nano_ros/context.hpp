#pragma once

/// @file context.hpp
/// @brief Context class for nano-ros initialization

#include <cstdint>
#include <memory>
#include <string>

namespace nano_ros {

// Forward declarations
class Node;

namespace ffi {
struct RustContext;
} // namespace ffi

/// @brief Context for nano-ros runtime
///
/// The Context manages the underlying transport connection and provides
/// the factory method for creating nodes. A Context must be created before
/// any nodes can be instantiated.
///
/// Example:
/// @code
/// auto ctx = nano_ros::Context::from_env();
/// auto node = ctx->create_node("my_node");
/// @endcode
class Context {
public:
    /// @brief Create a context with default options
    /// @return Shared pointer to the context
    /// @throws std::runtime_error if context creation fails
    static std::shared_ptr<Context> create();

    /// @brief Create a context from environment variables
    ///
    /// Reads configuration from:
    /// - ROS_DOMAIN_ID: Domain ID (default: 0)
    /// - ZENOH_LOCATOR: Router address (default: tcp/127.0.0.1:7447)
    /// - ZENOH_MODE: Session mode (default: client)
    ///
    /// @return Shared pointer to the context
    /// @throws std::runtime_error if context creation fails
    static std::shared_ptr<Context> from_env();

    /// @brief Check if the context is valid
    /// @return true if context is in a valid state
    bool ok() const;

    /// @brief Get the ROS domain ID
    /// @return Domain ID (0-232)
    uint32_t domain_id() const;

    /// @brief Create a node with the given name
    /// @param name Node name (must be a valid ROS identifier)
    /// @return Shared pointer to the node
    /// @throws std::runtime_error if node creation fails
    std::shared_ptr<Node> create_node(const std::string& name);

    /// @brief Create a node with name and namespace
    /// @param name Node name (must be a valid ROS identifier)
    /// @param ns Node namespace (e.g., "/my_robot")
    /// @return Shared pointer to the node
    /// @throws std::runtime_error if node creation fails
    std::shared_ptr<Node> create_node(
        const std::string& name,
        const std::string& ns
    );

    ~Context();

    // Non-copyable
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    // Movable
    Context(Context&&) noexcept;
    Context& operator=(Context&&) noexcept;

private:
    Context();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace nano_ros
