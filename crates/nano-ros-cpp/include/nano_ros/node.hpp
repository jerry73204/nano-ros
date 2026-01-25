#pragma once

/// @file node.hpp
/// @brief Node class for nano-ros

#include <memory>
#include <string>

namespace nano_ros {

// Forward declarations
class Context;

namespace ffi {
struct RustNode;
} // namespace ffi

/// @brief A ROS 2 node
///
/// Nodes are the main entry point for ROS 2 communication. They provide
/// methods to create publishers, subscribers, service clients, and servers.
///
/// Nodes are created via Context::create_node() and should not be
/// constructed directly.
///
/// Example:
/// @code
/// auto ctx = nano_ros::Context::from_env();
/// auto node = ctx->create_node("talker");
/// std::cout << "Node: " << node->get_fully_qualified_name() << std::endl;
/// @endcode
class Node : public std::enable_shared_from_this<Node> {
public:
    /// @brief Get the node name
    /// @return Node name without namespace
    std::string get_name() const;

    /// @brief Get the node namespace
    /// @return Node namespace (e.g., "/" or "/my_robot")
    std::string get_namespace() const;

    /// @brief Get the fully qualified node name
    /// @return Full name including namespace (e.g., "/my_robot/talker")
    std::string get_fully_qualified_name() const;

    ~Node();

    // Non-copyable
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // Movable
    Node(Node&&) noexcept;
    Node& operator=(Node&&) noexcept;

private:
    friend class Context;

    Node();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace nano_ros
