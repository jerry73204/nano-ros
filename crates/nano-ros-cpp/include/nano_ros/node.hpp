#pragma once

/// @file node.hpp
/// @brief Node class for nano-ros

#include "nano_ros/publisher.hpp"
#include "nano_ros/qos.hpp"
#include "nano_ros/subscription.hpp"
#include "nano_ros/time.hpp"

#include <functional>
#include <memory>
#include <string>

namespace nano_ros {

// Forward declarations
class Context;
class Clock;

namespace ffi {
struct RustNode;
}  // namespace ffi

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

    /// @brief Get the node's clock
    /// @return Shared pointer to the node's clock
    std::shared_ptr<Clock> get_clock() const;

    /// @brief Get the current time from the node's clock
    /// @return Current time
    ///
    /// This is a convenience method equivalent to get_clock()->now()
    Time now() const;

    // =========================================================================
    // Publisher creation
    // =========================================================================

    /// @brief Create a publisher for a topic
    /// @param topic The topic name
    /// @param qos Quality of Service settings
    /// @return Shared pointer to the publisher
    /// @throws std::runtime_error if creation fails
    ///
    /// Example:
    /// @code
    /// auto pub = node->create_publisher("/chatter", QoS(10));
    /// @endcode
    std::shared_ptr<Publisher> create_publisher(const std::string& topic, const QoS& qos);

    /// @brief Create a typed publisher for a topic
    /// @tparam MessageT The message type
    /// @param topic The topic name
    /// @param qos Quality of Service settings
    /// @return Shared pointer to the typed publisher
    /// @throws std::runtime_error if creation fails
    ///
    /// Example:
    /// @code
    /// auto pub = node->create_publisher<std_msgs::Int32>("/chatter", QoS(10));
    /// pub->publish(msg);
    /// @endcode
    template <typename MessageT>
    std::shared_ptr<TypedPublisher<MessageT>> create_publisher(const std::string& topic,
                                                               const QoS& qos) {
        auto raw_pub = create_publisher(topic, qos);
        return std::make_shared<TypedPublisher<MessageT>>(std::move(raw_pub));
    }

    // =========================================================================
    // Subscription creation
    // =========================================================================

    /// @brief Create a subscription for a topic
    /// @param topic The topic name
    /// @param qos Quality of Service settings
    /// @return Shared pointer to the subscription
    /// @throws std::runtime_error if creation fails
    ///
    /// Example:
    /// @code
    /// auto sub = node->create_subscription("/chatter", QoS(10));
    /// auto raw_data = sub->take_raw();
    /// @endcode
    std::shared_ptr<Subscription> create_subscription(const std::string& topic, const QoS& qos);

    /// @brief Create a typed subscription for a topic
    /// @tparam MessageT The message type
    /// @param topic The topic name
    /// @param qos Quality of Service settings
    /// @return Shared pointer to the typed subscription
    /// @throws std::runtime_error if creation fails
    ///
    /// Example:
    /// @code
    /// auto sub = node->create_subscription<std_msgs::Int32>("/chatter", QoS(10));
    /// while (auto msg = sub->take()) {
    ///     std::cout << msg->data << std::endl;
    /// }
    /// @endcode
    template <typename MessageT>
    std::shared_ptr<TypedSubscription<MessageT>> create_subscription(const std::string& topic,
                                                                     const QoS& qos) {
        auto raw_sub = create_subscription(topic, qos);
        return std::make_shared<TypedSubscription<MessageT>>(std::move(raw_sub));
    }

    /// @brief Create a typed subscription with a callback
    /// @tparam MessageT The message type
    /// @param topic The topic name
    /// @param qos Quality of Service settings
    /// @param callback Function to call when a message is received
    /// @return Shared pointer to the typed subscription
    /// @throws std::runtime_error if creation fails
    ///
    /// Example:
    /// @code
    /// auto sub = node->create_subscription<std_msgs::Int32>(
    ///     "/chatter", QoS(10),
    ///     [](const std_msgs::Int32& msg) {
    ///         std::cout << msg.data << std::endl;
    ///     });
    /// @endcode
    template <typename MessageT>
    std::shared_ptr<TypedSubscription<MessageT>> create_subscription(
        const std::string& topic, const QoS& qos, std::function<void(const MessageT&)> callback) {
        auto raw_sub = create_subscription(topic, qos);
        return std::make_shared<TypedSubscription<MessageT>>(std::move(raw_sub),
                                                             std::move(callback));
    }

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

}  // namespace nano_ros
