#pragma once

/// @file publisher.hpp
/// @brief Publisher class for nano-ros

#include "nano_ros/cdr.hpp"
#include "nano_ros/qos.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nano_ros {

// Forward declarations
class Node;

namespace ffi {
struct RustPublisher;
}  // namespace ffi

/// @brief A publisher that can send messages on a topic
///
/// Publishers are created via Node::create_publisher() and should not be
/// constructed directly.
///
/// For typed publishing, use the templated publish() method with a message type.
/// For raw CDR byte publishing, use publish_raw().
///
/// Example:
/// @code
/// auto publisher = node->create_publisher<std_msgs::Int32>("/chatter", QoS(10));
/// std_msgs::Int32 msg;
/// msg.data = 42;
/// publisher->publish(msg);
/// @endcode
class Publisher {
public:
    /// @brief Publish raw CDR-serialized bytes
    /// @param data The serialized message data
    /// @throws std::runtime_error if publish fails
    void publish_raw(const std::vector<uint8_t>& data);

    /// @brief Publish raw CDR-serialized bytes
    /// @param data Pointer to the serialized message data
    /// @param size Size of the data in bytes
    /// @throws std::runtime_error if publish fails
    void publish_raw(const uint8_t* data, size_t size);

    /// @brief Get the topic name
    /// @return The topic this publisher publishes to
    std::string get_topic_name() const;

    ~Publisher();

    // Non-copyable
    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    // Movable
    Publisher(Publisher&&) noexcept;
    Publisher& operator=(Publisher&&) noexcept;

private:
    friend class Node;

    Publisher();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// @brief A typed publisher wrapper
///
/// This template wraps a Publisher and provides type-safe publishing
/// by serializing messages to CDR format.
///
/// Example:
/// @code
/// auto publisher = node->create_publisher<std_msgs::Int32>("/chatter", QoS(10));
/// std_msgs::Int32 msg;
/// msg.data = 42;
/// publisher->publish(msg);
/// @endcode
template <typename MessageT>
class TypedPublisher {
public:
    /// @brief Construct from a Publisher
    explicit TypedPublisher(std::shared_ptr<Publisher> pub) : inner_(std::move(pub)) {}

    /// @brief Publish a typed message
    /// @param msg The message to publish
    /// @throws std::runtime_error if serialization or publish fails
    void publish(const MessageT& msg) {
        CdrWriter writer;
        writer.write_encapsulation();
        msg.serialize(writer);
        inner_->publish_raw(writer.data(), writer.size());
    }

    /// @brief Get the topic name
    std::string get_topic_name() const {
        return inner_->get_topic_name();
    }

    /// @brief Get the underlying Publisher
    std::shared_ptr<Publisher> get_publisher() const {
        return inner_;
    }

private:
    std::shared_ptr<Publisher> inner_;
};

}  // namespace nano_ros
