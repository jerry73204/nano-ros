#pragma once

/// @file subscription.hpp
/// @brief Subscription class for nano-ros

#include "nano_ros/cdr.hpp"
#include "nano_ros/qos.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nano_ros {

// Forward declarations
class Node;

namespace ffi {
struct RustSubscriber;
}  // namespace ffi

/// @brief A subscription that can receive messages from a topic
///
/// Subscriptions are created via Node::create_subscription() and should not be
/// constructed directly.
///
/// For typed message receiving, use the templated take() method.
/// For raw CDR byte receiving, use take_raw().
///
/// Example:
/// @code
/// auto subscription = node->create_subscription<std_msgs::Int32>(
///     "/chatter", QoS(10),
///     [](const std_msgs::Int32& msg) {
///         std::cout << "Received: " << msg.data << std::endl;
///     });
/// @endcode
class Subscription {
public:
    /// @brief Take a raw message if available (non-blocking)
    /// @return The raw CDR-serialized message data, or empty vector if no message
    /// @throws std::runtime_error if receive fails
    std::vector<uint8_t> take_raw();

    /// @brief Check if there's a message available
    /// @return true if a message can be taken
    bool has_message();

    /// @brief Get the topic name
    /// @return The topic this subscription listens to
    std::string get_topic_name() const;

    ~Subscription();

    // Non-copyable
    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;

    // Movable
    Subscription(Subscription&&) noexcept;
    Subscription& operator=(Subscription&&) noexcept;

private:
    friend class Node;

    Subscription();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// @brief A typed subscription wrapper
///
/// This template wraps a Subscription and provides type-safe message receiving
/// by deserializing messages from CDR format.
///
/// Example:
/// @code
/// auto subscription = node->create_subscription<std_msgs::Int32>("/chatter", QoS(10));
/// while (auto msg = subscription->take()) {
///     std::cout << "Received: " << msg->data << std::endl;
/// }
/// @endcode
template <typename MessageT>
class TypedSubscription {
public:
    /// @brief Callback type for message reception
    using CallbackT = std::function<void(const MessageT&)>;

    /// @brief Construct from a Subscription
    explicit TypedSubscription(std::shared_ptr<Subscription> sub) : inner_(std::move(sub)) {}

    /// @brief Construct from a Subscription with callback
    TypedSubscription(std::shared_ptr<Subscription> sub, CallbackT callback)
        : inner_(std::move(sub)), callback_(std::move(callback)) {}

    /// @brief Take a typed message if available (non-blocking)
    /// @return The deserialized message, or std::nullopt if no message
    /// @throws std::runtime_error if receive or deserialization fails
    std::optional<MessageT> take() {
        auto raw = inner_->take_raw();
        if (raw.empty()) {
            return std::nullopt;
        }
        CdrReader reader(raw.data(), raw.size());
        reader.read_encapsulation();
        MessageT msg;
        msg.deserialize(reader);
        return msg;
    }

    /// @brief Process any available messages through the callback
    /// @return Number of messages processed
    size_t spin_once() {
        if (!callback_) {
            return 0;
        }
        size_t count = 0;
        while (auto msg = take()) {
            callback_(*msg);
            ++count;
        }
        return count;
    }

    /// @brief Get the topic name
    std::string get_topic_name() const {
        return inner_->get_topic_name();
    }

    /// @brief Get the underlying Subscription
    std::shared_ptr<Subscription> get_subscription() const {
        return inner_;
    }

    /// @brief Set the callback function
    void set_callback(CallbackT callback) {
        callback_ = std::move(callback);
    }

private:
    std::shared_ptr<Subscription> inner_;
    CallbackT callback_;
};

}  // namespace nano_ros
