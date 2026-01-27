#pragma once

/// @file node.hpp
/// @brief Embedded-compatible Node API
///
/// This header provides a Node API for embedded systems that:
/// - Uses static allocation (no heap)
/// - Returns error codes instead of throwing exceptions
/// - Uses handles instead of smart pointers

#include <nano_ros/embedded/cdr.hpp>
#include <nano_ros/embedded/types.hpp>
#include <nano_ros/qos.hpp>

namespace nano_ros {

// Forward declarations
struct EmbeddedContext;
struct EmbeddedNode;
struct EmbeddedPublisher;
struct EmbeddedSubscriber;

// Handle types (opaque integers)
using ContextHandle = int32_t;
using NodeHandle = int32_t;
using PublisherHandle = int32_t;
using SubscriberHandle = int32_t;
using ServiceServerHandle = int32_t;
using ServiceClientHandle = int32_t;

/// Invalid handle value
constexpr int32_t INVALID_HANDLE = -1;

// =============================================================================
// EmbeddedContext - Session management
// =============================================================================

/// Embedded context for managing the transport session
///
/// The context must be initialized before creating nodes. On embedded systems,
/// this sets up the zenoh-pico session with the transport backend.
///
/// Example:
/// @code
/// // Static storage for context
/// alignas(EmbeddedContext) uint8_t ctx_storage[sizeof(EmbeddedContext)];
///
/// auto err = EmbeddedContext::init(ctx_storage, sizeof(ctx_storage));
/// if (err != ErrorCode::Ok) {
///     // Handle error
/// }
///
/// auto* ctx = reinterpret_cast<EmbeddedContext*>(ctx_storage);
/// @endcode
class EmbeddedContext {
public:
    /// Initialize context with static storage
    /// @param storage Pointer to storage buffer (must be aligned)
    /// @param storage_size Size of storage buffer
    /// @param locator Zenoh locator (e.g., "tcp/192.168.1.1:7447")
    /// @return Error code
    static ErrorCode init(void* storage, size_t storage_size,
                          StringView locator = "tcp/127.0.0.1:7447");

    /// Initialize context from environment variables
    /// Uses ZENOH_LOCATOR, ROS_DOMAIN_ID, etc.
    static ErrorCode init_from_env(void* storage, size_t storage_size);

    /// Close context and release resources
    void close();

    /// Check if context is open and valid
    bool is_open() const;

    /// Get domain ID
    uint32_t domain_id() const;

    /// Poll the transport for incoming data
    /// Must be called periodically on embedded systems
    /// @param timeout_ms Maximum time to wait (0 = non-blocking)
    /// @return Number of events processed
    uint32_t poll(uint32_t timeout_ms = 0);

    /// Get required storage size
    static constexpr size_t required_storage_size() {
        return sizeof(EmbeddedContext);
    }

private:
    friend class EmbeddedNode;

    EmbeddedContext() = default;

    // Opaque pointer to Rust context
    void* rust_context_;
    uint32_t domain_id_;
    bool is_open_;
};

// =============================================================================
// EmbeddedNode - Node for pub/sub/services
// =============================================================================

/// Embedded-compatible Node
///
/// Nodes are created from a context and provide pub/sub functionality.
/// All memory is statically allocated.
///
/// Example:
/// @code
/// alignas(EmbeddedNode) uint8_t node_storage[EmbeddedNode::required_storage_size()];
///
/// auto err = EmbeddedNode::create(node_storage, sizeof(node_storage),
///                                  ctx, "my_node", "");
/// if (err != ErrorCode::Ok) {
///     // Handle error
/// }
///
/// auto* node = reinterpret_cast<EmbeddedNode*>(node_storage);
/// @endcode
class EmbeddedNode {
public:
    /// Create a node with static storage
    /// @param storage Pointer to storage buffer
    /// @param storage_size Size of storage buffer
    /// @param context Pointer to initialized context
    /// @param name Node name
    /// @param ns Node namespace (can be empty)
    static ErrorCode create(void* storage, size_t storage_size, EmbeddedContext* context,
                            StringView name, StringView ns = "");

    /// Get node name
    StringView name() const;

    /// Get node namespace
    StringView namespace_() const;

    // =========================================================================
    // Publisher API
    // =========================================================================

    /// Create a publisher
    /// @param topic Topic name (e.g., "/chatter")
    /// @param type_name ROS type name (e.g., "std_msgs/msg/Int32")
    /// @param qos Quality of service settings
    /// @return Publisher handle or negative error code
    PublisherHandle create_publisher(StringView topic, StringView type_name, const QoS& qos);

    /// Publish raw CDR data
    /// @param handle Publisher handle
    /// @param data Pointer to CDR-serialized data (with encapsulation header)
    /// @param len Length of data in bytes
    /// @return Error code
    ErrorCode publish(PublisherHandle handle, const uint8_t* data, size_t len);

    /// Get topic name for publisher
    StringView get_publisher_topic(PublisherHandle handle) const;

    // =========================================================================
    // Subscriber API
    // =========================================================================

    /// Create a subscriber
    /// @param topic Topic name
    /// @param type_name ROS type name
    /// @param qos Quality of service settings
    /// @return Subscriber handle or negative error code
    SubscriberHandle create_subscriber(StringView topic, StringView type_name, const QoS& qos);

    /// Try to receive data (non-blocking)
    /// @param handle Subscriber handle
    /// @param buffer Buffer to receive data
    /// @param buffer_size Size of buffer
    /// @return Number of bytes received, 0 if no data, negative on error
    int32_t try_receive(SubscriberHandle handle, uint8_t* buffer, size_t buffer_size);

    /// Get topic name for subscriber
    StringView get_subscriber_topic(SubscriberHandle handle) const;

    // =========================================================================
    // Service Server API
    // =========================================================================

    /// Create a service server
    /// @param service_name Service name (e.g., "/add_two_ints")
    /// @param type_name ROS service type name
    /// @return Service server handle or negative error code
    ServiceServerHandle create_service(StringView service_name, StringView type_name);

    /// Try to receive a service request (non-blocking)
    /// @param handle Service server handle
    /// @param buffer Buffer to receive request
    /// @param buffer_size Size of buffer
    /// @return Number of bytes received, 0 if no request, negative on error
    int32_t try_receive_request(ServiceServerHandle handle, uint8_t* buffer, size_t buffer_size);

    /// Send a service reply
    /// @param handle Service server handle
    /// @param data Reply data (CDR-serialized)
    /// @param len Length of data
    /// @return Error code
    ErrorCode send_reply(ServiceServerHandle handle, const uint8_t* data, size_t len);

    // =========================================================================
    // Utility
    // =========================================================================

    /// Get required storage size
    static constexpr size_t required_storage_size() {
        return sizeof(EmbeddedNode);
    }

private:
    EmbeddedNode() = default;

    // Storage for names (note: ns_str_ avoids collision with namespace_() method)
    String<EmbeddedConfig::MAX_NODE_NAME> name_str_;
    String<EmbeddedConfig::MAX_NAMESPACE> ns_str_;

    // Opaque pointer to Rust node
    void* rust_node_ = nullptr;

    // Context reference
    EmbeddedContext* context_ = nullptr;
};

// =============================================================================
// Typed Publisher Wrapper
// =============================================================================

/// Type-safe publisher for embedded systems
///
/// Wraps EmbeddedNode::publish() with type-safe serialization.
///
/// Example:
/// @code
/// struct MyMessage {
///     int32_t value;
///     void serialize(EmbeddedCdrWriter& w) const { w.write_i32(value); }
/// };
///
/// uint8_t buffer[256];
/// EmbeddedTypedPublisher<MyMessage, 256> pub(node, handle, buffer);
/// MyMessage msg{42};
/// auto err = pub.publish(msg);
/// @endcode
template <typename MessageT, size_t BufferSize = EmbeddedConfig::DEFAULT_BUFFER_SIZE>
class EmbeddedTypedPublisher {
public:
    EmbeddedTypedPublisher(EmbeddedNode* node, PublisherHandle handle, uint8_t* buffer)
        : node_(node), handle_(handle), buffer_(buffer) {}

    /// Publish a typed message
    ErrorCode publish(const MessageT& msg) {
        EmbeddedCdrWriter writer(buffer_, BufferSize);
        writer.write_encapsulation();
        msg.serialize(writer);

        if (writer.has_error()) {
            return writer.error();
        }

        return node_->publish(handle_, writer.data(), writer.size());
    }

    /// Get publisher handle
    PublisherHandle handle() const {
        return handle_;
    }

private:
    EmbeddedNode* node_;
    PublisherHandle handle_;
    uint8_t* buffer_;
};

// =============================================================================
// Typed Subscriber Wrapper
// =============================================================================

/// Type-safe subscriber for embedded systems
///
/// Wraps EmbeddedNode::try_receive() with type-safe deserialization.
template <typename MessageT, size_t BufferSize = EmbeddedConfig::DEFAULT_BUFFER_SIZE>
class EmbeddedTypedSubscriber {
public:
    EmbeddedTypedSubscriber(EmbeddedNode* node, SubscriberHandle handle, uint8_t* buffer)
        : node_(node), handle_(handle), buffer_(buffer) {}

    /// Try to receive a typed message (non-blocking)
    /// @param msg Output message
    /// @return true if message was received, false if no data
    bool try_receive(MessageT& msg) {
        int32_t len = node_->try_receive(handle_, buffer_, BufferSize);
        if (len <= 0) {
            return false;
        }

        EmbeddedCdrReader reader(buffer_, static_cast<size_t>(len));
        reader.read_encapsulation();
        msg.deserialize(reader);

        return reader.is_ok();
    }

    /// Get subscriber handle
    SubscriberHandle handle() const {
        return handle_;
    }

private:
    EmbeddedNode* node_;
    SubscriberHandle handle_;
    uint8_t* buffer_;
};

// =============================================================================
// Polling Executor for Embedded
// =============================================================================

/// Simple polling executor for embedded systems
///
/// Provides a main loop abstraction that polls the context and processes
/// callbacks. Does not use threads.
///
/// Example:
/// @code
/// EmbeddedPollingExecutor exec(&ctx);
/// exec.add_node(&node);
///
/// while (true) {
///     uint32_t delta_ms = get_elapsed_ms();
///     exec.spin_once(delta_ms);
///     platform_sleep_ms(10);
/// }
/// @endcode
class EmbeddedPollingExecutor {
public:
    explicit EmbeddedPollingExecutor(EmbeddedContext* context) : context_(context) {}

    /// Process available work
    /// @param delta_ms Milliseconds since last call (for timer processing)
    /// @return Number of callbacks executed
    uint32_t spin_once(uint32_t /*delta_ms*/) {
        // Poll transport
        uint32_t events = context_->poll(0);

        // TODO: Process timer callbacks based on delta_ms

        return events;
    }

private:
    EmbeddedContext* context_;
};

}  // namespace nano_ros
