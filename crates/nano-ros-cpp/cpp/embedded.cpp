/// @file embedded.cpp
/// @brief Embedded system implementation for nano-ros C++ bindings
///
/// This file provides the implementation for embedded systems that:
/// - Uses static allocation (no heap in C++ side)
/// - Calls into Rust FFI for actual functionality
/// - Returns error codes instead of throwing exceptions

#include <cstring>

#include <nano_ros/embedded/node.hpp>

// =============================================================================
// FFI declarations - implemented in Rust
// =============================================================================

extern "C" {

// Context FFI
void* nano_ros_context_create(const char* locator, size_t locator_len, uint32_t domain_id,
                              int32_t* error_out);
void nano_ros_context_destroy(void* ctx);
int32_t nano_ros_context_is_open(void* ctx);
uint32_t nano_ros_context_domain_id(void* ctx);
uint32_t nano_ros_context_poll(void* ctx, uint32_t timeout_ms);

// Node FFI
void* nano_ros_node_create(void* ctx, const char* name, size_t name_len, const char* ns,
                           size_t ns_len, int32_t* error_out);
void nano_ros_node_destroy(void* node);

// Publisher FFI
int32_t nano_ros_publisher_create(void* node, const char* topic, size_t topic_len,
                                  const char* type_name, size_t type_name_len, uint8_t reliability,
                                  uint8_t durability, int32_t history_depth);
int32_t nano_ros_publish(void* node, int32_t pub_handle, const uint8_t* data, size_t len);

// Subscriber FFI
int32_t nano_ros_subscriber_create(void* node, const char* topic, size_t topic_len,
                                   const char* type_name, size_t type_name_len, uint8_t reliability,
                                   uint8_t durability, int32_t history_depth);
int32_t nano_ros_try_receive(void* node, int32_t sub_handle, uint8_t* buffer, size_t buffer_size);

// Service FFI
int32_t nano_ros_service_create(void* node, const char* service_name, size_t service_name_len,
                                const char* type_name, size_t type_name_len);
int32_t nano_ros_service_try_receive_request(void* node, int32_t srv_handle, uint8_t* buffer,
                                             size_t buffer_size);
int32_t nano_ros_service_send_reply(void* node, int32_t srv_handle, const uint8_t* data,
                                    size_t len);

}  // extern "C"

namespace nano_ros {

// =============================================================================
// StringView implementation (freestanding only)
// =============================================================================

#if defined(NANO_ROS_EMBEDDED) && !defined(NANO_ROS_USE_ETL)
StringView::StringView(const char* str) : data_(str), size_(0) {
    if (str != nullptr) {
        while (str[size_] != '\0') {
            ++size_;
        }
    }
}
#endif

// =============================================================================
// EmbeddedContext implementation
// =============================================================================

ErrorCode EmbeddedContext::init(void* storage, size_t storage_size, StringView locator) {
    if (storage == nullptr || storage_size < sizeof(EmbeddedContext)) {
        return ErrorCode::BufferTooSmall;
    }

    // Check alignment
    if (reinterpret_cast<uintptr_t>(storage) % alignof(EmbeddedContext) != 0) {
        return ErrorCode::InvalidArgument;
    }

    auto* ctx = new (storage) EmbeddedContext();

    int32_t error = 0;
    ctx->rust_context_ = nano_ros_context_create(locator.data(), locator.size(), 0, &error);

    if (error != 0 || ctx->rust_context_ == nullptr) {
        ctx->is_open_ = false;
        return static_cast<ErrorCode>(error != 0 ? error : -1);
    }

    ctx->domain_id_ = nano_ros_context_domain_id(ctx->rust_context_);
    ctx->is_open_ = true;

    return ErrorCode::Ok;
}

ErrorCode EmbeddedContext::init_from_env(void* storage, size_t storage_size) {
    // Default to localhost for now
    // In a full implementation, this would read from environment variables
    return init(storage, storage_size, "tcp/127.0.0.1:7447");
}

void EmbeddedContext::close() {
    if (rust_context_ != nullptr) {
        nano_ros_context_destroy(rust_context_);
        rust_context_ = nullptr;
    }
    is_open_ = false;
}

bool EmbeddedContext::is_open() const {
    return is_open_ && rust_context_ != nullptr;
}

uint32_t EmbeddedContext::domain_id() const {
    return domain_id_;
}

uint32_t EmbeddedContext::poll(uint32_t timeout_ms) {
    if (rust_context_ == nullptr) {
        return 0;
    }
    return nano_ros_context_poll(rust_context_, timeout_ms);
}

// =============================================================================
// EmbeddedNode implementation
// =============================================================================

ErrorCode EmbeddedNode::create(void* storage, size_t storage_size, EmbeddedContext* context,
                               StringView name, StringView ns) {
    if (storage == nullptr || storage_size < sizeof(EmbeddedNode)) {
        return ErrorCode::BufferTooSmall;
    }

    if (context == nullptr || !context->is_open()) {
        return ErrorCode::NotInitialized;
    }

    // Check alignment
    if (reinterpret_cast<uintptr_t>(storage) % alignof(EmbeddedNode) != 0) {
        return ErrorCode::InvalidArgument;
    }

    auto* node = new (storage) EmbeddedNode();

    // Store names
    node->name_str_.assign(name.data(), name.size());
    node->ns_str_.assign(ns.data(), ns.size());
    node->context_ = context;

    int32_t error = 0;
    node->rust_node_ = nano_ros_node_create(context->rust_context_, name.data(), name.size(),
                                            ns.data(), ns.size(), &error);

    if (error != 0 || node->rust_node_ == nullptr) {
        return static_cast<ErrorCode>(error != 0 ? error : -1);
    }

    return ErrorCode::Ok;
}

StringView EmbeddedNode::name() const {
    return StringView(name_str_.data(), name_str_.size());
}

StringView EmbeddedNode::namespace_() const {
    return StringView(ns_str_.data(), ns_str_.size());
}

// =============================================================================
// Publisher API
// =============================================================================

PublisherHandle EmbeddedNode::create_publisher(StringView topic, StringView type_name,
                                               const QoS& qos) {
    if (rust_node_ == nullptr) {
        return INVALID_HANDLE;
    }

    uint8_t reliability = (qos.get_reliability() == ReliabilityPolicy::Reliable) ? 1 : 0;
    uint8_t durability = (qos.get_durability() == DurabilityPolicy::TransientLocal) ? 1 : 0;
    auto depth = static_cast<int32_t>(qos.get_depth());

    return nano_ros_publisher_create(rust_node_, topic.data(), topic.size(), type_name.data(),
                                     type_name.size(), reliability, durability, depth);
}

ErrorCode EmbeddedNode::publish(PublisherHandle handle, const uint8_t* data, size_t len) {
    if (rust_node_ == nullptr || handle < 0) {
        return ErrorCode::InvalidArgument;
    }

    int32_t result = nano_ros_publish(rust_node_, handle, data, len);
    return (result == 0) ? ErrorCode::Ok : static_cast<ErrorCode>(result);
}

StringView EmbeddedNode::get_publisher_topic(PublisherHandle /*handle*/) const {
    // TODO: Implement topic name retrieval from Rust side
    return StringView(nullptr, 0);
}

// =============================================================================
// Subscriber API
// =============================================================================

SubscriberHandle EmbeddedNode::create_subscriber(StringView topic, StringView type_name,
                                                 const QoS& qos) {
    if (rust_node_ == nullptr) {
        return INVALID_HANDLE;
    }

    uint8_t reliability = (qos.get_reliability() == ReliabilityPolicy::Reliable) ? 1 : 0;
    uint8_t durability = (qos.get_durability() == DurabilityPolicy::TransientLocal) ? 1 : 0;
    auto depth = static_cast<int32_t>(qos.get_depth());

    return nano_ros_subscriber_create(rust_node_, topic.data(), topic.size(), type_name.data(),
                                      type_name.size(), reliability, durability, depth);
}

int32_t EmbeddedNode::try_receive(SubscriberHandle handle, uint8_t* buffer, size_t buffer_size) {
    if (rust_node_ == nullptr || handle < 0 || buffer == nullptr) {
        return -1;
    }

    return nano_ros_try_receive(rust_node_, handle, buffer, buffer_size);
}

StringView EmbeddedNode::get_subscriber_topic(SubscriberHandle /*handle*/) const {
    // TODO: Implement topic name retrieval from Rust side
    return StringView(nullptr, 0);
}

// =============================================================================
// Service Server API
// =============================================================================

ServiceServerHandle EmbeddedNode::create_service(StringView service_name, StringView type_name) {
    if (rust_node_ == nullptr) {
        return INVALID_HANDLE;
    }

    return nano_ros_service_create(rust_node_, service_name.data(), service_name.size(),
                                   type_name.data(), type_name.size());
}

int32_t EmbeddedNode::try_receive_request(ServiceServerHandle handle, uint8_t* buffer,
                                          size_t buffer_size) {
    if (rust_node_ == nullptr || handle < 0 || buffer == nullptr) {
        return -1;
    }

    return nano_ros_service_try_receive_request(rust_node_, handle, buffer, buffer_size);
}

ErrorCode EmbeddedNode::send_reply(ServiceServerHandle handle, const uint8_t* data, size_t len) {
    if (rust_node_ == nullptr || handle < 0) {
        return ErrorCode::InvalidArgument;
    }

    int32_t result = nano_ros_service_send_reply(rust_node_, handle, data, len);
    return (result == 0) ? ErrorCode::Ok : static_cast<ErrorCode>(result);
}

}  // namespace nano_ros
