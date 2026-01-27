/// @file main.cpp
/// @brief Embedded C++ talker example for nano-ros
///
/// This example demonstrates the embedded-compatible API that works
/// without std library (no heap allocation in C++, no exceptions).
///
/// Build for desktop testing:
///   mkdir build && cd build
///   cmake .. && make
///   ./cpp_embedded_talker
///
/// Build for embedded:
///   mkdir build && cd build
///   cmake .. -DBUILD_FOR_EMBEDDED=ON
///   make

#if defined(NANO_ROS_EMBEDDED)
// Embedded mode: use embedded headers
#include <nano_ros/embedded/node.hpp>
#include <std_msgs/msg/int32.hpp>  // Generated with --embedded flag
#else
// Desktop mode: use standard headers
#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#endif

#if defined(NANO_ROS_EMBEDDED)

// ============================================================================
// Embedded implementation (no std library)
// ============================================================================

// Static storage for context and node
alignas(nano_ros::EmbeddedContext) static uint8_t g_ctx_storage[sizeof(nano_ros::EmbeddedContext)];
alignas(nano_ros::EmbeddedNode) static uint8_t g_node_storage[sizeof(nano_ros::EmbeddedNode)];

// Buffer for serialization
static uint8_t g_pub_buffer[256];

// Platform-specific functions (implement for your platform)
extern "C" {
void platform_sleep_ms(uint32_t ms);
uint32_t platform_get_time_ms();
void platform_print(const char* msg);
void platform_print_int(const char* prefix, int32_t value);
}

int main() {
    platform_print("nano-ros Embedded C++ Talker Example\n");

    // Initialize context
    auto err = nano_ros::EmbeddedContext::init(g_ctx_storage, sizeof(g_ctx_storage),
                                                "tcp/127.0.0.1:7447");
    if (err != nano_ros::ErrorCode::Ok) {
        platform_print("Failed to initialize context\n");
        return 1;
    }

    auto* ctx = reinterpret_cast<nano_ros::EmbeddedContext*>(g_ctx_storage);
    platform_print("Context initialized\n");

    // Create node
    err = nano_ros::EmbeddedNode::create(g_node_storage, sizeof(g_node_storage), ctx,
                                         "embedded_talker", "");
    if (err != nano_ros::ErrorCode::Ok) {
        platform_print("Failed to create node\n");
        ctx->close();
        return 1;
    }

    auto* node = reinterpret_cast<nano_ros::EmbeddedNode*>(g_node_storage);
    platform_print("Node created\n");

    // Create publisher
    nano_ros::QoS qos(10);
    auto pub_handle = node->create_publisher("/chatter", "std_msgs/msg/Int32", qos);
    if (pub_handle < 0) {
        platform_print("Failed to create publisher\n");
        ctx->close();
        return 1;
    }

    platform_print("Publisher created, starting publish loop...\n");

    // Create typed publisher wrapper
    nano_ros::EmbeddedTypedPublisher<std_msgs::msg::Int32, 256> pub(node, pub_handle, g_pub_buffer);

    // Publish loop
    int32_t count = 0;
    while (true) {
        // Poll transport
        ctx->poll(0);

        // Publish message
        std_msgs::msg::Int32 msg;
        msg.data = count++;

        err = pub.publish(msg);
        if (err == nano_ros::ErrorCode::Ok) {
            platform_print_int("Published: ", msg.data);
        } else {
            platform_print("Publish failed\n");
        }

        // Sleep
        platform_sleep_ms(1000);
    }

    ctx->close();
    return 0;
}

// Default platform implementations for desktop testing
// (These would be replaced by actual platform implementations on embedded)
#if !defined(PLATFORM_ZEPHYR) && !defined(PLATFORM_NUTTX)
#include <stdio.h>
#include <unistd.h>

void platform_sleep_ms(uint32_t ms) {
    usleep(ms * 1000);
}

uint32_t platform_get_time_ms() {
    // Simple implementation - not accurate
    static uint32_t time = 0;
    return time++;
}

void platform_print(const char* msg) {
    printf("%s", msg);
    fflush(stdout);
}

void platform_print_int(const char* prefix, int32_t value) {
    printf("%s%d\n", prefix, value);
    fflush(stdout);
}
#endif

#else

// ============================================================================
// Desktop implementation (with std library)
// ============================================================================

int main() {
    std::cout << "nano-ros C++ Talker Example (Desktop Mode)\n";

    // Initialize context
    auto ctx = nano_ros::Context::from_env();
    if (!ctx) {
        std::cerr << "Failed to create context\n";
        return 1;
    }
    std::cout << "Context initialized on domain " << ctx->domain_id() << "\n";

    // Create node
    auto node = ctx->create_node("cpp_talker", "");
    std::cout << "Node '" << node->get_name() << "' created\n";

    // Create publisher
    auto pub = node->create_publisher<std_msgs::msg::Int32>("/chatter", nano_ros::QoS(10));
    std::cout << "Publisher on '/chatter' created\n";

    // Publish loop
    int32_t count = 0;
    while (true) {
        std_msgs::msg::Int32 msg;
        msg.data = count++;

        pub->publish(msg);
        std::cout << "Published: " << msg.data << "\n";

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

#endif  // NANO_ROS_EMBEDDED
