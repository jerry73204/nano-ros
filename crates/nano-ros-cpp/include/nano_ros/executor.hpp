#pragma once

/// @file executor.hpp
/// @brief Executor classes for nano-ros callback processing

#include "nano_ros/node.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace nano_ros {

// Forward declarations
class Context;

namespace ffi {
struct RustSingleThreadedExecutor;
struct RustPollingExecutor;
}  // namespace ffi

// =============================================================================
// ExecutorOptions
// =============================================================================

/// @brief Options for executor creation
struct ExecutorOptions {
    std::shared_ptr<Context> context;

    ExecutorOptions() = default;
    explicit ExecutorOptions(std::shared_ptr<Context> ctx) : context(std::move(ctx)) {}
};

// =============================================================================
// SingleThreadedExecutor - Desktop/std platforms (rclcpp-compatible)
// =============================================================================

/// @brief Single-threaded executor with blocking spin (requires std)
///
/// This executor provides an rclcpp-compatible API with blocking spin methods.
/// Use this on desktop platforms where std::thread is available.
///
/// Example:
/// @code
/// auto ctx = nano_ros::Context::from_env();
/// auto node = ctx->create_node("my_node");
/// auto sub = node->create_subscription<std_msgs::msg::Int32>("/topic", QoS(10),
///     [](const std_msgs::msg::Int32& msg) {
///         std::cout << "Received: " << msg.data << std::endl;
///     });
///
/// nano_ros::SingleThreadedExecutor executor;
/// executor.add_node(node);
/// executor.spin();  // Blocks until cancel() is called
/// @endcode
class SingleThreadedExecutor {
public:
    /// @brief Construct an executor with default options
    SingleThreadedExecutor();

    /// @brief Construct an executor with explicit options
    explicit SingleThreadedExecutor(const ExecutorOptions& options);

    ~SingleThreadedExecutor();

    // Non-copyable
    SingleThreadedExecutor(const SingleThreadedExecutor&) = delete;
    SingleThreadedExecutor& operator=(const SingleThreadedExecutor&) = delete;

    // Movable
    SingleThreadedExecutor(SingleThreadedExecutor&&) noexcept;
    SingleThreadedExecutor& operator=(SingleThreadedExecutor&&) noexcept;

    /// @brief Add a node to this executor
    /// @param node The node to add
    void add_node(std::shared_ptr<Node> node);

    /// @brief Remove a node from this executor
    /// @param node The node to remove
    void remove_node(std::shared_ptr<Node> node);

    /// @brief Blocking spin loop - processes work until cancel() is called
    void spin();

    /// @brief Execute work with optional timeout (rclcpp-compatible)
    ///
    /// @param timeout_ns Timeout in nanoseconds:
    ///   - < 0: Block until work is available (infinite timeout)
    ///   - == 0: Non-blocking, only process immediately available work
    ///   - > 0: Wait up to timeout_ns for work
    /// @return Number of callbacks executed
    uint32_t spin_once(int64_t timeout_ns = -1);

    /// @brief Execute all immediately available work (non-blocking)
    /// @param max_duration_ns Maximum duration to spend (0 = no limit)
    /// @return Number of callbacks executed
    uint32_t spin_some(int64_t max_duration_ns = 0);

    /// @brief Request the executor to stop spinning (thread-safe)
    void cancel();

    /// @brief Check if the executor is not cancelled
    /// @return true if the executor is still spinning
    bool is_spinning() const;

    /// @brief Get the number of nodes in this executor
    size_t node_count() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// PollingExecutor - Embedded platforms (Zephyr, NuttX, bare-metal)
// =============================================================================

/// @brief Polling executor for embedded platforms (no threads, no blocking)
///
/// This executor is designed for embedded systems where threads are not
/// available or not desired. The user must call spin_once() periodically
/// from their main loop or RTOS task.
///
/// Example (Zephyr):
/// @code
/// auto ctx = nano_ros::Context::from_env();
/// auto node = ctx->create_node("zephyr_node");
/// auto sub = node->create_subscription<std_msgs::msg::Int32>("/topic", QoS(10),
///     on_message);  // function pointer
///
/// nano_ros::PollingExecutor executor;
/// executor.add_node(node);
///
/// while (true) {
///     executor.spin_once(10);  // 10ms since last call
///     k_msleep(10);
/// }
/// @endcode
class PollingExecutor {
public:
    /// @brief Construct an executor with default options
    PollingExecutor();

    /// @brief Construct an executor with explicit options
    explicit PollingExecutor(const ExecutorOptions& options);

    ~PollingExecutor();

    // Non-copyable
    PollingExecutor(const PollingExecutor&) = delete;
    PollingExecutor& operator=(const PollingExecutor&) = delete;

    // Movable
    PollingExecutor(PollingExecutor&&) noexcept;
    PollingExecutor& operator=(PollingExecutor&&) noexcept;

    /// @brief Add a node to this executor
    /// @param node The node to add
    void add_node(std::shared_ptr<Node> node);

    /// @brief Remove a node from this executor
    /// @param node The node to remove
    void remove_node(std::shared_ptr<Node> node);

    /// @brief Process all available work (non-blocking)
    ///
    /// @param delta_ms Milliseconds elapsed since last call (for timer processing)
    /// @return Number of callbacks executed
    ///
    /// This method:
    /// 1. Polls transport for incoming messages
    /// 2. Invokes subscription callbacks for received messages
    /// 3. Fires ready timers based on delta_ms
    uint32_t spin_once(uint32_t delta_ms);

    /// @brief Get the number of nodes in this executor
    size_t node_count() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// Convenience functions (match rclcpp:: namespace)
// =============================================================================

/// @brief Spin a single node until cancelled
///
/// Creates a SingleThreadedExecutor, adds the node, and calls spin().
///
/// @param node The node to spin
void spin(std::shared_ptr<Node> node);

/// @brief Process all immediately available work for a single node
///
/// Creates a SingleThreadedExecutor, adds the node, and calls spin_some().
///
/// @param node The node to process
void spin_some(std::shared_ptr<Node> node);

}  // namespace nano_ros
