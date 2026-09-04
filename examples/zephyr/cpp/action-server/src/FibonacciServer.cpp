/// @file FibonacciServer.cpp
/// @brief Zephyr C++ Fibonacci action server — typed component.

#include "FibonacciServer.hpp"

#include <cstdio>
#include <cstring>

namespace zephyr_cpp_action_server {

static int32_t read_i32_le(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

static void write_u32_le(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

// Goal callback: decode `int32 order`, stash the goal, accept-and-execute.
int32_t FibonacciServer::on_goal(const uint8_t goal_id[16], const uint8_t* data, size_t len) {
    // Goal CDR: 4-byte encapsulation header, then int32 order (offset 4).
    if (len < 8 || pending_) {
        return static_cast<int32_t>(::nros::GoalResponse::Reject);
    }
    std::memcpy(goal_id_, goal_id, 16);
    order_ = read_i32_le(data + 4);
    pending_ = true;
    std::printf("Received goal request with order %d\n", static_cast<int>(order_));
    return static_cast<int32_t>(::nros::GoalResponse::AcceptAndExecute);
}

// Cancel callback: this server does not support cancellation.
int32_t FibonacciServer::on_cancel(const uint8_t /*goal_id*/[16]) {
    return static_cast<int32_t>(::nros::CancelResponse::Reject);
}

// Timer: execute one pending goal (compute the sequence + complete the goal).
void FibonacciServer::on_tick() {
    if (!pending_) {
        return;
    }
    pending_ = false;
    std::printf("Executing goal\n");

    // Clamp the term count to the result buffer (encap + u32 len + N int32).
    int32_t n = order_;
    if (n < 0) {
        n = 0;
    }
    if (n > 16) {
        n = 16;
    }

    int32_t seq[16];
    for (int32_t i = 0; i < n; ++i) {
        if (i == 0) {
            seq[i] = 0;
        } else if (i == 1) {
            seq[i] = 1;
        } else {
            seq[i] = seq[i - 1] + seq[i - 2];
        }
    }

    // Result CDR: encapsulation header (CDR_LE) + u32 length + N int32.
    uint8_t buf[8 + 4 * 16];
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x00;
    write_u32_le(buf + 4, static_cast<uint32_t>(n));
    for (int32_t i = 0; i < n; ++i) {
        write_u32_le(buf + 8 + 4 * i, static_cast<uint32_t>(seq[i]));
    }
    size_t result_len = 8 + 4 * static_cast<size_t>(n);

    nros_cpp_ret_t rc = nros_cpp_action_server_complete_goal(
        storage_.bytes, executor_, reinterpret_cast<const uint8_t(*)[16]>(goal_id_),
        static_cast<int32_t>(nros::GoalStatus::Succeeded), buf, result_len);
    if (rc == 0) {
        std::printf("Goal succeeded\n");
    } else {
        std::printf("Failed to complete goal (rc=%d)\n", static_cast<int>(rc));
    }
}

::rclcpp::Result FibonacciServer::configure(::nros::Node& node) {
    // Unbuffered stdout — a full-buffered console can swallow the final
    // line(s) when the harness kills the QEMU before a flush.
    // `::setvbuf` (global) not `std::setvbuf` — Zephyr's minimal libcpp/picolibc
    // `<cstdio>` declares it in the global namespace only.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    executor_ = node.executor_handle();
    ::rclcpp::Result r = ::nros::bind_action_server_raw<FibonacciServer, &FibonacciServer::on_goal,
                                                        &FibonacciServer::on_cancel>(
        node, storage_.bytes, "/fibonacci", "example_interfaces/action/Fibonacci", this);
    if (!r.ok()) {
        return r;
    }
    r = ::nros::bind_timer<FibonacciServer, &FibonacciServer::on_tick>(node, timer_, 200, this);
    if (r.ok()) {
        // Readiness marker the e2e harness greps before sending a goal.
        std::printf("Waiting for action goals\n");
    }
    return r;
}

} // namespace zephyr_cpp_action_server
