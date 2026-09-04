// Zephyr C++ Fibonacci action server — TYPED component (RFC-0043 / phase-244.C2).
//
// `configure` binds the member goal/cancel callbacks (by identity) as a raw
// action server on `/fibonacci`, and a timer that drives goal execution. The
// real handler decodes the CDR goal (int32 order), computes the sequence, and
// completes the goal with a CDR result (int32[] sequence). No interpreter.
#ifndef NROS_ZEPHYR_ACTION_SERVER_CPP_FIBONACCISERVER_HPP
#define NROS_ZEPHYR_ACTION_SERVER_CPP_FIBONACCISERVER_HPP

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

namespace zephyr_cpp_action_server {

class FibonacciServer {
    ::nros::ActionServerStorage storage_; // executor-arena-held action server
    ::nros::Timer timer_;                 // drives accepted-goal execution
    void* executor_ = nullptr;            // stashed for complete_goal

    // One in-flight goal (this server runs goals one at a time).
    bool pending_ = false;
    uint8_t goal_id_[16] = {};
    int32_t order_ = 0;

    // Goal callback (by identity): stash the goal + accept-and-execute.
    int32_t on_goal(const uint8_t goal_id[16], const uint8_t* data, size_t len);
    // Cancel callback (by identity): this server does not support cancel.
    int32_t on_cancel(const uint8_t goal_id[16]);
    // Timer: execute a pending goal (compute + complete).
    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace zephyr_cpp_action_server

#endif
