// Zephyr C++ Fibonacci action client — typed callback component.
//
// `configure` binds member callbacks (goal-response / feedback / result) by
// identity via `bind_action_client`, then sends one goal. The acceptance +
// result arrive in the member callbacks, dispatched by the binding's
// poll-timer pump each spin tick.
#ifndef NROS_ZEPHYR_ACTION_CLIENT_CPP_FIBONACCICLIENT_HPP
#define NROS_ZEPHYR_ACTION_CLIENT_CPP_FIBONACCICLIENT_HPP

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

namespace zephyr_cpp_action_client {

class FibonacciClient {
    ::nros::ActionClientStorage client_;
    ::nros::Timer poll_timer_;
    int32_t order_ = 10;

    // Member callbacks, bound by identity (no naming).
    void on_goal_response(bool accepted, const uint8_t goal_id[16]);
    void on_feedback(const uint8_t goal_id[16], const uint8_t* data, size_t len);
    void on_result(const uint8_t goal_id[16], int32_t status, const uint8_t* data, size_t len);

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace zephyr_cpp_action_client

#endif
