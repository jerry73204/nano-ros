#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "example_interfaces.hpp"

namespace action_client_pkg {

/// FibClient — A4 actions (C++ projection of the Rust `action_client_pkg` / the C
/// `c_fib_client_pkg`). A typed component whose 500 ms timer drives the POLL-model action client
/// (`create_action_client_raw` + the raw `send_goal_async` / `get_result_async` / `poll` FFI — a
/// component callback must never block the executor, so there is no blocking call). Each tick
/// pumps `nros_cpp_action_client_poll` (which drains the GET-query replies into the registered
/// callbacks) and runs a small state machine:
///   idle → send_goal_async(order=10) → (goal-response cb: accepted) → get_result_async →
///   (result cb: deserialize the sequence + PRINT last element) → done.
/// A wait counter re-sends if a goal/result reply never arrives (the first request(s) can be
/// dropped before the server is discovered, like the A1 client's resend guard). The server
/// computes 0,1,1,2,3,5,8,13,21,34,55 for order = 10, so the client printing `result last=55`
/// proves the cross-process action round-trip.
///
/// PRINT-ONLY: the result needs only example_interfaces, so (unlike a /sum-republishing client)
/// there is no second typed-interface archive and thus no multi-interface codegen-dedup gap.
class FibClient {
    using Action = example_interfaces::action::Fibonacci;

    enum Phase {
        Idle = 0,        // no goal in flight — send one
        GoalSent = 1,    // goal sent — await goal-response callback
        NeedResult = 2,  // accepted — request the result from the tick
        AwaitResult = 3, // result requested — await result callback
        Done = 4         // result received + printed
    };

    ::nros::ActionClientStorage client_;
    ::nros::Timer timer_;
    uint8_t goal_id_[16];
    int phase_ = Idle;
    int waits_ = 0; // ticks waited in the current phase (resend guard)

    // Member callback bodies (the static trampolines below forward `ctx` into these).
    void on_goal_response(bool accepted, const uint8_t goal_id[16]);
    void on_result(const uint8_t goal_id[16], int32_t status, const uint8_t* data, size_t len);

    // Captureless trampolines matching the FFI set_callbacks typedefs.
    static void s_goal_response(bool accepted, const uint8_t goal_id[16], void* ctx);
    static void s_result(const uint8_t goal_id[16], int32_t status, const uint8_t* data, size_t len,
                         void* ctx);

    void send_goal();
    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace action_client_pkg
