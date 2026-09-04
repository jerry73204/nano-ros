#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "example_interfaces.hpp"

namespace action_server_pkg {

/// FibServer — A4 actions (C++ projection of the Rust `action_server_pkg` / the C
/// `c_fib_server_pkg`). A typed component: `configure` binds the member goal/cancel callbacks on
/// `/fibonacci` via the COMPONENT seam `::nros::bind_action_server_raw<C, &on_goal, &on_cancel>`
/// (the trampolines are no-alloc; the goal CDR is parsed with the generated
/// `example_interfaces::action::Fibonacci::Goal::ffi_deserialize`). The goal callback ACCEPTS the
/// goal and stashes its UUID + order; a 500 ms timer tick — the only place the executor is free
/// for action ops, since a component callback must never block — computes the Fibonacci sequence
/// and completes the goal with `nros_cpp_action_server_complete_goal`. Cross-process only (issue
/// 0096): server + client run as two entries.
class FibServer {
    using Action = example_interfaces::action::Fibonacci;

    ::nros::ActionServerStorage storage_;
    ::nros::Timer timer_;
    void* executor_ = nullptr; // opaque executor handle (needed for complete_goal)
    uint8_t goal_id_[16];      // the accepted goal's UUID
    int32_t order_ = 0;        // the requested Fibonacci order
    bool has_pending_ = false; // a goal is accepted + awaiting its tick-driven result
    int goal_count_ = 0;

    int32_t on_goal(const uint8_t goal_id[16], const uint8_t* data, size_t len);
    int32_t on_cancel(const uint8_t goal_id[16]);
    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace action_server_pkg
