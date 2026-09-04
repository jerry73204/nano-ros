#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace ctrl_pkg {

/// High-tier control node. Publishes a monotonic counter on /ctrl every
/// 10 ms. The configure-shape (RFC-0043) receives a Node& to create
/// publishers and timers; the entry binds it to the high-priority sched
/// context via nros_cpp_node_create_ex (emitted by the C++ codegen path).
class Ctrl {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace ctrl_pkg
