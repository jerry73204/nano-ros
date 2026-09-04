#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace telem_pkg {

/// Low-tier telemetry node. Publishes a monotonic counter on /telem every
/// 100 ms. Bound to the low-priority sched context by the C++ codegen path
/// (NodeBuilder::sched). Runs at 1/10 the cadence of ctrl_pkg; the e2e test
/// asserts ctrl publishes ≥3× as many messages as telem.
class Telem {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace telem_pkg
