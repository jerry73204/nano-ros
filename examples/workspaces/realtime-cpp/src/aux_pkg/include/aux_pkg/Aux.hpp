#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace aux_pkg {

/// ws-realtime-cpp-mps2 — mid-tier auxiliary node. Publishes a monotonic
/// counter on /aux every 50 ms. The configure-shape (RFC-0043) receives a
/// Node& to create publishers and timers. Bound to the mid-priority FreeRTOS
/// task (priority 3) via FreertosBoard::run_tiers (RFC-0015 §5 embedded). This
/// tier is spawned BY a spawned tier (boot→mid→low), so it is the middle hop
/// the #144 chained-spawn fix serializes.
class Aux {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace aux_pkg
