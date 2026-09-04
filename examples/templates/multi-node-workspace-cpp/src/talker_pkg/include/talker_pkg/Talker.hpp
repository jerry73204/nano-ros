#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace talker_pkg {

/// Talker — a stateful component (RFC-0043). `configure` creates a publisher
/// on `/chatter` and binds the member `on_tick` (by identity, no name) as a
/// 2 Hz timer callback that publishes a real counter. The typed Entry
/// constructs this object in static storage and calls `configure(node)`; the
/// executor dispatches `on_tick` during `spin_once`.
class Talker {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick(); // real body; bound via &Talker::on_tick (no callback name)

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace talker_pkg
