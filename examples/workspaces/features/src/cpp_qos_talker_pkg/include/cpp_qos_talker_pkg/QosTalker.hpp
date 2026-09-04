#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_qos_talker_pkg {

/// QosTalker — a stateful typed component (RFC-0043). `configure` creates a
/// typed `Publisher<std_msgs::msg::Int32>` on `/chatter` with a NON-DEFAULT QoS
/// profile (RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10)) built via the
/// `rclcpp::QoS` builder, and binds the member `on_tick` as a 1 Hz timer that
/// publishes a real counter. QoS is a per-entity contract set IN CODE (no launch
/// `qos_overrides`); the matching `QosListener` declares the byte-identical
/// profile so the two endpoints connect.
class QosTalker {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick(); // real body; bound via &QosTalker::on_tick (no callback name)

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_qos_talker_pkg
