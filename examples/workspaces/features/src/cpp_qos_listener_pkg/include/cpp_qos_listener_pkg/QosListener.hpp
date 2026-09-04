#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_qos_listener_pkg {

/// QosListener — a stateful typed component (RFC-0043). `configure` binds the
/// member `on_msg` as a typed subscription on `/chatter` with a QoS
/// profile that MATCHES the talker's publisher (RELIABLE + TRANSIENT_LOCAL +
/// KEEP_LAST(10), built via the `rclcpp::QoS` builder). Matching the per-entity QoS
/// contract is what lets the QoS-tagged endpoints connect. The member decodes the
/// CDR-encoded Int32 and prints `Received: N`.
class QosListener {
    int recv_ = 0;

    void on_msg(const ::std_msgs::msg::Int32& msg); // typed member callback

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_qos_listener_pkg
