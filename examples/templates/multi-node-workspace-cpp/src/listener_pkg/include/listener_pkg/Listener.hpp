#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace listener_pkg {

/// Listener — a stateful component (RFC-0043). `configure` binds the member
/// `on_msg` (by identity, no name) as a TYPED member subscription on
/// `/chatter`. The raw path borrows the wire bytes (no copy/deserialize); the
/// member decodes the CDR-encoded Int32 and counts receipts.
class Listener {
    int recv_ = 0;

    void on_msg(const ::std_msgs::msg::Int32& msg); // typed member callback

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace listener_pkg
