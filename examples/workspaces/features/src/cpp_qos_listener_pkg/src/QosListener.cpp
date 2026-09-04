// QosListener — typed component (RFC-0043), the C++ projection of ws-qos-c's
// QosListener. `configure` builds the SAME non-default `rclcpp::QoS`
// (`.reliable().transient_local().keep_last(10)`) the talker declares and passes
// it to the raw subscription bind. Matching the per-entity QoS contract is what
// lets the QoS-tagged endpoints connect; a mismatch makes the listener receive
// nothing.

#include "cpp_qos_listener_pkg/QosListener.hpp"

#include <cstdio>

namespace cpp_qos_listener_pkg {

void QosListener::on_msg(const ::std_msgs::msg::Int32& msg) {
    std::printf("Received: %d\n", static_cast<int>(msg.data));
    ++recv_;
}

::rclcpp::Result QosListener::configure(::nros::Node& node) {
    // `::setvbuf` (C global): line-buffer stdout so each `Received:` flushes live.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    // Byte-identical to the talker's profile — both endpoints must declare the same
    // RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(10) contract to connect.
    const ::rclcpp::QoS qos =
        ::rclcpp::QoS::default_profile().reliable().transient_local().keep_last(10);
    // Typed member binding (RFC-0044): keyexpr + deserialize come from the
    // generated `std_msgs::msg::Int32` (issue #218 — hand-decode retired).
    return ::nros::bind_subscription<::std_msgs::msg::Int32, QosListener, &QosListener::on_msg>(
        node, "/chatter", this, qos);
}

} // namespace cpp_qos_listener_pkg
