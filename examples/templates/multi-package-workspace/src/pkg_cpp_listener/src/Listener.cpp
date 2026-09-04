/// @file Listener.cpp
/// @brief multi-package-workspace demo — C++ listener, typed component
/// (RFC-0043 / phase-244.C4).

#include "Listener.hpp"

#include <cstdio>

#include "std_msgs.hpp"

namespace pkg_cpp_listener {

void Listener::on_msg(const ::std_msgs::msg::Int32& msg) {
    std::printf("[pkg_cpp_listener] received: %d (total=%d)\n", static_cast<int>(msg.data),
                ++recv_);
}

::rclcpp::Result Listener::configure(::nros::Node& node) {
    // Typed member binding (RFC-0044): keyexpr + deserialize come from the
    // generated `std_msgs::msg::Int32` (issue #218 — hand-decode retired).
    ::rclcpp::Result r =
        ::nros::bind_subscription<::std_msgs::msg::Int32, Listener, &Listener::on_msg>(
            node, "/chatter", this);
    if (r.ok()) {
        std::printf("[pkg_cpp_listener] subscribed to /chatter\n");
    }
    return r;
}

} // namespace pkg_cpp_listener
