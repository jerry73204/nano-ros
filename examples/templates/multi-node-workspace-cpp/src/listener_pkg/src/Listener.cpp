// Listener — typed component (RFC-0043). Typed member callback on the generated
// `std_msgs::msg::Int32` (issue #218).

#include "listener_pkg/Listener.hpp"

#include <cstdio>

namespace listener_pkg {

void Listener::on_msg(const ::std_msgs::msg::Int32& msg) {
    std::printf("Received: %d\n", static_cast<int>(msg.data));
    ++recv_;
}

::rclcpp::Result Listener::configure(::nros::Node& node) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // The typed `Publisher<Int32>` registers the DDS-mangled keyexpr, so the
    // raw sub must match on `Int32::TYPE_NAME` (240.1 finding; raw↔typed
    // type-name unification is a separate concern).
    return ::nros::bind_subscription<::std_msgs::msg::Int32, Listener, &Listener::on_msg>(
        node, "/chatter", this);
}

} // namespace listener_pkg
