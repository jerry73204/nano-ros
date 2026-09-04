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
    // `::setvbuf` (C global), not `std::setvbuf` — Zephyr's picolibc <cstdio> does not put
    // setvbuf in namespace std; the C global is available on every platform.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    // Typed member binding (RFC-0044): keyexpr + deserialize come from the
    // generated `std_msgs::msg::Int32` (issue #218 — hand-decode retired).
    return ::nros::bind_subscription<::std_msgs::msg::Int32, Listener, &Listener::on_msg>(
        node, "/chatter", this);
}

} // namespace listener_pkg
