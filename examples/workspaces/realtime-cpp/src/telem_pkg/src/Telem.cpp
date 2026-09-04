// Telem.cpp — ws-realtime-cpp low-tier telemetry node.
//
// Publishes a monotonic Int32 counter on /telem every 100 ms via the typed
// Publisher<std_msgs::msg::Int32> (generated serialization). Running at 1/10
// the cadence of ctrl_pkg, the e2e test asserts ctrl publishes at least 3× as
// many messages as telem in the same window.

#include "telem_pkg/Telem.hpp"

#include <cstdio>

namespace telem_pkg {

void Telem::on_tick() {
    std_msgs::msg::Int32 msg;
    msg.data = count_;
    if (pub_.publish(msg).ok()) {
        std::printf("[telem] tick=%d\n", count_);
    }
    count_++;
}

::rclcpp::Result Telem::configure(::nros::Node& node) {
    // Line-buffer stdout so each tick flushes immediately when piped.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/telem");
    if (!r.ok()) return r;
    return ::nros::bind_timer<Telem, &Telem::on_tick>(node, timer_, 100, this);
}

} // namespace telem_pkg
