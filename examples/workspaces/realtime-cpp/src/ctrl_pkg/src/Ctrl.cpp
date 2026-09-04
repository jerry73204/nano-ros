// Ctrl.cpp — ws-realtime-cpp high-tier control node.
//
// Publishes a monotonic Int32 counter on /ctrl every 10 ms via the typed
// Publisher<std_msgs::msg::Int32> (generated serialization). The
// configure-shape receives a Node& that is already bound to the
// high-priority sched context (emitted by the C++ codegen path via
// NodeBuilder::sched).

#include "ctrl_pkg/Ctrl.hpp"

#include <cstdio>

namespace ctrl_pkg {

void Ctrl::on_tick() {
    std_msgs::msg::Int32 msg;
    msg.data = count_;
    if (pub_.publish(msg).ok()) {
        std::printf("[ctrl] tick=%d\n", count_);
    }
    count_++;
}

::rclcpp::Result Ctrl::configure(::nros::Node& node) {
    // Line-buffer stdout so each tick flushes immediately when piped.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/ctrl");
    if (!r.ok()) return r;
    return ::nros::bind_timer<Ctrl, &Ctrl::on_tick>(node, timer_, 10, this);
}

} // namespace ctrl_pkg
