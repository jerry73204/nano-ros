/// @file Talker.cpp
/// @brief Zephyr C++ talker — typed component (RFC-0043 / phase-244.C2).

#include "Talker.hpp"

#include <cstdio>

namespace zephyr_cpp_talker {

void Talker::on_tick() {
    // Pre-increment so the first payload is "Hello World: 1", matching the
    // official ROS 2 demo talker.
    ++count_;
    char payload[64];
    std::snprintf(payload, sizeof(payload), "Hello World: %d", count_);
    std_msgs::msg::String m;
    m.data = payload;
    if (pub_.publish(m).ok()) {
        std::printf("Publishing: '%s'\n", m.data.c_str());
    }
}

::rclcpp::Result Talker::configure(::nros::Node& node) {
    // Unbuffered stdout — a full-buffered console can swallow the final
    // line(s) when the harness kills the QEMU before a flush.
    // `::setvbuf` (global) not `std::setvbuf` — Zephyr's minimal libcpp/picolibc
    // `<cstdio>` declares it in the global namespace only.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    return ::nros::bind_timer<Talker, &Talker::on_tick>(node, timer_, 500, this);
}

} // namespace zephyr_cpp_talker
