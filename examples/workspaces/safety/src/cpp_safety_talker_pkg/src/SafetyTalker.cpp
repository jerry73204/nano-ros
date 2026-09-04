// SafetyTalker — Phase 269 W3: counter publisher on /chatter.
// When NANO_ROS_SAFETY_E2E=ON (lowered from [system].features = ["safety"] via
// NanoRosCapabilities.cmake), the zenoh backend automatically attaches a CRC-32 +
// sequence number on every publish. No code change required here.

#include "cpp_safety_talker_pkg/SafetyTalker.hpp"

#include <cstdio>

namespace cpp_safety_talker_pkg {

void SafetyTalker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = counter_++;
    if (pub_.publish(m).ok()) {
        std::printf("[TALKER] Published: %d\n", m.data);
        std::fflush(stdout);
    }
}

::rclcpp::Result SafetyTalker::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    return ::nros::bind_timer<SafetyTalker, &SafetyTalker::on_tick>(node, timer_, 1000, this);
}

} // namespace cpp_safety_talker_pkg
