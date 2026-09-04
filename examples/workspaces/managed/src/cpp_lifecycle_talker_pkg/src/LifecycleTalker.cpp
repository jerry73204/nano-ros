// LifecycleTalker — Phase 269 W2: managed-node C++ component that publishes a counter
// on /chatter. Lifecycle state machine (register + Configure→Activate) is handled by the
// generated entry's __nros_entry_setup via nros_cpp_lifecycle_autostart; this component
// has no lifecycle callbacks of its own. The e2e test checks that
// `ros2 lifecycle get /talker` returns `active` at boot.

#include "cpp_lifecycle_talker_pkg/LifecycleTalker.hpp"

#include <cstdio>

namespace cpp_lifecycle_talker_pkg {

void LifecycleTalker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = counter_++;
    if (pub_.publish(m).ok()) {
        std::printf("Published: %d\n", m.data);
    }
}

::rclcpp::Result LifecycleTalker::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    return ::nros::bind_timer<LifecycleTalker, &LifecycleTalker::on_tick>(node, timer_, 1000, this);
}

} // namespace cpp_lifecycle_talker_pkg
