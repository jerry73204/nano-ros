// Talker — typed component (RFC-0043). Real `on_tick` body bound by identity;
// no string callback name, no synthesizing interpreter.

#include "talker_pkg/Talker.hpp"

#include <cstdio>

namespace talker_pkg {

void Talker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = count_++;
    if (pub_.publish(m).ok()) {
        std::printf("Published: %d\n", m.data);
    }
}

::rclcpp::Result Talker::configure(::nros::Node& node) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    // Member-fn-pointer-as-template-param → no-alloc trampoline; `this` is ctx.
    return ::nros::bind_timer<Talker, &Talker::on_tick>(node, timer_, 500, this);
}

} // namespace talker_pkg
