// ReadingTalker — typed component (RFC-0043) on the GENERATED
// `custom_msgs::msg::Reading` bindings (phase-293 / issue #212). The
// workspace-local schema is YOURS (src/custom_msgs/msg/Reading.msg); adding a
// field costs a regeneration, not a byte-offset audit.

#include "cpp_reading_talker_pkg/ReadingTalker.hpp"

#include <cstdio>

namespace cpp_reading_talker_pkg {

void ReadingTalker::on_tick() {
    ::custom_msgs::msg::Reading msg;
    msg.temperature = 20.0 + static_cast<double>(count_) * 0.5;
    msg.humidity = 50.0;
    msg.sequence = count_;

    if (pub_.publish(msg).ok()) {
        std::printf("[reading_talker] sent seq=%d temp=%.1f\n", static_cast<int>(msg.sequence),
                    msg.temperature);
    }
    count_++;
}

::rclcpp::Result ReadingTalker::configure(::nros::Node& node) {
    // `::setvbuf` (C global): line-buffer stdout so each `sent seq=` flushes live.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/reading");
    if (!r.ok()) return r;
    return ::nros::bind_timer<ReadingTalker, &ReadingTalker::on_tick>(node, timer_, 1000, this);
}

} // namespace cpp_reading_talker_pkg
