// ReadingListener — typed component (RFC-0043) on the GENERATED
// `custom_msgs::msg::Reading` bindings (phase-293 / issue #212): the typed
// member callback receives the deserialized struct; no hand CDR, no
// hand-typed DDS name.

#include "cpp_reading_listener_pkg/ReadingListener.hpp"

#include <cstdio>

namespace cpp_reading_listener_pkg {

void ReadingListener::on_reading(const ::custom_msgs::msg::Reading& msg) {
    std::printf("reading seq=%d temp=%.1f\n", static_cast<int>(msg.sequence), msg.temperature);
    ++recv_;
}

::rclcpp::Result ReadingListener::configure(::nros::Node& node) {
    // `::setvbuf` (C global): line-buffer stdout so each `reading seq=` flushes live.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    // Typed member binding (RFC-0044 §242.2): keyexpr + deserialize come from
    // the generated `custom_msgs::msg::Reading`.
    return ::nros::bind_subscription<::custom_msgs::msg::Reading, ReadingListener,
                                     &ReadingListener::on_reading>(node, "/reading", this);
}

} // namespace cpp_reading_listener_pkg
