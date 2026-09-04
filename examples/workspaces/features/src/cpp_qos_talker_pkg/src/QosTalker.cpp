// QosTalker — typed component (RFC-0043), the C++ projection of ws-qos-c's
// QosTalker. The nano-ros QoS differentiator in C++: instead of the default
// profile the committed cpp_lifecycle_talker_pkg uses, `configure` builds a NON-DEFAULT
// `rclcpp::QoS` via the fluent builder (`.reliable().transient_local().keep_last(10)`)
// and passes it to `Node::create_publisher`. The matching QosListener declares
// the byte-identical profile so the QoS-matched endpoints connect.

#include "cpp_qos_talker_pkg/QosTalker.hpp"

#include <cstdio>

namespace cpp_qos_talker_pkg {

void QosTalker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = count_++;
    if (pub_.publish(m).ok()) {
        std::printf("Published: %d\n", m.data);
    }
}

::rclcpp::Result QosTalker::configure(::nros::Node& node) {
    // `::setvbuf` (C global): line-buffer stdout so each `Published:` flushes
    // immediately when piped (the test reads the output live).
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    // Non-default QoS contract both endpoints declare: RELIABLE delivery,
    // TRANSIENT_LOCAL durability, KEEP_LAST(10) history depth.
    const ::rclcpp::QoS qos =
        ::rclcpp::QoS::default_profile().reliable().transient_local().keep_last(10);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter", qos);
    if (!r.ok()) return r;
    // Member-fn-pointer-as-template-param → no-alloc trampoline; `this` is ctx.
    return ::nros::bind_timer<QosTalker, &QosTalker::on_tick>(node, timer_, 1000, this);
}

} // namespace cpp_qos_talker_pkg
