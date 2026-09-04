// Talker — typed component (RFC-0043). Real `on_tick` body bound by identity;
// no string callback name, no synthesizing interpreter.

#include "talker_pkg/Talker.hpp"

#include <cstdio>

#include <nros/log.hpp> // A5 — node logging via the nros-log facade

namespace talker_pkg {

void Talker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = count_++;
    if (pub_.publish(m).ok()) {
        std::printf("Published: %d\n", m.data);
    }
    // A5 — log each tick via the nros-log facade. `nros_log_default_logger()` is the built-in
    // DEFAULT_LOGGER (level Info; nullptr would DROP the record). The first emit lazy-installs the
    // default sink → the posix platform writer → "[INFO] nros: cpp_talker logging seq=N".
    NROS_LOG_INFO(nros_log_default_logger(), "cpp_talker logging seq=%d", m.data);
}

::rclcpp::Result Talker::configure(::nros::Node& node) {
    // `::setvbuf` (C global), not `std::setvbuf` — Zephyr's picolibc <cstdio> does not put
    // setvbuf in namespace std; the C global is available on every platform.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    // Member-fn-pointer-as-template-param → no-alloc trampoline; `this` is ctx.
    return ::nros::bind_timer<Talker, &Talker::on_tick>(node, timer_, 1000, this);
}

} // namespace talker_pkg
