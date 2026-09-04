/// @file Listener.cpp
/// @brief Zephyr C++ listener — typed component (RFC-0043 / phase-244.C2).

#include "Listener.hpp"

#include <cstdio>

namespace zephyr_cpp_listener {

void Listener::on_raw(const uint8_t* data, size_t len) {
    // Decode via the generated typed deserializer (phase-277 W4; was
    // hand-rolled CDR) and log the official ROS 2 demo line.
    std_msgs::msg::String m;
    if (std_msgs::msg::String::ffi_deserialize(data, len, &m) == 0) {
        std::printf("I heard: [%s]\n", m.data.c_str());
        ++recv_;
    }
}

::rclcpp::Result Listener::configure(::nros::Node& node) {
    // Unbuffered stdout — a full-buffered console can swallow the final
    // line(s) when the harness kills the QEMU before a flush.
    // `::setvbuf` (global) not `std::setvbuf` — Zephyr's minimal libcpp/picolibc
    // `<cstdio>` declares it in the global namespace only.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    // Raw (zero-copy) subscription bound to the member by identity; type name is
    // the DDS-mangled form to match the sibling typed talker's Publisher<String>.
    ::rclcpp::Result r = ::nros::bind_subscription_raw<Listener, &Listener::on_raw>(
        node, "/chatter", std_msgs::msg::String::TYPE_NAME, this);
    if (r.ok()) {
        // Readiness marker the rtos_e2e harness greps before driving the talker.
        // Shared readiness marker (phase-342 W7) plus zephyr.rs's own
        // NODE_READY_MARKER, which stays until that file uses the shared table.
        std::printf("Subscriber created for topic: %s\n", "/chatter");
        std::printf("Waiting for messages\n");
    }
    return r;
}

} // namespace zephyr_cpp_listener
