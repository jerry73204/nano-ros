#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_safety_talker_pkg {

/// SafetyTalker — Phase 269 W3: counter publisher on /chatter.
/// When built with NANO_ROS_SAFETY_E2E=ON the zenoh backend automatically
/// attaches a CRC-32 + sequence number on every publish — no code change needed.
class SafetyTalker {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int32_t counter_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_safety_talker_pkg
