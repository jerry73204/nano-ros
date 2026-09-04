#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_param_talker_pkg {

/// ParamTalker — Phase 269 W1: reads `publish_period_ms` live each tick from the
/// executor-backed parameter store via `nros_cpp_get_param_integer` and publishes
/// the value on /chatter.
class ParamTalker {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    void* executor_handle_ = nullptr; /* saved at configure for live reads in on_tick */

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_param_talker_pkg
