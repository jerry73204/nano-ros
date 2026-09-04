#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_lifecycle_talker_pkg {

/// LifecycleTalker — Phase 269 W2: counter publisher on /chatter.
/// Lifecycle state machine (register + Configure→Activate) is handled by the
/// generated entry's `__nros_entry_setup` via `nros_cpp_lifecycle_autostart`.
class LifecycleTalker {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int32_t counter_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_lifecycle_talker_pkg
