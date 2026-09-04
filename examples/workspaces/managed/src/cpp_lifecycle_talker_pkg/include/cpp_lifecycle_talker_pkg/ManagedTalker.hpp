#pragma once

#include <nros/component.hpp>
#include <nros/lifecycle.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace cpp_lifecycle_talker_pkg {

/// ManagedTalker — Phase 270 (#103): a C++ managed node written with the
/// `nros::LifecycleNode` wrapper (NOT the entry `[lifecycle] autostart` codegen).
///
/// The component install hook `configure(Node&)` wires the publisher + timer and
/// then self-drives the REP-2002 machine via the wrapper: `bind()` the executor,
/// `register_services()` (binds the on_* trampolines), and `autostart(Active)`
/// (Configure→Activate, firing the overrides below). Publishing is GATED on the
/// active state by `on_activate`/`on_deactivate` — proof the overrides run.
class ManagedTalker : public ::nros::LifecycleNode {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int32_t counter_ = 0;
    bool active_ = false;

    void on_tick();

  public:
    ManagedTalker() = default;

    // rclcpp-shape transition hooks.
    ::nros::CallbackReturn on_configure(::nros::LifecycleState previous) override;
    ::nros::CallbackReturn on_activate(::nros::LifecycleState previous) override;
    ::nros::CallbackReturn on_deactivate(::nros::LifecycleState previous) override;

    // Component install hook.
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_lifecycle_talker_pkg
