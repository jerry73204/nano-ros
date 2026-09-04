// ManagedTalker — Phase 270 (#103): a managed C++ node built with the
// `nros::LifecycleNode` wrapper. Unlike LifecycleTalker (whose lifecycle is driven
// by the entry's `nros_cpp_lifecycle_autostart` codegen), this node authors its own
// transition behavior via the rclcpp-shape on_* overrides and self-drives the
// machine from its install hook. `cpp_lifecycle_node_wrapper_e2e` greps this node's
// stdout for the transition markers + the gated `Published:` lines.

#include "cpp_lifecycle_talker_pkg/ManagedTalker.hpp"

#include <cstdio>

namespace cpp_lifecycle_talker_pkg {

::nros::CallbackReturn ManagedTalker::on_configure(::nros::LifecycleState /*previous*/) {
    std::printf("LC:on_configure\n");
    return ::nros::CallbackReturn::Success;
}

::nros::CallbackReturn ManagedTalker::on_activate(::nros::LifecycleState /*previous*/) {
    active_ = true;
    std::printf("LC:on_activate\n");
    return ::nros::CallbackReturn::Success;
}

::nros::CallbackReturn ManagedTalker::on_deactivate(::nros::LifecycleState /*previous*/) {
    active_ = false;
    std::printf("LC:on_deactivate\n");
    return ::nros::CallbackReturn::Success;
}

void ManagedTalker::on_tick() {
    if (!active_) {
        return; // gated: publish only while Active (proves on_activate ran)
    }
    std_msgs::msg::Int32 m;
    m.data = counter_++;
    if (pub_.publish(m).ok()) {
        std::printf("Published: %d\n", m.data);
    }
}

::rclcpp::Result ManagedTalker::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    // Two-phase: bind the executor handle the component install exposes.
    bind(node.executor_handle());
    ::rclcpp::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) {
        return r;
    }
    r = ::nros::bind_timer<ManagedTalker, &ManagedTalker::on_tick>(node, timer_, 200, this);
    if (!r.ok()) {
        return r;
    }
    // Register REP-2002 services (binds the on_* trampolines) and drive to Active.
    // The wrapper's autostart binds callbacks first, so the overrides above fire.
    r = autostart(::nros::LifecycleState::Active);
    std::printf("LC:state=%d\n", static_cast<int>(get_current_state()));
    return r;
}

} // namespace cpp_lifecycle_talker_pkg
