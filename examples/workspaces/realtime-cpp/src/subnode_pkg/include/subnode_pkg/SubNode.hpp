#pragma once

#include <nros/component_node.hpp>

#include "std_msgs.hpp"

namespace subnode_pkg {

/// Sub-node (RFC-0047): ONE ComponentNode with TWO callback groups.
///
/// This is the RFC-0047 core proof: a single node that splits its callbacks across
/// two scheduling tiers via callback-group binding. The two groups ("ctrl" and
/// "telem") are declared in code (rclcpp shape) and assigned to workspace tiers in
/// system.toml `group_tiers` — the package itself carries NO tier reference, making
/// it portable across workspaces with different tier names.
///
/// - "ctrl"  group: 10 ms timer → publishes /ctrl  → bound to the "high" tier
/// - "telem" group: 100 ms timer → publishes /telem → bound to the "low" tier
///
/// The entry emits bind_group_sched("sub_node", "ctrl", SC_HIGH) and
/// bind_group_sched("sub_node", "telem", SC_LOW) before construction, so both
/// timers land on their respective sched contexts at registration.
class SubNode : public ::nros::ComponentNode {
    ::rclcpp::Publisher<std_msgs::msg::Int32> ctrl_pub_;
    ::rclcpp::Publisher<std_msgs::msg::Int32> telem_pub_;
    int ctrl_count_ = 0;
    int telem_count_ = 0;

    void on_ctrl();
    void on_telem();

  public:
    explicit SubNode(::nros::NodeHandle h);
};

} // namespace subnode_pkg
