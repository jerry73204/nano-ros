#pragma once

#include <nros/component_node.hpp>

#include "std_msgs.hpp"

namespace subnode_pkg {

/// Portability copy of SubNode (RFC-0047). IDENTICAL to
/// ws-realtime-cpp-subnode's SubNode. Deployed here with "fast"/"bulk" tier names
/// instead of "high"/"low" — no package change, tier binding from system.toml.
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
