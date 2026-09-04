#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "custom_msgs.hpp"

namespace cpp_reading_talker_pkg {

/// ReadingTalker — typed component (RFC-0043), the C++ projection of
/// ws-custom-msg-c's ReadingTalker, publishing the GENERATED
/// `custom_msgs::msg::Reading` (phase-293 / issue #212: struct, serializer,
/// and type name all come from the bindings generated from the workspace's
/// own `msg/Reading.msg`). `configure` creates a typed publisher on
/// `/reading` and binds `on_tick` as a 1 Hz timer whose `sequence` ramps.
class ReadingTalker {
    ::rclcpp::Publisher<::custom_msgs::msg::Reading> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick(); // real body; bound via &ReadingTalker::on_tick

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_reading_talker_pkg
