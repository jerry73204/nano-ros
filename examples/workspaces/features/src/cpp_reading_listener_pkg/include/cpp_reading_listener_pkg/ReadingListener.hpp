#pragma once

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "custom_msgs.hpp"

namespace cpp_reading_listener_pkg {

/// ReadingListener — typed component (RFC-0043) on the GENERATED
/// `custom_msgs::msg::Reading` bindings (phase-293 / issue #212). `configure`
/// binds `on_reading` as a typed member subscription on `/reading`; the
/// trampoline deserializes into the generated struct and the callback prints
/// the `sequence` + `temperature` fields.
class ReadingListener {
    int recv_ = 0;

    void on_reading(const ::custom_msgs::msg::Reading& msg); // typed member callback

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_reading_listener_pkg
