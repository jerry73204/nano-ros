// multi-package-workspace demo — C++ listener, typed component
// (RFC-0043 / phase-244.C4). A stateful component: `configure` binds the member
// `on_msg` (by identity, no callback name) as a typed member subscription on
// `/chatter`. The native typed Entry carrier constructs this object + calls
// `configure(node)` and runs `LinuxBoard::run_components`.
#ifndef PKG_CPP_LISTENER_LISTENER_HPP
#define PKG_CPP_LISTENER_LISTENER_HPP

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp" // generated C++ bindings — the on_msg signature names Int32

namespace pkg_cpp_listener {

class Listener {
    int recv_ = 0;

    void on_msg(const ::std_msgs::msg::Int32& msg); // real body; bound by identity

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace pkg_cpp_listener

#endif // PKG_CPP_LISTENER_LISTENER_HPP
