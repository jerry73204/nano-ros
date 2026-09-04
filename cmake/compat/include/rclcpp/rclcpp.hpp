// SPDX-License-Identifier: Apache-2.0
// rclcpp/rclcpp.hpp shim — Phase 209.B. Resolves `#include <rclcpp/rclcpp.hpp>`
// in ported ROS 2 source to the nano-ros compat surface.
#ifndef NROS_COMPAT_RCLCPP_RCLCPP_HPP
#define NROS_COMPAT_RCLCPP_RCLCPP_HPP
// phase-417 stage 6 step A — the `rclcpp::` names are declared BY the API
// headers now, not by a shim over them, so this resolves straight to the
// umbrella. `nros/rclcpp_compat.hpp` is an empty forwarder awaiting deletion.
#include "nros/nros.hpp"
#endif
