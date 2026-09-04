#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>
#include <nros/subscription.hpp>

#include "std_msgs.hpp"

namespace cpp_safety_listener_pkg {

/// SafetyListener — Phase 269 W3: validated-subscription listener on /chatter.
///
/// Uses `node.create_subscription_with_safety<Int32>()` to register an
/// integrity-carrying callback subscription. The handler receives the typed
/// message AND the E2E CRC verdict via `nros_cpp_integrity_status_t`:
///   - gap:       sequence-number gap since last in-order sample (0 = none)
///   - duplicate: true if the sequence number was already seen
///   - crc_valid: 1 = CRC ok, 0 = mismatch, -1 = no CRC on the wire
///
/// CRC-valid messages increment the counter and republish the count on /safe_ok.
/// Requires NANO_ROS_SAFETY_E2E=ON (lowered from [system].features = ["safety"]).
class SafetyListener {
    ::rclcpp::Publisher<std_msgs::msg::Int32> pub_ok_;
    ::rclcpp::Subscription<std_msgs::msg::Int32> sub_;
    int32_t received_ = 0;
    int32_t integrity_faults_ = 0;

    void on_chatter(const std_msgs::msg::Int32& msg, const nros_cpp_integrity_status_t& status);

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_safety_listener_pkg
