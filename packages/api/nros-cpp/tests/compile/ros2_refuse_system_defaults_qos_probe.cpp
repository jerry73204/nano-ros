/*
 * NEGATIVE probe — phase-417 W3.f, the named-QoS transcription.
 *
 * `rclcpp::SystemDefaultsQoS()` returned `QoS(10)`, which is
 * `rmw_qos_profile_DEFAULT` — a different upstream profile. Upstream's
 * `rmw_qos_profile_system_default` names no concrete policy at all: every field
 * is a sentinel meaning "let the RMW decide", and issue 0829 measured the two
 * reference RMWs resolving the depth sentinel to different numbers (Cyclone 1,
 * zenoh 42). `nros::QoS` has no sentinel — the backend is linked at build time
 * — so no value this could return would be right, which is why it is a refusal
 * and not a corrected constant like its six siblings.
 *
 * `just check cpp` requires this TU to FAIL. The six profiles that CAN be
 * transcribed (`SensorDataQoS`, `ServicesQoS`, `ParametersQoS`,
 * `ParameterEventsQoS`, `RosoutQoS`, `ClockQoS`) are checked field-by-field
 * against upstream in the POSITIVE probe — compile that one first.
 */

#include <nros/rclcpp_compat.hpp>

int ros2_refuse_system_defaults_qos_probe();
int ros2_refuse_system_defaults_qos_probe() {
    // Both upstream spellings; a class refusal has to cover the braced form too.
    rclcpp::SystemDefaultsQoS braced{};
    rclcpp::SystemDefaultsQoS called = rclcpp::SystemDefaultsQoS();
    return braced.depth() + called.depth();
}
