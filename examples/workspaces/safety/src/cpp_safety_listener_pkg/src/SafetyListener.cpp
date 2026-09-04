// SafetyListener — Phase 269 W3: C++ validated-subscription listener.
//
// Registers a safety subscription via `node.create_subscription_with_safety()`.
// The handler receives the typed message AND the E2E CRC/sequence status via
// `nros_cpp_integrity_status_t`. CRC-valid messages increment the received count
// and republish it on /safe_ok — which the cross-process e2e
// (tests/cpp_c_safety_integrity_e2e.rs) subscribes to and asserts climbs.
//
// Requires NANO_ROS_SAFETY_E2E=ON (lowered from [system].features = ["safety"]
// via NanoRosCapabilities.cmake).

#include "cpp_safety_listener_pkg/SafetyListener.hpp"

#include <cstdio>

namespace cpp_safety_listener_pkg {

// `Node::create_subscription_with_safety` takes a plain function-pointer-convertible
// callback `void(const M&, const nros_cpp_integrity_status_t&)` — the integrity path
// has no `void* ctx` seam (unlike the plain callback path), so it cannot carry `this`.
// A typed component is a singleton, so we forward through a file-static instance
// pointer set in `configure` and a non-capturing lambda (the `+` forces the
// function-pointer conversion). The lambda body sits in this TU's namespace scope but
// is defined inside the `configure` member below, so it retains access to the private
// `on_chatter`.
namespace {
SafetyListener* g_safety_listener_self = nullptr;
} // namespace

void SafetyListener::on_chatter(const std_msgs::msg::Int32& msg,
                                const nros_cpp_integrity_status_t& status) {
    if (status.crc_valid == 1) {
        received_++;
        std::printf("[LISTENER] CRC ok — data=%d count=%d gap=%lld dup=%s\n", msg.data, received_,
                    static_cast<long long>(status.gap), status.duplicate ? "true" : "false");
        std::fflush(stdout);

        // Republish the running CRC-valid count on /safe_ok.
        std_msgs::msg::Int32 ok_msg;
        ok_msg.data = received_;
        pub_ok_.publish(ok_msg);
    } else {
        integrity_faults_++;
        std::printf("[LISTENER] integrity fault — data=%d crc_valid=%d gap=%lld dup=%s faults=%d\n",
                    msg.data, static_cast<int>(status.crc_valid),
                    static_cast<long long>(status.gap), status.duplicate ? "true" : "false",
                    integrity_faults_);
        std::fflush(stdout);
    }
}

::rclcpp::Result SafetyListener::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    g_safety_listener_self = this;

    // Create /safe_ok publisher for reporting CRC-valid counts.
    ::rclcpp::Result r = node.create_publisher(pub_ok_, "/safe_ok");
    if (!r.ok()) return r;

    // Register the integrity-carrying subscription on /chatter.
    // The callback receives (const Int32&, const nros_cpp_integrity_status_t&).
    // Requires NANO_ROS_SAFETY_E2E=ON; the method is declared in node.hpp and
    // defined in subscription.hpp, both gated on #if defined(NANO_ROS_SAFETY_E2E).
    // The `+` forces the non-capturing lambda to a function pointer (the API's
    // required signature); it forwards to the singleton's `on_chatter`.
    return node.create_subscription_with_safety<std_msgs::msg::Int32>(
        sub_, "/chatter",
        +[](const std_msgs::msg::Int32& msg, const nros_cpp_integrity_status_t& status) {
            if (g_safety_listener_self != nullptr) {
                g_safety_listener_self->on_chatter(msg, status);
            }
        });
}

} // namespace cpp_safety_listener_pkg
