// issue 0339 — the standard rclcpp service-client idiom must COMPILE against
// the compat shim.
//
// `rclcpp::spin_until_future_complete` used to return `void`, so this file
// would not build: there was no value to compare, and a caller could not tell
// success from timeout. It now returns a `FutureReturnCode` mirroring
// upstream's.
//
// Syntax-only fixture, compiled by `tests/cpp_api_drift.rs`.

#include <nros/nros.hpp>

#include <memory>

namespace {

/// Minimal stand-in for the future type the shim accepts — it only requires
/// `is_ready()`, which both `nros::Promise` and a ported future provide.
struct FakeFuture {
    bool ready = false;
    bool is_ready() const { return ready; }
};

/// The canonical upstream shape. The point of the fixture is that this
/// compiles at all.
bool wait_for_response(const rclcpp::Node::SharedPtr& node, const FakeFuture& future) {
    if (rclcpp::spin_until_future_complete(node, future, 1000) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    }
    return false;
}

/// TIMEOUT and INTERRUPTED must be distinguishable from each other, not just
/// from success — that is the half a `void` return erased completely.
const char* describe(rclcpp::FutureReturnCode code) {
    switch (code) {
    case rclcpp::FutureReturnCode::SUCCESS:
        return "success";
    case rclcpp::FutureReturnCode::TIMEOUT:
        return "timeout";
    case rclcpp::FutureReturnCode::INTERRUPTED:
        return "interrupted";
    }
    return "unknown";
}

/// The unbounded form still compiles with the default argument.
bool wait_forever(const rclcpp::Node::SharedPtr& node, const FakeFuture& future) {
    return rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS;
}

} // namespace

int main() {
    rclcpp::Node::SharedPtr node;
    FakeFuture future;
    (void)wait_for_response(node, future);
    (void)wait_forever(node, future);
    (void)describe(rclcpp::FutureReturnCode::TIMEOUT);
    return 0;
}
