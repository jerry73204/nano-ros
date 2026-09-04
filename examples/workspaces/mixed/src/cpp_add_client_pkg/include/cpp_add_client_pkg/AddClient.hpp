#pragma once

#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "example_interfaces.hpp"

namespace cpp_add_client_pkg {

/// AddClient — A1 services (C++ projection). A typed component whose 1 Hz timer drives the
/// POLL-model service client (`create_service_client_raw` + the raw send/try-recv FFI — a
/// component callback must never block, so there is no blocking call): send a request, poll for
/// its reply next tick, on success PRINT the server-computed sum. `a` runs 0,1,2,… with `b = 1`,
/// so the sums are 1,2,3,…; the printed sums ARE the cross-process round-trip proof.
///
/// NOTE (vs the C `c_add_client_pkg`, which also republishes on `/sum`): this C++ client does
/// NOT republish, because a single C++ node linking TWO generated typed-interface archives
/// (std_msgs for /sum + example_interfaces for the service) double-defines the shared
/// builtin_interfaces FFI glue at link — a codegen-dedup gap (the C path hand-rolls raw CDR, so
/// it dodges it). Print-only keeps the demo to one interface pkg; the round-trip is still proven.
class AddClient {
    using Svc = example_interfaces::srv::AddTwoInts;

    ::nros::ServiceClientStorage client_;
    ::nros::Timer timer_;
    int64_t a_ = 0;
    bool in_flight_ = false;
    int waits_ = 0;

    void on_tick();

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace cpp_add_client_pkg
