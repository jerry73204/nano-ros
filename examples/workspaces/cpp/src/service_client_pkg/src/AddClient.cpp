// AddClient — typed, poll-model AddTwoInts service client. The C++ projection of the Rust
// service_client_pkg / the C c_add_client_pkg.

#include "service_client_pkg/AddClient.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace service_client_pkg {

void AddClient::on_tick() {
    if (in_flight_) {
        uint8_t resp[64];
        size_t rlen = 0;
        if (nros_cpp_service_client_take_response(client_.bytes, resp, sizeof(resp), &rlen) == 0 &&
            rlen > 0) {
            Svc::Response r;
            if (Svc::Response::ffi_deserialize(resp, rlen, &r) == 0) {
                in_flight_ = false;
                std::printf("[service_client_pkg] sum: %lld\n", static_cast<long long>(r.sum));
            }
        } else if (++waits_ > 6) {
            // No reply (request dropped before discovery) — resend.
            in_flight_ = false;
        }
    }

    if (!in_flight_) {
        Svc::Request req;
        req.a = a_;
        req.b = 1;
        uint8_t buf[64];
        size_t written = 0;
        if (Svc::Request::ffi_serialize(&req, buf, sizeof(buf), &written) == 0 &&
            nros_cpp_service_client_send_request(client_.bytes, buf, written) == 0) {
            in_flight_ = true;
            waits_ = 0;
        }
        a_++;
    }
}

::rclcpp::Result AddClient::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r =
        ::nros::create_service_client_raw(node, client_.bytes, "/add_two_ints", Svc::TYPE_NAME);
    if (!r.ok()) {
        return r;
    }
    return ::nros::bind_timer<AddClient, &AddClient::on_tick>(node, timer_, 500, this);
}

} // namespace service_client_pkg
