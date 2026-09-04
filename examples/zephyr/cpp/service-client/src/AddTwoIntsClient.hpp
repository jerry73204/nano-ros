// Zephyr C++ AddTwoInts service client — typed component.
//
// `configure` creates a service client + a timer that polls: the first tick
// sends ONE fixed request (2, 3); later ticks poll the reply and print the
// sum, then the client goes quiet. (Poll model — clients move to callbacks
// when the C/C++ callback wave lands.)
#ifndef NROS_ZEPHYR_SERVICE_CLIENT_CPP_ADDTWOINTSCLIENT_HPP
#define NROS_ZEPHYR_SERVICE_CLIENT_CPP_ADDTWOINTSCLIENT_HPP

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

namespace zephyr_cpp_service_client {

class AddTwoIntsClient {
    ::nros::ServiceClientStorage client_;
    ::nros::Timer timer_;
    int64_t a_ = 2;
    int64_t b_ = 3;
    bool awaiting_ = false;
    bool done_ = false;

    void on_tick(); // send / poll driver, bound by identity

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace zephyr_cpp_service_client

#endif // NROS_ZEPHYR_SERVICE_CLIENT_CPP_ADDTWOINTSCLIENT_HPP
