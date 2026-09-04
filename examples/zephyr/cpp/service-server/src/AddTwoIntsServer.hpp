// Zephyr C++ AddTwoInts service server — typed component (RFC-0043, phase-244.C2).
//
// `configure` binds the member `handle_add` (by identity) as a raw callback-
// style service on `/add_two_ints`; the real handler decodes the CDR request
// (int64 a, b) and writes the CDR reply (int64 sum). No interpreter synthesis.
#ifndef NROS_ZEPHYR_SERVICE_SERVER_CPP_ADDTWOINTSSERVER_HPP
#define NROS_ZEPHYR_SERVICE_SERVER_CPP_ADDTWOINTSSERVER_HPP

#include <cstddef>
#include <cstdint>

#include <nros/component.hpp>
#include <nros/nros.hpp>

namespace zephyr_cpp_service_server {

class AddTwoIntsServer {
    // Real handler bound by identity: fills `resp` from `req`, returns true to
    // send the reply. (No callback name, no ctx arg — `this` is the executor ctx.)
    bool handle_add(const uint8_t* req, size_t req_len, uint8_t* resp, size_t resp_cap,
                    size_t* resp_len);

  public:
    ::rclcpp::Result configure(::nros::Node& node);
};

} // namespace zephyr_cpp_service_server

#endif
