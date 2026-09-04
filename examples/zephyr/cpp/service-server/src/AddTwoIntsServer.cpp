/// @file AddTwoIntsServer.cpp
/// @brief Zephyr C++ AddTwoInts service server — typed component.

#include "AddTwoIntsServer.hpp"

#include <cstdio>

namespace zephyr_cpp_service_server {

static int64_t read_i64_le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return static_cast<int64_t>(v);
}

static void write_i64_le(uint8_t* p, int64_t x) {
    uint64_t v = static_cast<uint64_t>(x);
    for (int i = 0; i < 8; ++i) {
        p[i] = static_cast<uint8_t>(v >> (8 * i));
    }
}

bool AddTwoIntsServer::handle_add(const uint8_t* req, size_t req_len, uint8_t* resp,
                                  size_t resp_cap, size_t* resp_len) {
    // AddTwoInts request CDR: 4-byte encapsulation header, then int64 a, int64 b
    // (8-aligned within the payload → buffer offsets 4 and 12).
    if (req_len < 20 || resp_cap < 12) {
        return false;
    }
    int64_t a = read_i64_le(req + 4);
    int64_t b = read_i64_le(req + 12);
    int64_t sum = a + b;
    // Reply CDR: copy the request's encapsulation header, then int64 sum.
    resp[0] = req[0];
    resp[1] = req[1];
    resp[2] = req[2];
    resp[3] = req[3];
    write_i64_le(resp + 4, sum);
    *resp_len = 12;
    std::printf("Incoming request\na: %lld b: %lld\n", static_cast<long long>(a),
                static_cast<long long>(b));
    return true;
}

::rclcpp::Result AddTwoIntsServer::configure(::nros::Node& node) {
    // Unbuffered stdout — a full-buffered console can swallow the final
    // line(s) when the harness kills the QEMU before a flush.
    // `::setvbuf` (global) not `std::setvbuf` — Zephyr's minimal libcpp/picolibc
    // `<cstdio>` declares it in the global namespace only.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    ::rclcpp::Result r = ::nros::bind_service_raw<AddTwoIntsServer, &AddTwoIntsServer::handle_add>(
        node, "/add_two_ints", "example_interfaces/srv/AddTwoInts", this);
    if (r.ok()) {
        // Readiness marker the e2e harness greps before driving the client.
        std::printf("Waiting for service requests\n");
    }
    return r;
}

} // namespace zephyr_cpp_service_server
