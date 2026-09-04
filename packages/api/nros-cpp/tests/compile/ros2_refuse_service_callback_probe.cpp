/*
 * NEGATIVE probe — phase-417 W2.c, the callback-shape refusal.
 *
 * `rclcpp::Node::create_service<Srv>(name, callback)` hands the callback
 * `std::shared_ptr<Request>` and `std::shared_ptr<Response>` (plus a request
 * header), which needs a per-request heap allocation on the delivery path.
 * nano-ros has no allocator there (RFC-0022) and delivers by REFERENCE into
 * caller-owned storage, so adopting that signature would mean a second delivery
 * path — RFC-0019's "a second code path that can produce a different answer".
 *
 * Absence would give "no matching function for call to create_service", which
 * is honest and teaches nothing. The refusal names the required handler
 * signature instead, which is the whole migration.
 *
 * `just check cpp` requires this TU to FAIL. The handler shapes nano-ros DOES
 * take — `void(const S::Request&, S::Response&)` for a service,
 * `void(const S::Response&)` for a client, as a function pointer or a
 * capture-less lambda — plus the poll-style overloads and a QoS argument that
 * must NOT be mistaken for a callback, are all in the POSITIVE probe
 * (`ros2_api_adoption_stage2.cpp`). Compile that one first.
 */

#include <nros/nros.hpp>

#include <memory>

namespace {

struct AddTwoInts {
    struct Request {
        int64_t a = 0;
        int64_t b = 0;
        static const size_t SERIALIZED_SIZE_MAX = 16;
        static constexpr const char* TYPE_NAME =
            "example_interfaces::srv::dds_::AddTwoInts_Request_";
        static constexpr const char* TYPE_HASH = "RIHS01_add_two_ints_request_stub";
        static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
            if (out) *out = 0;
            return 0;
        }
        static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    };
    struct Response {
        int64_t sum = 0;
        static const size_t SERIALIZED_SIZE_MAX = 8;
        static constexpr const char* TYPE_NAME =
            "example_interfaces::srv::dds_::AddTwoInts_Response_";
        static constexpr const char* TYPE_HASH = "RIHS01_add_two_ints_response_stub";
        static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
            if (out) *out = 0;
            return 0;
        }
        static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    };
    static constexpr const char* TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_";
    static constexpr const char* TYPE_HASH = "RIHS01_add_two_ints_stub";
};

// Upstream's handler signature, verbatim.
void upstream_shaped_handler(const std::shared_ptr<AddTwoInts::Request> request,
                             std::shared_ptr<AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
}

} // namespace

int ros2_refuse_service_callback_probe(rclcpp::Node& node);
int ros2_refuse_service_callback_probe(rclcpp::Node& node) {
    auto service = node.create_service<AddTwoInts>("add_two_ints", &upstream_shaped_handler);
    (void)service;
    return 0;
}
