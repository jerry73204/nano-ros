#pragma once

/// @file service.hpp
/// @brief Service client and server classes for nano-ros

#include "nano_ros/cdr.hpp"
#include "nano_ros/node.hpp"
#include "nano_ros/qos.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nano_ros {

// Forward declarations
namespace ffi {
struct RustServiceClient;
struct RustServiceServer;
}  // namespace ffi

// =============================================================================
// ServiceClient - Raw service client
// =============================================================================

/// @brief Raw service client for calling services with pre-serialized data
///
/// This class provides a low-level interface for calling services where
/// the request and response are handled as raw CDR-serialized bytes.
class ServiceClient {
public:
    ~ServiceClient();

    // Non-copyable
    ServiceClient(const ServiceClient&) = delete;
    ServiceClient& operator=(const ServiceClient&) = delete;

    // Movable
    ServiceClient(ServiceClient&&) noexcept;
    ServiceClient& operator=(ServiceClient&&) noexcept;

    /// @brief Call the service with raw CDR data
    /// @param request Raw CDR-serialized request (with encapsulation header)
    /// @return Raw CDR-serialized response (with encapsulation header)
    /// @throws std::runtime_error if the call fails
    std::vector<uint8_t> call_raw(const std::vector<uint8_t>& request);

    /// @brief Get the service name
    std::string get_service_name() const;

private:
    friend class Node;
    ServiceClient();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// TypedServiceClient - Templated service client
// =============================================================================

/// @brief Typed service client for calling services with message types
///
/// @tparam ServiceT Service type with nested Request and Response types
///
/// Example:
/// @code
/// // Define service type (usually generated)
/// struct AddTwoInts {
///     struct Request { int64_t a, b; ... };
///     struct Response { int64_t sum; ... };
/// };
///
/// auto client = node->create_client<AddTwoInts>("/add_two_ints");
/// AddTwoInts::Request req{5, 3};
/// auto response = client->call(req);
/// std::cout << "Sum: " << response.sum << std::endl;
/// @endcode
template <typename ServiceT>
class TypedServiceClient {
public:
    using Request = typename ServiceT::Request;
    using Response = typename ServiceT::Response;

    ~TypedServiceClient() = default;

    // Non-copyable
    TypedServiceClient(const TypedServiceClient&) = delete;
    TypedServiceClient& operator=(const TypedServiceClient&) = delete;

    // Movable
    TypedServiceClient(TypedServiceClient&&) noexcept = default;
    TypedServiceClient& operator=(TypedServiceClient&&) noexcept = default;

    /// @brief Call the service with a typed request
    /// @param request The service request
    /// @return The service response
    /// @throws std::runtime_error if the call fails
    Response call(const Request& request) {
        // Serialize request
        CdrWriter writer(1024);
        writer.write_encapsulation();
        request.serialize(writer);

        // Call service
        auto response_data =
            inner_->call_raw(std::vector<uint8_t>(writer.data(), writer.data() + writer.size()));

        // Deserialize response
        CdrReader reader(response_data.data(), response_data.size());
        reader.read_encapsulation();
        Response response;
        response.deserialize(reader);
        return response;
    }

    /// @brief Get the service name
    std::string get_service_name() const {
        return inner_->get_service_name();
    }

private:
    friend class Node;
    explicit TypedServiceClient(std::shared_ptr<ServiceClient> inner) : inner_(std::move(inner)) {}

    std::shared_ptr<ServiceClient> inner_;
};

// =============================================================================
// ServiceServer - Raw service server
// =============================================================================

/// @brief Raw service server for handling service requests
///
/// This class provides a low-level interface for handling service requests
/// where the request and response are handled as raw CDR-serialized bytes.
class ServiceServer {
public:
    ~ServiceServer();

    // Non-copyable
    ServiceServer(const ServiceServer&) = delete;
    ServiceServer& operator=(const ServiceServer&) = delete;

    // Movable
    ServiceServer(ServiceServer&&) noexcept;
    ServiceServer& operator=(ServiceServer&&) noexcept;

    /// @brief Try to receive a service request (non-blocking)
    /// @return Raw CDR-serialized request, or empty vector if no request available
    std::vector<uint8_t> try_recv_request();

    /// @brief Send a response for the last received request
    /// @param response Raw CDR-serialized response (with encapsulation header)
    /// @throws std::runtime_error if sending fails
    void send_reply(const std::vector<uint8_t>& response);

    /// @brief Get the service name
    std::string get_service_name() const;

private:
    friend class Node;
    ServiceServer();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// TypedServiceServer - Templated service server
// =============================================================================

/// @brief Typed service server for handling service requests with message types
///
/// @tparam ServiceT Service type with nested Request and Response types
///
/// Example (polling):
/// @code
/// auto server = node->create_service<AddTwoInts>("/add_two_ints");
///
/// while (true) {
///     if (auto request = server->try_recv_request()) {
///         AddTwoInts::Response response;
///         response.sum = request->a + request->b;
///         server->send_reply(response);
///     }
///     std::this_thread::sleep_for(std::chrono::milliseconds(10));
/// }
/// @endcode
template <typename ServiceT>
class TypedServiceServer {
public:
    using Request = typename ServiceT::Request;
    using Response = typename ServiceT::Response;

    ~TypedServiceServer() = default;

    // Non-copyable
    TypedServiceServer(const TypedServiceServer&) = delete;
    TypedServiceServer& operator=(const TypedServiceServer&) = delete;

    // Movable
    TypedServiceServer(TypedServiceServer&&) noexcept = default;
    TypedServiceServer& operator=(TypedServiceServer&&) noexcept = default;

    /// @brief Try to receive a service request (non-blocking)
    /// @return The request if available, or nullopt if no request pending
    std::optional<Request> try_recv_request() {
        auto raw_request = inner_->try_recv_request();
        if (raw_request.empty()) {
            return std::nullopt;
        }

        // Deserialize request
        CdrReader reader(raw_request.data(), raw_request.size());
        reader.read_encapsulation();
        Request request;
        request.deserialize(reader);
        return request;
    }

    /// @brief Send a response for the last received request
    /// @param response The service response
    /// @throws std::runtime_error if sending fails
    void send_reply(const Response& response) {
        // Serialize response
        CdrWriter writer(1024);
        writer.write_encapsulation();
        response.serialize(writer);

        inner_->send_reply(std::vector<uint8_t>(writer.data(), writer.data() + writer.size()));
    }

    /// @brief Get the service name
    std::string get_service_name() const {
        return inner_->get_service_name();
    }

private:
    friend class Node;
    explicit TypedServiceServer(std::shared_ptr<ServiceServer> inner) : inner_(std::move(inner)) {}

    std::shared_ptr<ServiceServer> inner_;
};

}  // namespace nano_ros
