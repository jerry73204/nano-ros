// nros-cpp: Service server class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file service.hpp
 * @ingroup grp_service
 * @brief `nros::Service<S>` — typed service server.
 */

#ifndef NROS_CPP_SERVICE_HPP
#define NROS_CPP_SERVICE_HPP

#include <cstdint>
#include <cstddef>

#include "nros/config.hpp"
#include "nros/result.hpp"
#include "nros/size_bound.hpp" // nros::rx_buffer_capacity<M> — the receive-buffer size

// phase-417 W1.a — `<memory>` for the nested pointer aliases. Rationale (and
// why the test is `__has_include` rather than `__STDC_HOSTED__`, issue 0112)
// lives in `publisher.hpp`.
#if defined(NROS_CPP_STD)
#include <memory>
#define NROS_CPP_HAS_SHARED_PTR 1
#elif defined(__has_include)
#if __has_include(<memory>)
#include <memory>
#define NROS_CPP_HAS_SHARED_PTR 1
#endif
#endif

#include "nros_cpp_ffi.h"

// Phase 189.M3.3.e — `nros_cpp_service_server_register` is excluded from
// cbindgen (its Rust signature uses `RawServiceCallback`, an external-crate
// type alias cbindgen names without defining). Declare it locally with a plain
// function-pointer typedef matching the ABI (`bool(req, req_len, resp,
// resp_cap, resp_len, ctx)`).
extern "C" {
typedef bool (*nros_cpp_service_request_callback_t)(const uint8_t* req, size_t req_len,
                                                    uint8_t* resp, size_t resp_cap,
                                                    size_t* resp_len, void* ctx);

nros_cpp_ret_t nros_cpp_service_server_register(const nros_cpp_node_t* node,
                                                const char* service_name, const char* type_name,
                                                const char* type_hash, nros_cpp_qos_t qos,
                                                nros_cpp_service_request_callback_t callback,
                                                void* context, uint8_t sched_context,
                                                size_t* out_handle_id);
} // extern "C"

namespace nros {

/// Typed service server for a ROS 2 service.
///
/// Mirrors `rclcpp::Service<S>`. The service type `S` must provide
/// nested `Request` and `Response` types with `TYPE_NAME`, `TYPE_HASH`,
/// `SERIALIZED_SIZE_MAX`, `ffi_serialize()`, and `ffi_deserialize()`.
///
/// Usage:
/// ```cpp
/// nros::Service<example_interfaces::srv::AddTwoInts> srv;
/// NROS_TRY(node.create_service(srv, "/add_two_ints"));
/// typename decltype(srv)::RequestType req;
/// int64_t seq;
/// if (srv.take_request(req, seq)) {
///     typename decltype(srv)::ResponseType resp;
///     resp.sum = req.a + req.b;
///     srv.send_response(seq, resp);
/// }
/// ```
template <typename S> class Service {
  public:
#ifdef NROS_CPP_HAS_SHARED_PTR
    /// `rclcpp::Service<S>::SharedPtr` — phase-417 W1.a.
    ///
    /// rclcpp indexes its entity types this way, and
    /// `rclcpp::Service<S>::SharedPtr member_;` is close to universal in
    /// ported source. Ergonomics only (RFC-0089 §"Who implements an adopted
    /// name"): a spelling for `std::shared_ptr<Service<S>>`, no second code path.
    ///
    /// Present only where `<memory>` is — a freestanding target has no
    /// `std::shared_ptr` to alias.
    using SharedPtr = std::shared_ptr<Service<S>>;
    /// `rclcpp::Service<S>::ConstSharedPtr` — see `SharedPtr`.
    using ConstSharedPtr = std::shared_ptr<const Service<S>>;
    /// `rclcpp::Service<S>::UniquePtr` — see `SharedPtr`.
    using UniquePtr = std::unique_ptr<Service<S>>;
#endif

    using RequestType = typename S::Request;
    using ResponseType = typename S::Response;

    /// Phase 189.M3.3.e — typed request-handler signatures for the
    /// *callback-style* service (rclcpp dispatch model). The handler fills
    /// `response` from `request`; the executor sends the reply during spin.
    using TypedServiceFn = void (*)(const RequestType& request, ResponseType& response);
    using TypedServiceFnWithCtx = void (*)(const RequestType& request, ResponseType& response,
                                           void* ctx);

    /// Try to receive a typed request (non-blocking).
    ///
    /// @param req     Output request struct (filled on success).
    /// @param seq_id  Output sequence number for reply matching.
    /// @return Result::success() if a request was received and deserialized;
    ///         ErrorCode::TryAgain if no data is available;
    ///         ErrorCode::NotInitialized or the FFI error code otherwise;
    ///         ErrorCode::Error if deserialization failed.
    Result take_request(RequestType& req, int64_t& seq_id) {
        return try_recv_request_sized<::nros::rx_buffer_capacity<RequestType>::value>(req, seq_id);
    }

    /// @ref take_request with the receive buffer sized by the CALLER.
    ///
    /// This IS a receive buffer — issue 0964's survey listed the service
    /// request under "transmit", which is true of `Client<S>`'s request and
    /// false here: the server deserializes out of it, so an under-estimate
    /// truncates. See @ref Subscription::take_sized.
    template <size_t Cap> Result try_recv_request_sized(RequestType& req, int64_t& seq_id) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[Cap];
        size_t len = 0;
        int64_t seq = 0;
        nros_cpp_ret_t ret =
            nros_cpp_service_server_take_request_raw(storage_, buf, sizeof(buf), &len, &seq);
        if (ret != 0) return Result(ret);
        if (len == 0) return Result(ErrorCode::TryAgain);
        if (RequestType::ffi_deserialize(buf, len, &req) != 0) return Result(ErrorCode::Error);
        seq_id = seq;
        return Result::success();
    }

    /// @deprecated Use `take_request(RequestType&, int64_t&)`.
    ///
    /// phase-379 W6 decision 1 (2026-09-03): `take` -> `take`. rcl
    /// (`rcl_take_request`), rclcpp (`Service::take_request`) and our own RMW
    /// vtable (`take_request`) already said `take`; only this layer said
    /// `take`. Header-only forwarder, no ABI cost. Scheduled for removal.
    [[deprecated("Service::try_recv_request is deprecated; use Service::take_request")]] Result
    try_recv_request(RequestType& req, int64_t& seq_id) {
        return take_request(req, seq_id);
    }

    /// Send a typed reply to a previously received request.
    ///
    /// @param seq_id  Sequence number from take_request().
    /// @param resp    Response to send.
    /// @return Result indicating success or failure.
    Result send_response(int64_t seq_id, const ResponseType& resp) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[::nros::detail::buffer_bounds<ResponseType>::tx];
        size_t len = 0;
        if (ResponseType::ffi_serialize(&resp, buf, sizeof(buf), &len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(nros_cpp_service_server_send_response_raw(storage_, seq_id, buf, len));
    }

    /// @deprecated Use `send_response()`.
    ///
    /// Phase-379 W5: rcl, rclcpp and rclrs all say `send_response`, and our
    /// own C already used that word. Kept as a forwarder so an out-of-tree
    /// node on the old spelling still compiles and is told what to move to.
    [[deprecated("Service::send_reply() is deprecated; use Service::send_response()")]] Result
    send_reply(int64_t seq_id, const ResponseType& resp) {
        return send_response(seq_id, resp);
    }

    /// Check if the service is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Destructor — releases service server resources.
    ///
    /// Poll-style services own an `RmwServiceServer` in `storage_` and free it
    /// here. Callback-style services (Phase 189.M3.3.e) are owned by the executor
    /// arena (freed when the executor drops), so the dtor must NOT touch
    /// `storage_` for them.
    ~Service() {
        if (initialized_ && !callback_mode_) {
            nros_cpp_service_server_destroy(storage_);
        }
        initialized_ = false;
    }

    // Move semantics (non-copyable). Poll-style relocation goes through the
    // `nros_cpp_service_server_relocate` runtime call (Phase 84.C1). A
    // callback-style service must NOT be moved after register — the arena holds
    // `this` as the trampoline context (Phase 189.M3.3.e); the move only
    // transfers bookkeeping and leaves that pointer stale, so don't.
    Service(Service&& other)
        : initialized_(other.initialized_), user_fn_(other.user_fn_),
          user_fn_ctx_(other.user_fn_ctx_), user_ctx_(other.user_ctx_),
          handle_id_(other.handle_id_), callback_mode_(other.callback_mode_) {
        if (other.initialized_ && !other.callback_mode_) {
            nros_cpp_service_server_relocate(other.storage_, storage_);
        }
        other.initialized_ = false;
    }

    Service& operator=(Service&& other) {
        if (this != &other) {
            if (initialized_ && !callback_mode_) {
                nros_cpp_service_server_destroy(storage_);
            }
            initialized_ = other.initialized_;
            user_fn_ = other.user_fn_;
            user_fn_ctx_ = other.user_fn_ctx_;
            user_ctx_ = other.user_ctx_;
            handle_id_ = other.handle_id_;
            callback_mode_ = other.callback_mode_;
            if (other.initialized_ && !other.callback_mode_) {
                nros_cpp_service_server_relocate(other.storage_, storage_);
            }
            other.initialized_ = false;
        }
        return *this;
    }

    /// Default constructor — creates an uninitialized service server.
    /// Use `Node::create_service()` to initialize.
    Service() : storage_(), initialized_(false) {}

    /// Executor handle for the callback-style service (Phase 189.M3.3.e);
    /// `SIZE_MAX` for poll-style / uninitialized.
    size_t handle_id() const { return handle_id_; }

  private:
    Service(const Service&) = delete;
    Service& operator=(const Service&) = delete;

    friend class Node;

    /// Phase 189.M3.3.e — raw request trampoline matching `RawServiceCallback`
    /// (`bool(req, req_len, resp, resp_cap, resp_len, ctx)`). Deserializes the
    /// request, runs the user's typed handler, serializes the response. `ctx` is
    /// the `Service` object (`this`).
    static bool request_trampoline(const uint8_t* req, size_t req_len, uint8_t* resp,
                                   size_t resp_cap, size_t* resp_len, void* ctx) {
        auto* self = static_cast<Service*>(ctx);
        if (self == nullptr) return false;
        RequestType request;
        if (RequestType::ffi_deserialize(req, req_len, &request) != 0) return false;
        ResponseType response;
        if (self->user_fn_ != nullptr) {
            self->user_fn_(request, response);
        } else if (self->user_fn_ctx_ != nullptr) {
            self->user_fn_ctx_(request, response, self->user_ctx_);
        } else {
            return false;
        }
        size_t len = 0;
        if (ResponseType::ffi_serialize(&response, resp, resp_cap, &len) != 0) return false;
        *resp_len = len;
        return true;
    }

    alignas(8) uint8_t storage_[NROS_SERVICE_SERVER_SIZE];
    bool initialized_;
    // Callback-style state (Phase 189.M3.3.e); unused in poll mode.
    TypedServiceFn user_fn_ = nullptr;
    TypedServiceFnWithCtx user_fn_ctx_ = nullptr;
    void* user_ctx_ = nullptr;
    size_t handle_id_ = static_cast<size_t>(-1);
    bool callback_mode_ = false;
};

} // namespace nros

// Phase 84.G8: out-of-line definition of Node::create_service<S>().
#include "nros/node.hpp"

namespace nros {

template <typename S>
Result Node::create_service(Service<S>& out, const char* service_name, const QoS& qos) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);
    nros_cpp_ret_t ret = nros_cpp_service_server_create(
        &handle_, service_name, S::TYPE_NAME, S::Request::TYPE_HASH, ffi_qos, out.storage_);
    if (ret == 0) {
        out.initialized_ = true;
    }
    return Result(ret);
}

// Phase 189.M3.3.e — callback-style (arena-registered) service. The arena owns
// the server + dispatches `out`'s request handler during spin_once, so the
// handle is real and `options.sched_context` is functional.
template <typename S, typename F, typename>
Result Node::create_service(Service<S>& out, const char* service_name, F callback, const QoS& qos,
                            const ServiceOptions& options) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);

    // Store the user handler (compile error if F isn't convertible).
    out.user_fn_ = typename Service<S>::TypedServiceFn(callback);
    out.user_fn_ctx_ = nullptr;
    out.user_ctx_ = nullptr;

    uint8_t sched = (options.sched_context == SCHED_CONTEXT_UNSET)
                        ? 0u
                        : static_cast<uint8_t>(options.sched_context);
    size_t handle = static_cast<size_t>(-1);
    nros_cpp_ret_t ret = nros_cpp_service_server_register(
        &handle_, service_name, S::TYPE_NAME, S::Request::TYPE_HASH, ffi_qos,
        reinterpret_cast<nros_cpp_service_request_callback_t>(&Service<S>::request_trampoline),
        &out, sched, &handle);
    if (ret == 0) {
        out.handle_id_ = handle;
        out.callback_mode_ = true;
        out.initialized_ = true;
    }
    return Result(ret);
}

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0089 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0089 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// `rclcpp::Service<S>::SharedPtr` — see `publisher.hpp` for why the alias
// template is the whole adoption. The upstream shared_ptr CALLBACK shape is
// REFUSE-LOUD, and the refusal lives on `rclcpp::Node::create_service`
// (`nros/nros.hpp`) because that is the call site it fires at; its diagnostic
// is `NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK` in `log.hpp`.
namespace rclcpp {
template <typename S> using Service = ::nros::Service<S>;
} // namespace rclcpp

#endif // NROS_CPP_SERVICE_HPP
