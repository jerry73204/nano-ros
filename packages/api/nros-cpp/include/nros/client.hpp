// nros-cpp: Service client class
// Freestanding C++ -- no exceptions, no STL required

/**
 * @file client.hpp
 * @ingroup grp_service
 * @brief `nros::Client<S>` — typed service client.
 */

#ifndef NROS_CPP_CLIENT_HPP
#define NROS_CPP_CLIENT_HPP

#include <cstdint>
#include <cstddef>

#include "nros/config.hpp"
#include "nros/result.hpp"
#include "nros/size_bound.hpp" // nros::rx_buffer_capacity<M> — the receive-buffer size
#include "nros/future.hpp"

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

// Phase 189.M3.3.f — `nros_cpp_service_client_register` is excluded from
// cbindgen (its Rust signature uses `RawResponseCallback`, an external-crate
// type alias). Declare it locally with a matching fn-ptr typedef.
// (`nros_cpp_service_client_send_on_handle` takes no callback, so it comes from
// the cbindgen header.)
extern "C" {
typedef void (*nros_cpp_service_response_callback_t)(const uint8_t* data, size_t len, void* ctx);

nros_cpp_ret_t nros_cpp_service_client_register(const nros_cpp_node_t* node,
                                                const char* service_name, const char* type_name,
                                                const char* type_hash, nros_cpp_qos_t qos,
                                                nros_cpp_service_response_callback_t callback,
                                                void* context, uint8_t sched_context,
                                                size_t* out_handle_id);
} // extern "C"

namespace nros {

/// Typed service client for a ROS 2 service.
///
/// Mirrors `rclcpp::Client<S>`. The service type `S` must provide
/// nested `Request` and `Response` types with `TYPE_NAME`, `TYPE_HASH`,
/// `SERIALIZED_SIZE_MAX`, `ffi_serialize()`, and `ffi_deserialize()`.
///
/// Usage (async -- preferred):
/// ```cpp
/// nros::Client<example_interfaces::srv::AddTwoInts> client;
/// NROS_TRY(node.create_client(client, "/add_two_ints"));
/// auto fut = client.send_request(req);
/// ResponseType resp;
/// NROS_TRY(fut.wait(executor.handle(), 5000, resp));
/// ```
template <typename S> class Client {
  public:
#ifdef NROS_CPP_HAS_SHARED_PTR
    /// `rclcpp::Client<S>::SharedPtr` — phase-417 W1.a.
    ///
    /// rclcpp indexes its entity types this way, and
    /// `rclcpp::Client<S>::SharedPtr member_;` is close to universal in
    /// ported source. Ergonomics only (RFC-0087 §"Who implements an adopted
    /// name"): a spelling for `std::shared_ptr<Client<S>>`, no second code path.
    ///
    /// Present only where `<memory>` is — a freestanding target has no
    /// `std::shared_ptr` to alias.
    using SharedPtr = std::shared_ptr<Client<S>>;
    /// `rclcpp::Client<S>::ConstSharedPtr` — see `SharedPtr`.
    using ConstSharedPtr = std::shared_ptr<const Client<S>>;
    /// `rclcpp::Client<S>::UniquePtr` — see `SharedPtr`.
    using UniquePtr = std::unique_ptr<Client<S>>;
#endif

    using RequestType = typename S::Request;
    using ResponseType = typename S::Response;

    /// Phase 189.M3.3.f — typed response-handler signatures for the
    /// *callback-style* client (rclcpp async dispatch). The handler runs during
    /// `spin_once` when a reply arrives for a request sent via
    /// `async_send_request`.
    using TypedResponseFn = void (*)(const ResponseType& response);
    using TypedResponseFnWithCtx = void (*)(const ResponseType& response, void* ctx);

    /// Send a request and return a Future for the response (non-blocking).
    ///
    /// Call `wait()` on the returned future to block until the response
    /// arrives, or poll with `is_ready()` / `try_take()`.
    ///
    /// @param req  Request to send.
    /// @return Future that resolves to the response. Returns a consumed
    ///         (empty) future on serialization or send failure.
    Future<ResponseType> send_request(const RequestType& req) {
        return send_request_sized<::nros::rx_buffer_capacity<ResponseType>::value>(req);
    }

    /// @ref send_request with the REPLY buffer sized by the caller.
    ///
    /// The receive buffer of a `Future<T>` is a member, so the capacity is a
    /// class template argument rather than a function one: this returns a
    /// `Future<ResponseType, RespCap>` (issue 0964). The request buffer is a
    /// transmit scratch buffer and is deliberately left on the estimate --
    /// over-sizing there only wastes stack.
    ///
    /// @tparam RespCap  Stack bytes the returned future holds for the reply.
    template <size_t RespCap>
    Future<ResponseType, RespCap> send_request_sized(const RequestType& req) {
        using Fut = Future<ResponseType, RespCap>;
        if (!initialized_) return Fut();

        uint8_t req_buf[::nros::detail::buffer_bounds<RequestType>::tx];
        size_t req_len = 0;
        if (RequestType::ffi_serialize(&req, req_buf, sizeof(req_buf), &req_len) != 0) {
            return Fut();
        }

        nros_cpp_ret_t ret = nros_cpp_service_client_send_request(storage_, req_buf, req_len);
        if (ret != 0) return Fut();

        return Fut(storage_, &nros_cpp_service_client_take_response,
                   0 // slot 0 (single outstanding request)
        );
    }

    /// Send a request and block until a reply is received.
    ///
    /// Spins the executor internally (like the runtime's `Promise::wait`).
    /// Never calls `zpico_get` — all I/O is driven by `spin_once`.
    ///
    /// @param req          Request to send.
    /// @param resp         Output response struct (filled on success).
    /// @param timeout_ms   Maximum wait time (default 5000ms).
    /// @return Result indicating success, timeout, or failure.
    Result call(const RequestType& req, ResponseType& resp, uint32_t timeout_ms = 5000) {
        return call_sized<::nros::rx_buffer_capacity<ResponseType>::value>(req, resp, timeout_ms);
    }

    /// @ref call with the REPLY buffer sized by the caller (issue 0964).
    template <size_t RespCap>
    Result call_sized(const RequestType& req, ResponseType& resp, uint32_t timeout_ms = 5000) {
        if (!initialized_ || !executor_) return Result(ErrorCode::NotInitialized);
        auto fut = send_request_sized<RespCap>(req);
        return fut.wait(executor_, timeout_ms, resp);
    }

    /// Issue 0278 (Half B) — send a request and block up to `timeout_ms` for the
    /// reply WITHOUT spinning the executor, so this is safe to call from inside
    /// a subscription/timer callback (where `call()`/`Future::wait` would return
    /// `Reentrant`, issue 0290). It sends then sleep-polls the reply queue.
    ///
    /// CONSTRAINT: usable from a callback only on a MULTI-THREADED backend
    /// (zenoh MT, cyclonedds), where the backend's own read task delivers the
    /// reply into the client's queue while this loop yields. On a
    /// single-threaded / polled backend the reply can only arrive via
    /// `spin_once` — which the callback is blocking — so it will TIME OUT; use
    /// `call()` from the main loop there. Keep `timeout_ms` SHORT (tens of ms):
    /// this blocks the executor's dispatch thread for its duration.
    ///
    /// @param req          Request to send.
    /// @param resp         Output response struct (filled on success).
    /// @param timeout_ms   Maximum wait (default 100ms).
    /// @return success on a received reply; ErrorCode::Timeout on no reply in
    ///         time; NotInitialized / Error otherwise.
    Result call_polling(const RequestType& req, ResponseType& resp, uint32_t timeout_ms = 100) {
        return call_polling_sized<::nros::rx_buffer_capacity<ResponseType>::value>(req, resp,
                                                                                   timeout_ms);
    }

    /// @ref call_polling with the REPLY buffer sized by the caller (issue
    /// 0964). The request buffer stays on the estimate: it is transmit
    /// scratch, where an over-estimate only wastes stack.
    template <size_t RespCap>
    Result call_polling_sized(const RequestType& req, ResponseType& resp,
                              uint32_t timeout_ms = 100) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t req_buf[::nros::detail::buffer_bounds<RequestType>::tx];
        size_t req_len = 0;
        if (RequestType::ffi_serialize(&req, req_buf, sizeof(req_buf), &req_len) != 0) {
            return Result(ErrorCode::Error);
        }
        uint8_t resp_buf[RespCap];
        size_t resp_len = 0;
        nros_cpp_ret_t ret = nros_cpp_service_client_call_raw(
            storage_, req_buf, req_len, resp_buf, sizeof(resp_buf), &resp_len, timeout_ms);
        if (ret != 0) return Result(ret);
        if (resp_len == 0) return Result(ErrorCode::Timeout);
        if (ResponseType::ffi_deserialize(resp_buf, resp_len, &resp) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result::success();
    }

    /// Check if the client is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Phase 124.C.3 — graph-aware "is the matching server up?" probe.
    ///
    /// Returns the count from the RMW backend's matched-server view:
    /// * `1`  — at least one matching server is currently visible.
    /// * `ok(false)` — no matching server discovered yet.
    /// * `error(Unsupported)` — backend cannot answer (e.g. XRCE without
    ///           participant enumeration); caller must fall back to a timed
    ///           `wait_for_service` or assume reachability.
    /// * `error(<code>)` — the probe itself failed.
    ///
    /// Never spins the executor — synchronous, safe to call from
    /// inside callbacks. Mirrors `rclcpp::ClientBase::service_is_ready`
    /// but with a tri-state result instead of collapsing
    /// "don't know" and "no" into the same `false`.
    Expected<bool> service_is_ready() const {
        if (!initialized_) return Expected<bool>::error(ErrorCode::NotInitialized);
        int out = -1;
        nros_cpp_ret_t ret =
            nros_cpp_service_client_server_available(const_cast<uint8_t*>(storage_), &out);
        // A failed CALL and a backend that cannot ANSWER are different facts,
        // and the old `int` form reported both as `-1`. Keep them apart.
        if (ret != 0) return Expected<bool>::error(static_cast<ErrorCode>(ret));
        if (out < 0) return Expected<bool>::error(ErrorCode::Unsupported);
        return Expected<bool>::ok(out != 0);
    }

    /// @deprecated Use `service_is_ready()`.
    ///
    /// phase-379 W6 — preserved exactly: `1` ready, `0` not yet, `-1` cannot
    /// answer. It cannot distinguish a failed call from an unsupported backend,
    /// which is why it is replaced rather than kept.
    [[deprecated("Client::server_available is deprecated; use "
                 "Client::service_is_ready, which returns Expected<bool>")]] int
    server_available() const {
        auto r = service_is_ready();
        if (!r.ok()) return -1;
        return r.value() ? 1 : 0;
    }

    /// phase-338 W8 — block until a matching service server is discoverable.
    ///
    /// Mirrors `rclcpp::ClientBase::wait_for_service`. Prefer this over
    /// hand-rolling a retry loop around the first `call()` / `send_request()`:
    /// it waits for the actual condition instead of guessing an attempt count,
    /// and it re-probes, so a server that starts AFTER the wait begins is still
    /// seen (a single liveliness query samples the router's current token list
    /// and terminates).
    ///
    /// Spins the executor cooperatively while probing, so do NOT call it from
    /// inside a callback — use the non-blocking `server_available()` there.
    ///
    /// Returns ok when the server is visible, `Timeout` when the budget
    /// elapses.
    Result wait_for_service(uint32_t timeout_ms = 5000) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_service_client_wait_for_service(storage_, executor_, timeout_ms));
    }

    /// Phase 189.M3.3.f — callback-style async send. Only valid on a
    /// callback-style client (created via the `create_client(out, name, callback,
    /// ...)` overload); the reply is delivered to the registered response handler
    /// during `spin_once` (no Future). Returns immediately after sending.
    Result async_send_request(const RequestType& req) {
        if (!initialized_ || !callback_mode_) return Result(ErrorCode::NotInitialized);
        uint8_t req_buf[::nros::detail::buffer_bounds<RequestType>::tx];
        size_t req_len = 0;
        if (RequestType::ffi_serialize(&req, req_buf, sizeof(req_buf), &req_len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(
            nros_cpp_service_client_send_on_handle(executor_, handle_id_, req_buf, req_len));
    }

    /// Executor handle for the callback-style client (Phase 189.M3.3.f);
    /// `SIZE_MAX` for future-style / uninitialized.
    size_t handle_id() const { return handle_id_; }

    /// Destructor -- releases service client resources.
    ///
    /// Future-style clients own an `RmwServiceClient` in `storage_`; callback-style
    /// clients (M3.3.f) are owned by the executor arena, so the dtor must NOT
    /// touch `storage_` for them.
    ~Client() {
        if (initialized_ && !callback_mode_) {
            nros_cpp_service_client_destroy(storage_);
        }
        initialized_ = false;
    }

    // Move semantics (non-copyable). Future-style relocation goes through the
    // `nros_cpp_service_client_relocate` runtime call (Phase 84.C1). A
    // callback-style client must NOT be moved after register — the arena holds
    // `this` as the response trampoline context (M3.3.f).
    Client(Client&& other)
        : executor_(other.executor_), initialized_(other.initialized_), user_fn_(other.user_fn_),
          user_fn_ctx_(other.user_fn_ctx_), user_ctx_(other.user_ctx_),
          handle_id_(other.handle_id_), callback_mode_(other.callback_mode_) {
        if (other.initialized_ && !other.callback_mode_) {
            nros_cpp_service_client_relocate(other.storage_, storage_);
        }
        other.initialized_ = false;
    }

    Client& operator=(Client&& other) {
        if (this != &other) {
            if (initialized_ && !callback_mode_) {
                nros_cpp_service_client_destroy(storage_);
            }
            executor_ = other.executor_;
            initialized_ = other.initialized_;
            user_fn_ = other.user_fn_;
            user_fn_ctx_ = other.user_fn_ctx_;
            user_ctx_ = other.user_ctx_;
            handle_id_ = other.handle_id_;
            callback_mode_ = other.callback_mode_;
            if (other.initialized_ && !other.callback_mode_) {
                nros_cpp_service_client_relocate(other.storage_, storage_);
            }
            other.initialized_ = false;
        }
        return *this;
    }

    /// Default constructor -- creates an uninitialized service client.
    /// Use `Node::create_client()` to initialize.
    Client() : storage_(), executor_(nullptr), initialized_(false) {}

  private:
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    friend class Node;

    /// Phase 189.M3.3.f — raw response trampoline matching `RawResponseCallback`
    /// (`void(data, len, ctx)`). Deserializes the reply, runs the user's typed
    /// handler. `ctx` is the `Client` object (`this`).
    static void response_trampoline(const uint8_t* data, size_t len, void* ctx) {
        auto* self = static_cast<Client*>(ctx);
        if (self == nullptr) return;
        ResponseType response;
        if (ResponseType::ffi_deserialize(data, len, &response) != 0) return;
        if (self->user_fn_ != nullptr) {
            self->user_fn_(response);
        } else if (self->user_fn_ctx_ != nullptr) {
            self->user_fn_ctx_(response, self->user_ctx_);
        }
    }

    alignas(8) uint8_t storage_[NROS_SERVICE_CLIENT_SIZE];
    void* executor_;
    bool initialized_;
    // Callback-style state (Phase 189.M3.3.f); unused in future mode.
    TypedResponseFn user_fn_ = nullptr;
    TypedResponseFnWithCtx user_fn_ctx_ = nullptr;
    void* user_ctx_ = nullptr;
    size_t handle_id_ = static_cast<size_t>(-1);
    bool callback_mode_ = false;
};

} // namespace nros

// Phase 84.G8: out-of-line definition of Node::create_client<S>().
#include "nros/node.hpp"

namespace nros {

template <typename S>
Result Node::create_client(Client<S>& out, const char* service_name, const QoS& qos) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);
    nros_cpp_ret_t ret = nros_cpp_service_client_create(
        &handle_, service_name, S::TYPE_NAME, S::Request::TYPE_HASH, ffi_qos, out.storage_);
    if (ret == 0) {
        out.executor_ = executor_handle_;
        out.initialized_ = true;
    }
    return Result(ret);
}

// Phase 189.M3.3.f — callback-style (arena-registered) client. The arena owns
// the client + dispatches `out`'s response handler during spin_once; requests
// go through `async_send_request`. `options.sched_context` is functional.
template <typename S, typename F, typename>
Result Node::create_client(Client<S>& out, const char* service_name, F callback, const QoS& qos,
                           const ClientOptions& options) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);

    out.user_fn_ = typename Client<S>::TypedResponseFn(callback);
    out.user_fn_ctx_ = nullptr;
    out.user_ctx_ = nullptr;

    uint8_t sched = (options.sched_context == SCHED_CONTEXT_UNSET)
                        ? 0u
                        : static_cast<uint8_t>(options.sched_context);
    size_t handle = static_cast<size_t>(-1);
    nros_cpp_ret_t ret = nros_cpp_service_client_register(
        &handle_, service_name, S::TYPE_NAME, S::Request::TYPE_HASH, ffi_qos,
        reinterpret_cast<nros_cpp_service_response_callback_t>(&Client<S>::response_trampoline),
        &out, sched, &handle);
    if (ret == 0) {
        out.executor_ = executor_handle_;
        out.handle_id_ = handle;
        out.callback_mode_ = true;
        out.initialized_ = true;
    }
    return Result(ret);
}

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0087 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0087 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// `rclcpp::Client<S>::SharedPtr` — see `publisher.hpp`. As with `Service`, the
// upstream shared_ptr callback shape is refused on
// `rclcpp::Node::create_client` (`nros/nros.hpp`).
namespace rclcpp {
template <typename S> using Client = ::nros::Client<S>;
} // namespace rclcpp

#endif // NROS_CPP_CLIENT_HPP
