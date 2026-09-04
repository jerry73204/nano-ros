// nros-cpp: Action client class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file action_client.hpp
 * @ingroup grp_action
 * @brief `nros::ActionClient<A>` — typed action client.
 */

#ifndef NROS_CPP_ACTION_CLIENT_HPP
#define NROS_CPP_ACTION_CLIENT_HPP

#include <cstdint>
#include <cstddef>
#include <string.h>

#include "nros/config.hpp"
#include "nros/result.hpp"
#include "nros/size_bound.hpp" // nros::rx_buffer_capacity<M> — the receive-buffer size
#include "nros/future.hpp"
#include "nros/stream.hpp"
// Issue 0796 — `CancelReturnCode` (the `action_msgs/srv/CancelGoal` RPC status
// `cancel_goal` resolves to) is declared beside its per-goal sibling
// `CancelResponse` in action_server.hpp, which is also where
// polling_action_server.hpp reaches for them.
#include "nros/action_server.hpp"

// Phase 118.D: FFI declarations sourced from cbindgen-generated
// `nros_cpp_ffi.h`. cbindgen renders Rust `*const [u8; 16]` /
// `*mut [u8; 16]` parameters as C++ pointer-to-array
// (`const uint8_t (*goal_id)[16]`); member-function bodies below
// take `goal_id` as a 16-byte array (decays to `uint8_t*`) and
// `reinterpret_cast` at the call site to bridge the two ABI
// equivalent shapes.
#include "nros_cpp_ffi.h"

extern "C" {
typedef void (*nros_cpp_action_client_goal_response_callback_t)(bool accepted,
                                                                const uint8_t goal_id[16],
                                                                void* ctx);
typedef void (*nros_cpp_action_client_feedback_callback_t)(const uint8_t goal_id[16],
                                                           const uint8_t* data, size_t len,
                                                           void* ctx);
typedef void (*nros_cpp_action_client_result_callback_t)(const uint8_t goal_id[16], int32_t status,
                                                         const uint8_t* data, size_t len,
                                                         void* ctx);

nros_cpp_ret_t nros_cpp_action_client_set_callbacks(
    void* handle, nros_cpp_action_client_goal_response_callback_t goal_response,
    nros_cpp_action_client_feedback_callback_t feedback,
    nros_cpp_action_client_result_callback_t result, void* context);
} // extern "C"

namespace nros {

/// Typed action client for a ROS 2 action.
///
/// Mirrors `rclcpp_action::Client<A>`. The action type `A` must provide
/// nested `Goal`, `Result`, and `Feedback` types with `TYPE_NAME`, `TYPE_HASH`,
/// `SERIALIZED_SIZE_MAX`, `ffi_serialize()`, and `ffi_deserialize()`.
///
/// Usage:
/// ```cpp
/// nros::ActionClient<example_interfaces::action::Fibonacci> client;
/// NROS_TRY(node.create_action_client(client, "/fibonacci"));
/// typename decltype(client)::GoalType goal;
/// goal.order = 10;
/// uint8_t goal_id[16];
/// NROS_TRY(client.send_goal(goal, goal_id));
/// typename decltype(client)::ResultType result;
/// NROS_TRY(client.get_result(goal_id, result));
/// ```
template <typename A> class ActionClient {
  public:
    using GoalType = typename A::Goal;
    using ResultType = typename A::Result;
    using FeedbackType = typename A::Feedback;

    /// Goal acceptance response for the Future pattern.
    ///
    /// Returned by `send_goal_future()`. Contains the goal UUID and
    /// whether the server accepted the goal.
    struct GoalAccept {
        /// Kept a raw array: it is written through the FFI decoder below, and
        /// in-tree callers pass it straight on to the raw-array overloads.
        /// `GoalUUID(accept.goal_id)` is the one-token lift (phase-417 W4.b).
        uint8_t goal_id[16];
        bool accepted;

        // Receive-buffer size for `Future<GoalAccept>`; the payload is 17 bytes
        // (16-byte goal_id + 1 accepted byte), rounded up.
        static const size_t SERIALIZED_SIZE_MAX = 32;

        // Issue 0329 — the 17-byte wire layout is owned Rust-side
        // (`nros_cpp_action_goal_accept_decode`, beside its producer); the header
        // no longer hand-decodes it, so the two cannot drift.
        static int ffi_deserialize(const uint8_t* data, size_t len, GoalAccept* out) {
            if (!out) return -1;
            return nros_cpp_action_goal_accept_decode(data, len, &out->goal_id, &out->accepted) ==
                           NROS_CPP_RET_OK
                       ? 0
                       : -1;
        }
    };

    /// Send a goal and receive the generated goal UUID (blocking).
    ///
    /// Internally spins the executor until the server accepts or rejects
    /// the goal (Phase 82 compliant -- drives the executor).
    ///
    /// @param goal     Goal to send.
    /// @param goal_id  Output 16-byte goal UUID (filled on success).
    /// @return Result indicating success or failure.
    /// phase-338 W8 — block until the action server is discoverable.
    ///
    /// Mirrors `rclcpp_action::Client::wait_for_action_server`. Probes the
    /// `send_goal` queryable, which is the load-bearing entity for the first
    /// `send_goal()`. Prefer this over retrying `send_goal()` on timeout: it
    /// waits for the real condition and re-probes, so a server that comes up
    /// after the wait starts is still seen.
    ///
    /// Spins the executor while probing — not for use inside a callback.
    Result wait_for_action_server(uint32_t timeout_ms = 5000) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_wait_for_action_server(storage_, timeout_ms));
    }

    Result send_goal(const GoalType& goal, uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[::nros::detail::buffer_bounds<GoalType>::tx];
        size_t len = 0;
        if (GoalType::ffi_serialize(&goal, buf, sizeof(buf), &len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(nros_cpp_action_client_send_goal(storage_, buf, len,
                                                       reinterpret_cast<uint8_t(*)[16]>(goal_id)));
    }

    /// @ref send_goal writing the generated id into a `GoalUUID` value
    /// (phase-417 W4.b) — the shape you can put in a map.
    Result send_goal(const GoalType& goal, GoalUUID& goal_id) {
        return send_goal(goal, goal_id.data());
    }

    /// Get the result for a goal (blocking with timeout).
    ///
    /// Sends a get_result request and spins the executor until a reply
    /// arrives or timeout (Phase 82 compliant -- drives the executor).
    ///
    /// @param goal_id  16-byte goal UUID from send_goal().
    /// @param result   Output result struct (filled on success).
    /// @return Result indicating success, timeout, or failure.
    Result get_result(const uint8_t goal_id[16], ResultType& result) {
        return get_result_sized<::nros::rx_buffer_capacity<ResultType>::value>(goal_id, result);
    }

    /// @ref get_result taking a `GoalUUID` value (phase-417 W4.b).
    Result get_result(const GoalUUID& goal_id, ResultType& result) {
        return get_result_sized<::nros::rx_buffer_capacity<ResultType>::value>(goal_id.data(),
                                                                               result);
    }

    /// @ref get_result with the receive buffer sized by the CALLER.
    /// See @ref Subscription::try_recv_sized (issue 0964).
    template <size_t Cap> Result get_result_sized(const uint8_t goal_id[16], ResultType& result) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[Cap];
        size_t len = 0;
        nros_cpp_ret_t ret = nros_cpp_action_client_get_result(
            storage_, executor_, reinterpret_cast<const uint8_t(*)[16]>(goal_id), buf, sizeof(buf),
            &len);
        if (ret != 0) return Result(ret);

        if (ResultType::ffi_deserialize(buf, len, &result) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result::success();
    }

    // =================================================================
    // Future-based API — non-blocking, polled via Future<T>
    // =================================================================

    /// Send a goal and return a Future for the acceptance response.
    ///
    /// Returns immediately after sending the goal request. Poll the
    /// returned future (or call `wait()`) to get the `GoalAccept` result.
    ///
    /// Usage:
    /// ```cpp
    /// auto fut = client.send_goal_future(goal);
    /// GoalAccept accept;
    /// NROS_TRY(fut.wait(executor.handle(), 5000, accept));
    /// if (accept.accepted) { /* use accept.goal_id */ }
    /// ```
    ///
    /// @param goal  Goal to send.
    /// @return Future that resolves to GoalAccept. Returns a consumed
    ///         (empty) future on serialization or send failure.
    Future<GoalAccept> send_goal_future(const GoalType& goal) {
        if (!initialized_) return Future<GoalAccept>();

        uint8_t buf[::nros::detail::buffer_bounds<GoalType>::tx];
        size_t len = 0;
        if (GoalType::ffi_serialize(&goal, buf, sizeof(buf), &len) != 0) {
            return Future<GoalAccept>();
        }

        uint8_t goal_id[16];
        nros_cpp_ret_t ret = nros_cpp_action_client_send_goal_async(
            storage_, buf, len, reinterpret_cast<uint8_t(*)[16]>(goal_id));
        if (ret != 0) return Future<GoalAccept>();

        return Future<GoalAccept>(storage_, &nros_cpp_action_client_try_recv_goal_response,
                                  0 // slot 0 (single outstanding goal request)
        );
    }

    /// Request a goal result and return a Future for the result.
    ///
    /// Sends the get_result request asynchronously and returns a Future
    /// that resolves when the result arrives. Poll the future (or call
    /// `wait()`) to retrieve the deserialized result.
    ///
    /// Usage:
    /// ```cpp
    /// auto fut = client.get_result_future(goal_id);
    /// ResultType result;
    /// NROS_TRY(fut.wait(executor.handle(), 10000, result));
    /// ```
    ///
    /// @param goal_id  16-byte goal UUID from send_goal() or GoalAccept.
    /// @return Future that resolves to ResultType. Returns a consumed
    ///         (empty) future on send failure.
    Future<ResultType> get_result_future(const uint8_t goal_id[16]) {
        return get_result_future_sized<::nros::rx_buffer_capacity<ResultType>::value>(goal_id);
    }

    /// @ref get_result_future taking a `GoalUUID` value (phase-417 W4.b).
    Future<ResultType> get_result_future(const GoalUUID& goal_id) {
        return get_result_future_sized<::nros::rx_buffer_capacity<ResultType>::value>(
            goal_id.data());
    }

    /// @ref get_result_future with the RESULT buffer sized by the caller.
    ///
    /// A `Future<T>` holds its receive buffer as a member, so the capacity is a
    /// class template argument: this returns `Future<ResultType, Cap>`
    /// (issue 0964).
    template <size_t Cap>
    Future<ResultType, Cap> get_result_future_sized(const uint8_t goal_id[16]) {
        using Fut = Future<ResultType, Cap>;
        if (!initialized_) return Fut();

        nros_cpp_ret_t ret = nros_cpp_action_client_get_result_async(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id));
        if (ret != 0) return Fut();

        return Fut(storage_, &nros_cpp_action_client_try_recv_result,
                   0 // slot 0 (single outstanding result request)
        );
    }

    /// Try to receive feedback (non-blocking).
    ///
    /// @param feedback Output feedback struct (filled on success).
    /// @return Result::success() if feedback was received and deserialized;
    ///         ErrorCode::TryAgain if no feedback is available right now;
    ///         ErrorCode::NotInitialized if the client is not initialized;
    ///         ErrorCode::Error if deserialization failed; otherwise the
    ///         FFI error code.
    Result try_recv_feedback(FeedbackType& feedback) {
        return try_recv_feedback_sized<::nros::rx_buffer_capacity<FeedbackType>::value>(feedback);
    }

    /// @ref try_recv_feedback with the receive buffer sized by the CALLER.
    /// See @ref Subscription::try_recv_sized (issue 0964).
    template <size_t Cap> Result try_recv_feedback_sized(FeedbackType& feedback) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[Cap];
        size_t len = 0;
        nros_cpp_ret_t ret =
            nros_cpp_action_client_try_recv_feedback(storage_, buf, sizeof(buf), &len);
        if (ret != 0) return Result(ret);
        if (len == 0) return Result(ErrorCode::TryAgain);
        if (FeedbackType::ffi_deserialize(buf, len, &feedback) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result::success();
    }

    /// Get a reference to the action client's feedback stream.
    ///
    /// The stream yields `FeedbackType` values across all currently-active
    /// goals for this client — feedback is not goal-scoped at this layer.
    /// Callers that need per-goal separation should use the callback API
    /// (`set_callbacks(SendGoalOptions{ .feedback = … })`), which delivers
    /// `(goal_id, bytes, len, ctx)` via an executor-driven trampoline.
    ///
    /// Usage (blocking):
    /// ```cpp
    /// FeedbackType fb;
    /// NROS_TRY(client.feedback_stream().wait_next(executor.handle(), 500, fb));
    /// ```
    ///
    /// Usage (non-blocking):
    /// ```cpp
    /// FeedbackType fb;
    /// Result r = client.feedback_stream().try_next(fb);
    /// if (r.ok()) { ... }
    /// ```
    Stream<FeedbackType>& feedback_stream() {
        if (initialized_ && !feedback_stream_.is_valid()) {
            feedback_stream_.bind(storage_, &nros_cpp_action_client_try_recv_feedback);
        }
        return feedback_stream_;
    }

    const Stream<FeedbackType>& feedback_stream() const { return feedback_stream_; }

    // =================================================================
    // Async (non-blocking) API — callbacks invoked during spin_once()
    // =================================================================

    /// Options for async goal sending (mirrors rclcpp SendGoalOptions).
    ///
    /// Set callback pointers before calling send_goal_async(). Callbacks are
    /// invoked during spin_once() when the corresponding response arrives.
    /// All callbacks receive the context pointer for user state.
    struct SendGoalOptions {
        /// Called when the server accepts or rejects the goal.
        void (*goal_response)(bool accepted, const uint8_t goal_id[16], void* ctx);
        /// Called when feedback is received for the goal.
        void (*feedback)(const uint8_t goal_id[16], const uint8_t* data, size_t len, void* ctx);
        /// Called when the result is received.
        void (*result)(const uint8_t goal_id[16], int32_t status, const uint8_t* data, size_t len,
                       void* ctx);
        /// User context pointer passed to all callbacks.
        void* context;

        SendGoalOptions() : goal_response(0), feedback(0), result(0), context(0) {}
    };

    /// Send a goal asynchronously (non-blocking).
    ///
    /// Returns immediately after sending the goal request. The goal UUID
    /// is filled on success. Responses arrive via callbacks registered
    /// with the executor (see SendGoalOptions and Node::create_action_client).
    ///
    /// @param goal     Goal to send.
    /// @param goal_id  Output 16-byte goal UUID (filled on success).
    /// @return Result indicating success or failure.
    Result send_goal_async(const GoalType& goal, uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[::nros::detail::buffer_bounds<GoalType>::tx];
        size_t len = 0;
        if (GoalType::ffi_serialize(&goal, buf, sizeof(buf), &len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(nros_cpp_action_client_send_goal_async(
            storage_, buf, len, reinterpret_cast<uint8_t(*)[16]>(goal_id)));
    }

    /// @ref send_goal_async writing the generated id into a `GoalUUID` value
    /// (phase-417 W4.b).
    Result send_goal_async(const GoalType& goal, GoalUUID& goal_id) {
        return send_goal_async(goal, goal_id.data());
    }

    /// Cancel a goal (non-blocking) — issue 0796.
    ///
    /// Sends the `action_msgs/srv/CancelGoal` request and returns; read the
    /// RPC outcome with `try_recv_cancel_response()` once the executor has
    /// spun. Mirrors C's `nros_action_cancel_goal` and Rust's
    /// `ActionClient::cancel_goal` — rclcpp_action's `async_cancel_goal`
    /// returns a future, which RFC-0021 has no runtime to await.
    ///
    /// Until this existed, a C++ application written against the CALLBACK
    /// tier could start a goal and had no way to stop it: cancel was only on
    /// the L1 `PollingActionClient::send_cancel_request`.
    ///
    /// @param goal_id  16-byte goal UUID from send_goal() / send_goal_async().
    /// @return Result indicating the request was sent.
    Result cancel_goal(const uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_cancel_goal(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id)));
    }

    /// @ref cancel_goal taking a `GoalUUID` value (phase-417 W4.b).
    Result cancel_goal(const GoalUUID& goal_id) { return cancel_goal(goal_id.data()); }

    /// Try to read the reply to a `cancel_goal()` (non-blocking).
    ///
    /// @param out  Receives the RPC return code on success.
    /// @return Result::success() when a reply was consumed;
    ///         ErrorCode::TryAgain when none has arrived yet.
    Result try_recv_cancel_response(CancelReturnCode& out) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        int8_t code = 0;
        nros_cpp_ret_t ret = nros_cpp_action_client_try_recv_cancel_response(storage_, &code);
        if (ret != 0) return Result(ret);
        out = static_cast<CancelReturnCode>(code);
        return Result::success();
    }

    /// Request the result for a goal asynchronously (non-blocking).
    ///
    /// Returns immediately after sending the get_result request. The result
    /// arrives via the result callback during poll().
    ///
    /// @param goal_id  16-byte goal UUID from send_goal_async().
    /// @return Result indicating success or failure.
    Result get_result_async(const uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_get_result_async(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id)));
    }

    /// @ref get_result_async taking a `GoalUUID` value (phase-417 W4.b).
    Result get_result_async(const GoalUUID& goal_id) { return get_result_async(goal_id.data()); }

    /// Register async callbacks for goal response, feedback, and result.
    ///
    /// @param options  Callback pointers and context.
    /// @return Result::success() on success, ErrorCode::NotInitialized
    ///         if the client is not initialized, or the FFI error code.
    Result set_callbacks(const SendGoalOptions& options) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_set_callbacks(
            storage_, options.goal_response, options.feedback, options.result, options.context));
    }

    /// Poll for pending async replies (non-blocking).
    ///
    /// Checks for goal acceptance, feedback, and result replies.
    /// Invokes the corresponding callbacks registered via set_callbacks().
    /// Call this in the spin loop after spin_once().
    void poll() {
        if (!initialized_) return;
        nros_cpp_action_client_poll(storage_);
    }

    /// Check if the action client is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Destructor — releases action client resources.
    ~ActionClient() {
        if (initialized_) {
            nros_cpp_action_client_destroy(storage_);
            initialized_ = false;
        }
        feedback_stream_ = Stream<FeedbackType>();
    }

    // Move semantics (non-copyable). Relocation goes through the
    // `nros_cpp_action_client_relocate` runtime call (Phase 84.C1).
    // The feedback stream is rebound to the new storage afterwards.
    ActionClient(ActionClient&& other)
        : executor_(other.executor_), initialized_(other.initialized_) {
        if (other.initialized_) {
            nros_cpp_action_client_relocate(other.storage_, storage_);
            other.initialized_ = false;
        }
        other.feedback_stream_ = Stream<FeedbackType>();
    }

    ActionClient& operator=(ActionClient&& other) {
        if (this != &other) {
            if (initialized_) {
                nros_cpp_action_client_destroy(storage_);
                feedback_stream_ = Stream<FeedbackType>();
            }
            executor_ = other.executor_;
            initialized_ = other.initialized_;
            if (other.initialized_) {
                nros_cpp_action_client_relocate(other.storage_, storage_);
                other.initialized_ = false;
            }
            other.feedback_stream_ = Stream<FeedbackType>();
        }
        return *this;
    }

    /// Default constructor — creates an uninitialized action client.
    /// Use `Node::create_action_client()` to initialize.
    ActionClient() : executor_(nullptr), initialized_(false) {}

  private:
    ActionClient(const ActionClient&) = delete;
    ActionClient& operator=(const ActionClient&) = delete;

    friend class Node;

    alignas(8) uint8_t storage_[NROS_CPP_ACTION_CLIENT_STORAGE_SIZE];
    void* executor_; // Stashed executor handle (Phase 82) for blocking helpers
    bool initialized_;
    Stream<FeedbackType> feedback_stream_;
    // Phase 87.6 put a `char action_name_[256]` here for an accessor that was
    // never written; phase-417 W4.b deleted it. See the matching note in
    // `action_server.hpp` — populated at construction, read by nothing, and a
    // duplicate of a name the runtime already holds.
};

} // namespace nros

// Phase 84.G8: out-of-line definition of Node::create_action_client<A>().
#include "nros/node.hpp"

namespace nros {

template <typename A>
Result Node::create_action_client(ActionClient<A>& out, const char* action_name, const QoS& qos) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);
    nros_cpp_ret_t ret = nros_cpp_action_client_create(&handle_, action_name, A::TYPE_NAME,
                                                       A::Goal::TYPE_HASH, ffi_qos, out.storage_);
    if (ret == 0) {
        out.executor_ = executor_handle_;
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

// `rclcpp_action::Client<A>` — a type alias only; see `action_server.hpp`.
namespace rclcpp_action {
template <typename A> using Client = ::nros::ActionClient<A>;
} // namespace rclcpp_action

#endif // NROS_CPP_ACTION_CLIENT_HPP
