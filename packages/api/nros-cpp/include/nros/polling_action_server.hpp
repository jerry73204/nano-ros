// nros-cpp: Polling (L1) action server class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file polling_action_server.hpp
 * @ingroup grp_action
 * @brief `nros::PollingActionServer<A>` — caller-polls action server.
 *
 * Phase 122.3.d.b — typed C++ wrapper over the L1 polling FFI added
 * in 122.3.d.a. Mirrors `ActionServer<A>` but drops the executor
 * registration / callback model: the caller drives `take_*` /
 * `accept_goal` / `complete_goal` / `try_handle_get_result` from a
 * spin loop. Used by RTIC / embassy / FreeRTOS-task-per-entity
 * patterns where the user owns scheduling.
 */

#ifndef NROS_CPP_POLLING_ACTION_SERVER_HPP
#define NROS_CPP_POLLING_ACTION_SERVER_HPP

#include <cstddef>
#include <cstdint>
#include <string.h>

#include "nros/action_server.hpp" // GoalResponse / CancelResponse / GoalStatus
#include "nros/config.hpp"
#include "nros/node.hpp"
#include "nros/nros_cpp_config_generated.h"
#include "nros/result.hpp"
#include "nros/size_bound.hpp" // nros::rx_buffer_capacity<M> — the receive-buffer size

#include "nros_cpp_ffi.h"

namespace nros {

/// Typed L1 polling-mode action server.
///
/// Caller drives the lifecycle directly — no executor callback.
/// Usage:
/// ```cpp
/// using Fib = example_interfaces::action::Fibonacci;
/// nros::PollingActionServer<Fib> srv;
/// NROS_TRY(node.create_polling_action_server(srv, "/fibonacci"));
/// while (running) {
///     uint8_t goal_id[16]; int64_t seq;
///     typename Fib::Goal goal;
///     if (srv.try_recv_goal_request(goal_id, goal, seq).ok()) {
///         srv.accept_goal(goal_id, seq);
///         // ... execute, publish feedback, eventually complete:
///         srv.complete_goal(goal_id, nros::GoalStatus::Succeeded, result);
///     }
///     srv.try_handle_get_result();
/// }
/// ```
template <typename A> class PollingActionServer {
  public:
    static constexpr size_t ACTION_NAME_MAX = 256;

    using GoalType = typename A::Goal;
    using ResultType = typename A::Result;
    using FeedbackType = typename A::Feedback;

    PollingActionServer() : storage_{}, action_name_{}, initialized_(false) {}

    ~PollingActionServer() {
        if (initialized_) {
            nros_cpp_action_server_destroy_polling(storage_);
            initialized_ = false;
        }
    }

    PollingActionServer(const PollingActionServer&) = delete;
    PollingActionServer& operator=(const PollingActionServer&) = delete;

    bool is_valid() const { return initialized_; }
    const char* get_action_name() const { return initialized_ ? action_name_ : ""; }

    /// Try to receive a goal request. On success fills `goal_id`
    /// (16 bytes), deserializes the goal into `out_goal`, and writes
    /// the matching `out_sequence_number`.
    Result try_recv_goal_request(uint8_t goal_id[16], GoalType& out_goal,
                                 int64_t& out_sequence_number) {
        return try_recv_goal_request_sized<::nros::rx_buffer_capacity<GoalType>::value>(
            goal_id, out_goal, out_sequence_number);
    }

    /// @ref try_recv_goal_request writing the id into a `GoalUUID` value
    /// (phase-417 W4.b) — the shape you can key a `{goal -> state}` map on.
    Result try_recv_goal_request(GoalUUID& goal_id, GoalType& out_goal,
                                 int64_t& out_sequence_number) {
        return try_recv_goal_request_sized<::nros::rx_buffer_capacity<GoalType>::value>(
            goal_id.data(), out_goal, out_sequence_number);
    }

    /// @ref try_recv_goal_request_sized taking a `GoalUUID` value.
    template <size_t Cap>
    Result try_recv_goal_request_sized(GoalUUID& goal_id, GoalType& out_goal,
                                       int64_t& out_sequence_number) {
        return try_recv_goal_request_sized<Cap>(goal_id.data(), out_goal, out_sequence_number);
    }

    /// @ref try_recv_goal_request with the receive buffer sized by the CALLER.
    /// See @ref Subscription::try_recv_sized (issue 0964).
    template <size_t Cap>
    Result try_recv_goal_request_sized(uint8_t goal_id[16], GoalType& out_goal,
                                       int64_t& out_sequence_number) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[Cap];
        int32_t rc = nros_cpp_action_server_try_recv_goal_request_raw(
            storage_, buf, sizeof(buf), reinterpret_cast<uint8_t(*)[16]>(goal_id),
            &out_sequence_number);
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) return Result(ErrorCode::TryAgain);
        if (GoalType::ffi_deserialize(buf, static_cast<size_t>(rc), &out_goal) != 0)
            return Result(ErrorCode::Error);
        return Result::success();
    }

    /// Accept a goal received via `try_recv_goal_request`.
    Result accept_goal(const uint8_t goal_id[16], int64_t sequence_number) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_accept_goal_raw(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id), sequence_number));
    }

    /// @ref accept_goal taking a `GoalUUID` value (phase-417 W4.b).
    Result accept_goal(const GoalUUID& goal_id, int64_t sequence_number) {
        return accept_goal(goal_id.data(), sequence_number);
    }

    /// Reject a goal received via `try_recv_goal_request`.
    Result reject_goal(int64_t sequence_number) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_reject_goal_raw(storage_, sequence_number));
    }

    /// Publish a feedback message for an accepted goal.
    Result publish_feedback(const uint8_t goal_id[16], const FeedbackType& fb) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[::nros::detail::buffer_bounds<FeedbackType>::tx];
        size_t len = 0;
        if (FeedbackType::ffi_serialize(&fb, buf, sizeof(buf), &len) != 0)
            return Result(ErrorCode::Error);
        return Result(nros_cpp_action_server_publish_feedback_raw(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id), buf, len));
    }

    /// @ref publish_feedback taking a `GoalUUID` value (phase-417 W4.b).
    Result publish_feedback(const GoalUUID& goal_id, const FeedbackType& fb) {
        return publish_feedback(goal_id.data(), fb);
    }

    /// Mark a goal terminal with a typed result.
    Result complete_goal(const uint8_t goal_id[16], GoalStatus status, const ResultType& result) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[::nros::detail::buffer_bounds<ResultType>::tx];
        size_t len = 0;
        if (ResultType::ffi_serialize(&result, buf, sizeof(buf), &len) != 0)
            return Result(ErrorCode::Error);
        return Result(nros_cpp_action_server_complete_goal_raw(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id), static_cast<int32_t>(status),
            buf, len));
    }

    /// @ref complete_goal taking a `GoalUUID` value (phase-417 W4.b).
    Result complete_goal(const GoalUUID& goal_id, GoalStatus status, const ResultType& result) {
        return complete_goal(goal_id.data(), status, result);
    }

    /// Terminate `goal_id` as SUCCEEDED — phase-417 W4.b.
    ///
    /// The same three terminal verbs `ActionServer<A>` grew, for the same
    /// reason: C spells them `nros_action_succeed` / `_abort` / `_canceled`,
    /// Rust `ActionServerHandle::succeed` / `abort` / `canceled`, and
    /// rclcpp_action `goal_handle->succeed(result)`. Forwarders onto
    /// `complete_goal` — no state, no second code path.
    Result succeed(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Succeeded, result);
    }

    /// Terminate `goal_id` as ABORTED. See @ref succeed.
    Result abort(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Aborted, result);
    }

    /// Terminate `goal_id` as CANCELED. See @ref succeed.
    Result canceled(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Canceled, result);
    }

    /// @ref succeed taking a `GoalUUID` value.
    Result succeed(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Succeeded, result);
    }

    /// @ref abort taking a `GoalUUID` value.
    Result abort(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Aborted, result);
    }

    /// @ref canceled taking a `GoalUUID` value.
    Result canceled(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Canceled, result);
    }

    /// Phase 122.3.c.6.d — peek a pending cancel-goal request.
    /// On success fills `goal_id`, `out_sequence_number`,
    /// `out_current_status` (matches `nros::GoalStatus` discriminants).
    /// Returns Result::success() if a request was peeked,
    /// ErrorCode::TryAgain if none pending.
    Result try_recv_cancel_request(uint8_t goal_id[16], int64_t& out_sequence_number,
                                   GoalStatus& out_current_status) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        int8_t status_raw = 0;
        int32_t rc = nros_cpp_action_server_try_recv_cancel_request_raw(
            storage_, reinterpret_cast<uint8_t(*)[16]>(goal_id), &out_sequence_number, &status_raw);
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) return Result(ErrorCode::TryAgain);
        out_current_status = static_cast<GoalStatus>(status_raw);
        return Result::success();
    }

    /// @ref try_recv_cancel_request writing the id into a `GoalUUID` value
    /// (phase-417 W4.b).
    Result try_recv_cancel_request(GoalUUID& goal_id, int64_t& out_sequence_number,
                                   GoalStatus& out_current_status) {
        return try_recv_cancel_request(goal_id.data(), out_sequence_number, out_current_status);
    }

    /// Phase 122.3.c.6.d — reply to a previously-peeked cancel
    /// request. `return_code` is a `nros::CancelReturnCode`:
    /// 0 = Ok (one+ goals canceling), 1 = Rejected, 2 = UnknownGoal,
    /// 3 = GoalTerminated. `accepted` is a contiguous array of
    /// 16-byte goal IDs that will transition to CANCELING.
    ///
    /// Issue 0796 — this doc said `nros::CancelResponse`, which in C++ is the
    /// PER-GOAL Reject/Accept decision and has never had four values. Prefer
    /// the `CancelReturnCode` overload below; the `int8_t` one is kept because
    /// callers pass the raw byte they read off the wire.
    Result send_cancel_reply(int64_t sequence_number, int8_t return_code,
                             const uint8_t (*accepted)[16], size_t accepted_count) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_send_cancel_reply_raw(
            storage_, sequence_number, return_code, accepted, accepted_count));
    }

    /// Typed overload of `send_cancel_reply` (issue 0796).
    Result send_cancel_reply(int64_t sequence_number, CancelReturnCode return_code,
                             const uint8_t (*accepted)[16], size_t accepted_count) {
        return send_cancel_reply(sequence_number, static_cast<int8_t>(return_code), accepted,
                                 accepted_count);
    }

    /// @ref send_cancel_reply reading the accepted ids from an array of
    /// `GoalUUID` values (phase-417 W4.b) — the array you can actually build,
    /// now that a goal id is a value type. `GoalUUID` is standard-layout with
    /// a single `uint8_t[16]` member and the static_asserts beside its
    /// definition pin `sizeof`/`alignof`, so an array of them IS the
    /// contiguous `uint8_t[N][16]` the FFI reads.
    Result send_cancel_reply(int64_t sequence_number, CancelReturnCode return_code,
                             const GoalUUID* accepted, size_t accepted_count) {
        return send_cancel_reply(sequence_number, static_cast<int8_t>(return_code),
                                 reinterpret_cast<const uint8_t(*)[16]>(accepted), accepted_count);
    }

    /// Phase 122.3.c.6.e — caller-owned wake-state slot. One per
    /// (channel) pair; declare next to the server, pass into
    /// `set_*_wake_callback`. Must outlive the server.
    struct WakeState {
        alignas(8) uint64_t _opaque[2] = {0, 0};
    };

    /// Register a C callback fired when a goal request arrives.
    Result set_goal_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_set_goal_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    /// Register a C callback fired when a cancel-goal request arrives.
    Result set_cancel_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_set_cancel_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    /// Register a C callback fired when a get_result query arrives.
    Result set_get_result_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_server_set_get_result_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    /// Serve one pending get_result query (call from spin loop).
    /// `default_result` is returned to clients querying goals that
    /// haven't been completed yet.
    /// Returns Result::success() if one was served,
    /// ErrorCode::TryAgain if none pending.
    Result try_handle_get_result(const ResultType& default_result = ResultType{}) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[::nros::detail::buffer_bounds<ResultType>::tx];
        size_t len = 0;
        if (ResultType::ffi_serialize(&default_result, buf, sizeof(buf), &len) != 0)
            return Result(ErrorCode::Error);
        int32_t rc = nros_cpp_action_server_try_handle_get_result_raw(storage_, buf, len);
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) return Result(ErrorCode::TryAgain);
        return Result::success();
    }

  private:
    friend class Node;

    static constexpr size_t kStorageU64s = NROS_CPP_RAW_ACTION_SERVER_OPAQUE_U64S;
    alignas(8) uint64_t storage_[kStorageU64s];
    char action_name_[ACTION_NAME_MAX];
    bool initialized_;
};

} // namespace nros

#include "nros/node.hpp"

namespace nros {

template <typename A>
Result Node::create_polling_action_server(PollingActionServer<A>& out, const char* action_name) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_ret_t ret =
        nros_cpp_action_server_init_polling(&handle_, action_name, A::TYPE_NAME, A::Goal::TYPE_HASH,
                                            reinterpret_cast<void*>(out.storage_));
    if (ret != 0) return Result(ret);
    size_t name_len = 0;
    while (action_name[name_len] != '\0' && name_len + 1 < sizeof(out.action_name_)) {
        out.action_name_[name_len] = action_name[name_len];
        ++name_len;
    }
    out.action_name_[name_len] = '\0';
    out.initialized_ = true;
    return Result::success();
}

} // namespace nros

#endif // NROS_CPP_POLLING_ACTION_SERVER_HPP
