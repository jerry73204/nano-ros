// nros-cpp: Polling (L1) action client class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file polling_action_client.hpp
 * @ingroup grp_action
 * @brief `nros::PollingActionClient<A>` — caller-polls action client.
 *
 * Phase 122.3.d.b — typed C++ wrapper over the L1 polling FFI added
 * in 122.3.d.a. Mirrors `ActionClient<A>` but drops the executor
 * arena / callback model: the caller drives `send_goal_raw` +
 * `take_*` from a spin loop.
 */

#ifndef NROS_CPP_POLLING_ACTION_CLIENT_HPP
#define NROS_CPP_POLLING_ACTION_CLIENT_HPP

#include <cstddef>
#include <cstdint>
#include <string.h>

// phase-417 W4.b — `nros::GoalUUID`, the goal-id VALUE type, is declared
// beside the shared action enums in action_server.hpp (which is also where
// polling_action_server.hpp reaches for them).
#include "nros/action_server.hpp"
#include "nros/config.hpp"
#include "nros/node.hpp"
#include "nros/nros_cpp_config_generated.h"
#include "nros/result.hpp"
#include "nros/size_bound.hpp" // nros::rx_buffer_capacity<M> — the receive-buffer size

#include "nros_cpp_ffi.h"

namespace nros {

/// Typed L1 polling-mode action client.
///
/// Usage:
/// ```cpp
/// using Fib = example_interfaces::action::Fibonacci;
/// nros::PollingActionClient<Fib> cli;
/// NROS_TRY(node.create_polling_action_client(cli, "/fibonacci"));
/// uint8_t goal_id[16];
/// typename Fib::Goal g;  g.order = 10;
/// cli.send_goal(g, goal_id);
/// while (running) {
///     typename Fib::Feedback fb; uint8_t fid[16];
///     if (cli.try_recv_feedback(fid, fb).ok()) { /* … */ }
///     typename Fib::Result r;
///     if (cli.try_recv_result(r).ok()) { break; }
/// }
/// ```
template <typename A> class PollingActionClient {
  public:
    static constexpr size_t ACTION_NAME_MAX = 256;

    using GoalType = typename A::Goal;
    using ResultType = typename A::Result;
    using FeedbackType = typename A::Feedback;

    PollingActionClient() : storage_{}, action_name_{}, initialized_(false) {}

    ~PollingActionClient() {
        if (initialized_) {
            nros_cpp_action_client_destroy_polling(storage_);
            initialized_ = false;
        }
    }

    PollingActionClient(const PollingActionClient&) = delete;
    PollingActionClient& operator=(const PollingActionClient&) = delete;

    bool is_valid() const { return initialized_; }
    const char* get_action_name() const { return initialized_ ? action_name_ : ""; }

    /// Send a typed goal. Writes generated 16-byte UUID into
    /// `goal_id_out`.
    Result send_goal(const GoalType& goal, uint8_t goal_id_out[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[::nros::detail::buffer_bounds<GoalType>::tx];
        size_t len = 0;
        if (GoalType::ffi_serialize(&goal, buf, sizeof(buf), &len) != 0)
            return Result(ErrorCode::Error);
        return Result(nros_cpp_action_client_send_goal_raw(
            storage_, buf, len, reinterpret_cast<uint8_t(*)[16]>(goal_id_out)));
    }

    /// @ref send_goal writing the generated id into a `GoalUUID` value
    /// (phase-417 W4.b).
    Result send_goal(const GoalType& goal, GoalUUID& goal_id_out) {
        return send_goal(goal, goal_id_out.data());
    }

    /// Try to receive the send_goal RPC reply (accept / reject).
    /// Caller deserializes the wire CDR as needed; this template
    /// just returns the raw bytes count in `out_len`.
    Result try_recv_goal_response(uint8_t* buf, size_t cap, size_t& out_len) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        int32_t rc = nros_cpp_action_client_try_recv_goal_response_raw(storage_, buf, cap);
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) {
            out_len = 0;
            return Result(ErrorCode::TryAgain);
        }
        out_len = static_cast<size_t>(rc);
        return Result::success();
    }

    /// Issue a get_result request for the given goal.
    Result send_get_result_request(const uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_send_get_result_request_raw(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id)));
    }

    /// @ref send_get_result_request taking a `GoalUUID` value (phase-417 W4.b).
    Result send_get_result_request(const GoalUUID& goal_id) {
        return send_get_result_request(goal_id.data());
    }

    /// Try to receive the get_result reply as a typed result.
    /// The reply has wire layout: CDR header (4B) + status byte (1B) +
    /// result payload — deserializes the trailing payload into `out`.
    Result try_recv_result(ResultType& out) {
        return try_recv_result_sized<::nros::rx_buffer_capacity<ResultType>::value>(out);
    }

    /// @ref try_recv_result with the RESULT PAYLOAD capacity chosen by the
    /// caller. `Cap` is the payload, not the buffer: the wire reply carries a
    /// 5-byte prefix (CDR header + status), which this adds (issue 0964).
    /// See @ref Subscription::try_recv_sized.
    template <size_t Cap> Result try_recv_result_sized(ResultType& out) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[Cap + 5];
        int32_t rc = nros_cpp_action_client_try_recv_result_raw(storage_, buf, sizeof(buf));
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) return Result(ErrorCode::TryAgain);
        // CDR encapsulation header (4) + status byte (1) = 5-byte
        // prefix before the result payload.
        constexpr size_t kPrefix = 5;
        if (static_cast<size_t>(rc) <= kPrefix) return Result(ErrorCode::Error);
        size_t payload_len = static_cast<size_t>(rc) - kPrefix;
        if (ResultType::ffi_deserialize(buf + kPrefix, payload_len, &out) != 0)
            return Result(ErrorCode::Error);
        return Result::success();
    }

    /// Issue a cancel request for the given goal.
    Result send_cancel_request(const uint8_t goal_id[16]) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_send_cancel_request_raw(
            storage_, reinterpret_cast<const uint8_t(*)[16]>(goal_id)));
    }

    /// @ref send_cancel_request taking a `GoalUUID` value (phase-417 W4.b).
    Result send_cancel_request(const GoalUUID& goal_id) {
        return send_cancel_request(goal_id.data());
    }

    /// Try to receive the cancel RPC reply (raw CDR bytes).
    Result try_recv_cancel_response(uint8_t* buf, size_t cap, size_t& out_len) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        int32_t rc = nros_cpp_action_client_try_recv_cancel_response_raw(storage_, buf, cap);
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) {
            out_len = 0;
            return Result(ErrorCode::TryAgain);
        }
        out_len = static_cast<size_t>(rc);
        return Result::success();
    }

    /// Try to receive a feedback message. Writes the source `goal_id`
    /// (16 bytes) and deserializes the payload into `out_fb`.
    Result try_recv_feedback(uint8_t goal_id_out[16], FeedbackType& out_fb) {
        return try_recv_feedback_sized<::nros::rx_buffer_capacity<FeedbackType>::value>(goal_id_out,
                                                                                        out_fb);
    }

    /// @ref try_recv_feedback writing the source id into a `GoalUUID` value
    /// (phase-417 W4.b).
    Result try_recv_feedback(GoalUUID& goal_id_out, FeedbackType& out_fb) {
        return try_recv_feedback_sized<::nros::rx_buffer_capacity<FeedbackType>::value>(
            goal_id_out.data(), out_fb);
    }

    /// @ref try_recv_feedback_sized taking a `GoalUUID` value.
    template <size_t Cap>
    Result try_recv_feedback_sized(GoalUUID& goal_id_out, FeedbackType& out_fb) {
        return try_recv_feedback_sized<Cap>(goal_id_out.data(), out_fb);
    }

    /// @ref try_recv_feedback with the receive buffer sized by the CALLER.
    /// See @ref Subscription::try_recv_sized (issue 0964).
    template <size_t Cap>
    Result try_recv_feedback_sized(uint8_t goal_id_out[16], FeedbackType& out_fb) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        uint8_t buf[Cap];
        int32_t rc = nros_cpp_action_client_try_recv_feedback_raw(
            storage_, buf, sizeof(buf), reinterpret_cast<uint8_t(*)[16]>(goal_id_out));
        if (rc < 0) return Result(static_cast<nros_cpp_ret_t>(rc));
        if (rc == 0) return Result(ErrorCode::TryAgain);
        if (FeedbackType::ffi_deserialize(buf, static_cast<size_t>(rc), &out_fb) != 0)
            return Result(ErrorCode::Error);
        return Result::success();
    }

    /// Phase 122.3.c.6.e — caller-owned wake-state slot. One per
    /// (channel) pair; declare next to the client, pass into
    /// `set_*_wake_callback`. Must outlive the client.
    struct WakeState {
        alignas(8) uint64_t _opaque[2] = {0, 0};
    };

    Result set_goal_response_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_set_goal_response_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    Result set_cancel_response_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_set_cancel_response_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    Result set_result_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_set_result_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

    Result set_feedback_wake_callback(WakeState& state, void (*cb)(void*), void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_action_client_set_feedback_wake_callback(
            storage_, reinterpret_cast<nros_cpp_wake_state_t*>(&state),
            reinterpret_cast<void (*)(void*)>(cb), ctx));
    }

  private:
    friend class Node;

    static constexpr size_t kStorageU64s = NROS_CPP_RAW_ACTION_CLIENT_OPAQUE_U64S;
    alignas(8) uint64_t storage_[kStorageU64s];
    char action_name_[ACTION_NAME_MAX];
    bool initialized_;
};

} // namespace nros

#include "nros/node.hpp"

namespace nros {

template <typename A>
Result Node::create_polling_action_client(PollingActionClient<A>& out, const char* action_name) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_ret_t ret =
        nros_cpp_action_client_init_polling(&handle_, action_name, A::TYPE_NAME, A::Goal::TYPE_HASH,
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

#endif // NROS_CPP_POLLING_ACTION_CLIENT_HPP
