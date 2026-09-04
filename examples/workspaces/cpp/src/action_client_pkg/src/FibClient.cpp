// FibClient — typed, poll-model Fibonacci action client. The C++ projection of the Rust
// action_client_pkg / the C c_fib_client_pkg.

#include "action_client_pkg/FibClient.hpp"

#include <cstdio>
#include <cstring>

namespace action_client_pkg {

/// Goal-response callback (fired from poll()): on accept, advance to request the result; on
/// reject, fall back to idle to resend.
void FibClient::on_goal_response(bool accepted, const uint8_t goal_id[16]) {
    if (accepted) {
        std::memcpy(goal_id_, goal_id, 16);
        phase_ = NeedResult;
        waits_ = 0;
        std::printf("[action_client_pkg] goal accepted\n");
    } else {
        phase_ = Idle;
    }
}

/// Result callback (fired from poll()): deserialize the Fibonacci sequence and print its last
/// element — the cross-process round-trip proof.
void FibClient::on_result(const uint8_t goal_id[16], int32_t status, const uint8_t* data,
                          size_t len) {
    (void)goal_id;
    (void)status;
    Action::Result result;
    if (data && len > 0 && Action::Result::ffi_deserialize(data, len, &result) == 0 &&
        result.sequence.length() > 0) {
        std::printf("[action_client_pkg] result seq=[");
        for (uint32_t i = 0; i < result.sequence.length(); i++) {
            std::printf(i > 0 ? ", %d" : "%d", result.sequence[i]);
        }
        std::printf("]\n");
        std::printf("[action_client_pkg] result last=%d\n",
                    result.sequence[result.sequence.length() - 1]);
        phase_ = Done;
    }
}

void FibClient::s_goal_response(bool accepted, const uint8_t goal_id[16], void* ctx) {
    static_cast<FibClient*>(ctx)->on_goal_response(accepted, goal_id);
}

void FibClient::s_result(const uint8_t goal_id[16], int32_t status, const uint8_t* data, size_t len,
                         void* ctx) {
    static_cast<FibClient*>(ctx)->on_result(goal_id, status, data, len);
}

void FibClient::send_goal() {
    Action::Goal goal;
    goal.order = 10;
    uint8_t buf[Action::Goal::SERIALIZED_SIZE_MAX];
    size_t written = 0;
    if (Action::Goal::ffi_serialize(&goal, buf, sizeof(buf), &written) == 0 &&
        nros_cpp_action_client_send_goal_async(client_.bytes, buf, written, &goal_id_) == 0) {
        phase_ = GoalSent;
        waits_ = 0;
    }
}

void FibClient::on_tick() {
    // Pump pending replies → fires the goal-response / result callbacks.
    nros_cpp_action_client_poll(client_.bytes);

    switch (phase_) {
    case Idle:
        send_goal();
        break;
    case GoalSent:
        if (++waits_ > 10) {
            phase_ = Idle; // no goal response — resend
        }
        break;
    case NeedResult:
        if (nros_cpp_action_client_get_result_async(client_.bytes, &goal_id_) == 0) {
            phase_ = AwaitResult;
            waits_ = 0;
        }
        break;
    case AwaitResult:
        if (++waits_ > 40) {
            phase_ = NeedResult; // no result — re-request
        }
        break;
    case Done:
    default:
        break;
    }
}

::rclcpp::Result FibClient::configure(::nros::Node& node) {
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    phase_ = Idle;
    waits_ = 0;

    ::rclcpp::Result r =
        ::nros::create_action_client_raw(node, client_.bytes, "/fibonacci", Action::TYPE_NAME);
    if (!r.ok()) {
        return r;
    }
    nros_cpp_ret_t ret =
        nros_cpp_action_client_set_callbacks(client_.bytes, &FibClient::s_goal_response,
                                             /*feedback=*/nullptr, &FibClient::s_result, this);
    if (ret != 0) {
        return ::rclcpp::Result(ret);
    }
    return ::nros::bind_timer<FibClient, &FibClient::on_tick>(node, timer_, 500, this);
}

} // namespace action_client_pkg
