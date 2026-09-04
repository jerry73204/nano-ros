// FibServer — typed Fibonacci action server bound by identity (no hand-rolled CDR). The C++
// projection of the Rust action_server_pkg / the C c_fib_server_pkg.

#include "action_server_pkg/FibServer.hpp"

#include <cstdio>
#include <cstring>

namespace action_server_pkg {

/// Goal callback — receives the goal UUID + the goal's CDR bytes. Parses the order, ACCEPTS, and
/// stashes the goal for the timer to complete. Returns a `GoalResponse` discriminant.
int32_t FibServer::on_goal(const uint8_t goal_id[16], const uint8_t* data, size_t len) {
    Action::Goal goal;
    if (Action::Goal::ffi_deserialize(data, len, &goal) != 0) {
        /* issue 0737 — say why. A silent reject is indistinguishable from
         * a request that never arrived. */
        std::fprintf(stderr,
                     "[action_server_pkg] goal REJECTED — deserialize failed, %zu byte(s)\n", len);
        return static_cast<int32_t>(::nros::GoalResponse::Reject);
    }
    if (goal.order < 0 || goal.order >= 64) {
        std::printf("[action_server_pkg] goal order=%d REJECTED (out of range)\n", goal.order);
        return static_cast<int32_t>(::nros::GoalResponse::Reject);
    }

    std::memcpy(goal_id_, goal_id, 16);
    order_ = goal.order;
    has_pending_ = true;
    std::printf("[action_server_pkg] goal order=%d\n", goal.order);
    return static_cast<int32_t>(::nros::GoalResponse::AcceptAndExecute);
}

int32_t FibServer::on_cancel(const uint8_t goal_id[16]) {
    (void)goal_id;
    return static_cast<int32_t>(::nros::CancelResponse::Accept);
}

/// Timer tick — the only place the executor is free for action ops. Computes the Fibonacci
/// sequence (order + 1 elements: 0,1,1,2,3,… so order 10 → last element 55) for the accepted
/// goal and completes it with the result CDR.
void FibServer::on_tick() {
    if (!has_pending_) {
        return;
    }

    Action::Result result;
    int32_t a = 0;
    int32_t b = 1;
    for (int32_t i = 0; i <= order_; i++) {
        result.sequence.push_back(a);
        int32_t next = a + b;
        a = b;
        b = next;
    }

    uint8_t buf[Action::Result::SERIALIZED_SIZE_MAX];
    size_t written = 0;
    if (Action::Result::ffi_serialize(&result, buf, sizeof(buf), &written) != 0) {
        return;
    }
    if (nros_cpp_action_server_complete_goal(storage_.bytes, executor_, &goal_id_,
                                             static_cast<int32_t>(nros::GoalStatus::Succeeded), buf,
                                             written) == 0) {
        has_pending_ = false;
        goal_count_++;
        std::printf("[action_server_pkg] completed last=%d\n",
                    result.sequence[result.sequence.length() - 1]);
    }
}

::rclcpp::Result FibServer::configure(::nros::Node& node) {
    // `::setvbuf` (C global), not `std::setvbuf` — Zephyr picolibc lacks the std:: name.
    ::setvbuf(stdout, nullptr, _IONBF, 0);
    executor_ = node.executor_handle();
    has_pending_ = false;

    ::rclcpp::Result r =
        ::nros::bind_action_server_raw<FibServer, &FibServer::on_goal, &FibServer::on_cancel>(
            node, storage_.bytes, "/fibonacci", Action::TYPE_NAME, this);
    if (!r.ok()) {
        return r;
    }
    r = ::nros::bind_timer<FibServer, &FibServer::on_tick>(node, timer_, 500, this);
    if (r.ok()) {
        std::printf("[action_server_pkg] fibonacci action server ready\n");
    }
    return r;
}

} // namespace action_server_pkg
