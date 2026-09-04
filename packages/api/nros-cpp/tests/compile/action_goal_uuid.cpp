// phase-417 W4.b — the goal-id VALUE type and the three terminal verbs.
//
// Two cross-language disagreements this TU pins down, both catalogued by the
// stage-4 audit ("make our own three languages agree"):
//
//  1. C++ had no goal-id TYPE. C has `nros_goal_uuid_t` and Rust has
//     `nros_core::GoalId`; C++ spelled every goal id `const uint8_t
//     goal_id[16]`, which DECAYS to a pointer — it cannot be compared with
//     `==`, cannot be returned by value, and cannot be stored in a container.
//     `rclcpp_action::GoalUUID` is a `std::array<uint8_t, 16>` and is a map key
//     in most real action servers, so this was a drop-in blocker rather than a
//     style difference. `nros::GoalUUID` is that type, over the SAME 16 bytes.
//
//  2. C++ had no terminal verbs. C has `nros_action_succeed` / `_abort` /
//     `_canceled`, Rust has `ActionServerHandle::succeed` / `abort` /
//     `canceled` (renamed from `cancel` in the same work item), and every
//     rclcpp_action server ends with `goal_handle->succeed(result)`. C++ had
//     only `complete_goal(goal_id, GoalStatus, result)`.
//
// The header `-fsyntax-only` loop in `just check cpp` only PARSES these
// templates, so nothing there instantiates the new overloads. This TU does,
// across all four action tiers, and it also asserts the property that makes
// keeping BOTH spellings safe: a raw-array call site must still select the
// raw-array overload. That is what the `explicit` converting constructor buys,
// and it is the one thing that would break every in-tree action example if it
// regressed — their callbacks receive `const uint8_t[16]` from the FFI.
//
// `just check cpp` compiles this with `-fsyntax-only -std=c++14`.
#include <nros/nros.hpp>

#include <map>
#include <type_traits>

namespace nros_cpp_action_goal_uuid_compile_test {

// Mirror of a codegen'd message (the three action payloads share the shape).
struct Payload {
    int32_t value{0};
    static const size_t SERIALIZED_SIZE_MAX = 32;
    static constexpr const char* TYPE_NAME = "test_msgs::action::dds_::Fib_";
    static constexpr const char* TYPE_HASH = "RIHS01_fib_stub";
    static int ffi_deserialize(const uint8_t*, size_t, Payload*) { return 0; }
    static int ffi_serialize(const Payload*, uint8_t*, size_t, size_t* out) {
        if (out) *out = 0;
        return 0;
    }
};

// Mirror of a codegen'd action type.
struct Fib {
    using Goal = Payload;
    using Result = Payload;
    using Feedback = Payload;
    static constexpr const char* TYPE_NAME = "test_msgs::action::dds_::Fib_";
};

// ── 1. GoalUUID is a VALUE ────────────────────────────────────────────────
//
// Not "a pointer with a nicer name": these are the properties a map key and a
// freestanding target both need. A regression in any of them is a compile
// error here rather than a link-time surprise on an MCU.
static_assert(sizeof(::nros::GoalUUID) == 16, "GoalUUID must be exactly 16 bytes");
static_assert(std::is_trivially_copyable<::nros::GoalUUID>::value,
              "GoalUUID must be trivially copyable — it is memcpy'd across the FFI boundary");
static_assert(std::is_standard_layout<::nros::GoalUUID>::value,
              "GoalUUID must be standard-layout — an array of them IS the uint8_t[N][16] the FFI "
              "reads in send_cancel_reply");

// The converting constructor is `explicit`, which is what keeps the raw-array
// overloads reachable. If it ever became implicit, every raw call site would
// go ambiguous — so assert the property directly, not just the outcome.
static_assert(!std::is_convertible<const uint8_t*, ::nros::GoalUUID>::value,
              "GoalUUID(const uint8_t*) must stay explicit, or the raw-array overloads become "
              "ambiguous at every call site");

// ── 2. It behaves like a value: compare, copy, store ──────────────────────

inline bool comparisons_and_container_use(const uint8_t raw_a[16], const uint8_t raw_b[16]) {
    ::nros::GoalUUID a(raw_a);
    ::nros::GoalUUID b(raw_b);

    // The thing a raw `uint8_t[16]` cannot do.
    if (a == b) return true;
    if (a != b) { /* fallthrough */
    }

    // Copyable and assignable — needed to stash the id from a callback.
    ::nros::GoalUUID copy = a;
    copy = b;

    // The default is the zero ("null") id, mirroring `GoalId::zero()`.
    ::nros::GoalUUID zero;
    if (!zero.is_zero()) return false;

    // The drop-in case: `{goal -> state}`, which is how nearly every real
    // rclcpp_action server tracks work it accepted.
    std::map<::nros::GoalUUID, int> per_goal_state;
    per_goal_state[a] = 1;
    per_goal_state[b] = 2;
    return per_goal_state.find(copy) != per_goal_state.end();
}

// ── 3. The callback tier: verbs + both id spellings ───────────────────────

inline ::nros::Result callback_tier_server(::nros::Node& node) {
    ::nros::ActionServer<Fib> server;
    ::nros::Result r = node.create_action_server(server, "/fib");

    Fib::Result result;

    // The goal callback still receives a raw `const uint8_t[16]` from the FFI
    // trampoline; lifting it to a value is one token.
    (void)server.set_goal_callback([](const uint8_t uuid[16], const Fib::Goal&) {
        ::nros::GoalUUID id(uuid);
        return id.is_zero() ? ::nros::GoalResponse::Reject : ::nros::GoalResponse::AcceptAndExecute;
    });

    ::nros::GoalUUID id;

    // The three verbs, on a value id — `goal_handle->succeed(result)` shaped.
    (void)server.succeed(id, result);
    (void)server.abort(id, result);
    (void)server.canceled(id, result);
    (void)server.publish_feedback(id, result);
    (void)server.complete_goal(id, ::nros::GoalStatus::Canceling, result);

    // …and on a raw array, which must still resolve — every in-tree C++ action
    // example spells it this way.
    uint8_t raw[16] = {0};
    (void)server.succeed(raw, result);
    (void)server.abort(raw, result);
    (void)server.canceled(raw, result);
    (void)server.complete_goal(raw, ::nros::GoalStatus::Aborted, result);
    (void)server.complete_goal(raw, result);
    (void)server.publish_feedback(raw, result);
    return r;
}

inline ::nros::Result callback_tier_client(::nros::Node& node) {
    ::nros::ActionClient<Fib> client;
    ::nros::Result r = node.create_action_client(client, "/fib");

    Fib::Goal goal;
    Fib::Result result;

    // `send_goal` HANDS BACK an id — the direction that most wanted a value
    // type, because the caller then has to keep it.
    ::nros::GoalUUID id;
    (void)client.send_goal(goal, id);
    (void)client.send_goal_async(goal, id);
    (void)client.get_result(id, result);
    (void)client.get_result_async(id);
    (void)client.cancel_goal(id);
    (void)client.get_result_future(id);

    // Raw array still resolves.
    uint8_t raw[16] = {0};
    (void)client.send_goal(goal, raw);
    (void)client.get_result(raw, result);
    (void)client.cancel_goal(raw);

    // `GoalAccept::goal_id` stays a raw array (the FFI decoder writes it);
    // lifting it is the same one token.
    typename ::nros::ActionClient<Fib>::GoalAccept accept;
    ::nros::GoalUUID accepted_id(accept.goal_id);
    (void)client.get_result(accepted_id, result);
    return r;
}

// ── 4. The polling (L1) tier — the same four surfaces ─────────────────────

inline ::nros::Result polling_tier_server(::nros::Node& node) {
    ::nros::PollingActionServer<Fib> server;
    ::nros::Result r = node.create_polling_action_server(server, "/fib");

    ::nros::GoalUUID id;
    Fib::Goal goal;
    Fib::Result result;
    int64_t seq = 0;

    (void)server.try_recv_goal_request(id, goal, seq);
    (void)server.accept_goal(id, seq);
    (void)server.publish_feedback(id, result);
    (void)server.succeed(id, result);
    (void)server.abort(id, result);
    (void)server.canceled(id, result);
    (void)server.complete_goal(id, ::nros::GoalStatus::Succeeded, result);

    ::nros::GoalStatus status = ::nros::GoalStatus::Unknown;
    (void)server.try_recv_cancel_request(id, seq, status);

    // The array-of-ids reply. Building this is what a raw `uint8_t[N][16]`
    // made awkward and a value type makes ordinary.
    ::nros::GoalUUID accepted[2];
    (void)server.send_cancel_reply(seq, ::nros::CancelReturnCode::Ok, accepted, 2);

    // Raw array still resolves.
    uint8_t raw[16] = {0};
    (void)server.try_recv_goal_request(raw, goal, seq);
    (void)server.accept_goal(raw, seq);
    (void)server.complete_goal(raw, ::nros::GoalStatus::Aborted, result);
    return r;
}

inline ::nros::Result polling_tier_client(::nros::Node& node) {
    ::nros::PollingActionClient<Fib> client;
    ::nros::Result r = node.create_polling_action_client(client, "/fib");

    ::nros::GoalUUID id;
    Fib::Goal goal;
    Fib::Feedback feedback;

    (void)client.send_goal(goal, id);
    (void)client.send_get_result_request(id);
    (void)client.send_cancel_request(id);
    (void)client.try_recv_feedback(id, feedback);

    uint8_t raw[16] = {0};
    (void)client.send_goal(goal, raw);
    (void)client.send_cancel_request(raw);
    (void)client.try_recv_feedback(raw, feedback);
    return r;
}

// ── 5. Signature assertions ───────────────────────────────────────────────
//
// A verb that came back as `void`, or that quietly took a `GoalStatus`, would
// pass a name check and re-introduce exactly the "compiles and differs" hazard
// RFC-0089 is written against.
static_assert(
    std::is_same<decltype(std::declval<::nros::ActionServer<Fib>&>().succeed(
                     std::declval<const ::nros::GoalUUID&>(), std::declval<const Fib::Result&>())),
                 ::nros::Result>::value,
    "ActionServer<A>::succeed must return nros::Result");
static_assert(
    std::is_same<decltype(std::declval<::nros::ActionServer<Fib>&>().canceled(
                     std::declval<const ::nros::GoalUUID&>(), std::declval<const Fib::Result&>())),
                 ::nros::Result>::value,
    "ActionServer<A>::canceled must return nros::Result — and be spelled `canceled`, "
    "not `cancel`: C says `nros_action_canceled`, rclcpp_action says `canceled`, and "
    "Rust was renamed to match in phase-417 W4.b");
static_assert(
    std::is_same<decltype(std::declval<::nros::PollingActionServer<Fib>&>().abort(
                     std::declval<const ::nros::GoalUUID&>(), std::declval<const Fib::Result&>())),
                 ::nros::Result>::value,
    "PollingActionServer<A>::abort must return nros::Result");

} // namespace nros_cpp_action_goal_uuid_compile_test
