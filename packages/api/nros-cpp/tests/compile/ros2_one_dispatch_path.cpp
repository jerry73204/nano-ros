// POSITIVE compile probe for phase-417's structural blocker (RFC-0087
// §"There is also one mismatch the rename makes strictly worse", RFC-0019):
// the shim `rclcpp::Node` has ONE dispatch path, the executor's.
//
// The defect this pins down: `rclcpp::Node` used to run its own `pump()` —
// wall-timer period arithmetic, catch-up snapping, callback ordering, a
// `std::chrono::steady_clock` read — driven ONLY from `rclcpp::spin(node)` /
// `rclcpp::spin_some(node)`. A ported file that instead called
// `nros::spin_once()`, `nros::spin()`, or drove an `nros::Executor` got zero
// callbacks and no diagnostic. Both spin spellings are legitimate API; which
// one is wrong depends on which node object the file holds, so NO diagnostic
// can cover it. Today the two node types have different names, which is the
// only thing making the mismatch visible; after stage 6's rename they would
// not. Hence: structural prerequisite, not a loudness item.
//
// WHAT THIS PROBE PROVES
//   1. The mixed shape TYPE-CHECKS: a shim `rclcpp::Node` subclass that creates
//      a wall timer and a subscription, driven by `nros::spin_once()` and
//      `nros::spin(ms, ms)` — NOT `rclcpp::spin` — with no compat spin verb
//      anywhere in the file.
//   2. `rclcpp::Node` has NO `pump()` member. This is a REACHABILITY assertion,
//      not a style one: `pump()` existing at all means a second dispatch path
//      exists, whether or not this TU calls it. The detector is SFINAE over
//      `declval<Node&>().pump()`, so it fails if the member comes back under
//      any signature.
//   3. A shim wall timer OWNS an `nros::Timer` — i.e. `create_wall_timer`
//      registers on the executor arena rather than parking a deadline in the
//      node. `nros::Timer`'s handle is an arena `HandleId`, so this is the
//      structural evidence that the schedule left the wrapper.
//   4. `create_subscription` hands back the `rclcpp::Subscription<M>::SharedPtr`
//      a ported file declares, and the callable may capture (the shape the
//      native `void(*)(const M&)` overload cannot take).
//
// WHAT IT DOES NOT PROVE
//   That a callback FIRES under `nros::spin_once()`. That needs a running
//   session: linking the library (including these headers emits a
//   config-variant anchor), a backend, and a peer. A runtime cell for this
//   belongs in `nros_tests` beside the other rclcpp-compat fixtures; the
//   structural half is what a compile lane can hold, and it is the half that
//   regresses silently — a reintroduced `pump()` would break nothing that
//   compiles.
//
// Compiled HOSTED (no -ffreestanding), like its `ros2_api_adoption*` siblings:
// `rclcpp_compat.hpp` pulls <memory>/<string>/<functional> unconditionally and
// hands out `std::shared_ptr` in public signatures.

#include <nros/rclcpp_compat.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

namespace nros_cpp_one_dispatch_path_test {

// A generated-message-shaped stub, same shape the sibling probes use.
struct CounterMsg {
    int32_t data = 0;
    static const size_t SERIALIZED_SIZE_MAX = 16;
    static constexpr const char* TYPE_NAME = "std_msgs::msg::dds_::Int32_";
    static constexpr const char* TYPE_HASH = "RIHS01_int32_stub";
    static int ffi_publish(void*, const void*) { return 0; }
    static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
        if (out) *out = 0;
        return 0;
    }
    static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
};

// --- (2) `rclcpp::Node::pump()` must not exist ------------------------------
//
// The detector has to be SFINAE rather than a comment, because "we deleted the
// method" is exactly the kind of claim that quietly comes back. `pump()` was
// public, so a reintroduction under any signature re-arms the two-path defect.

template <typename T, typename = void> struct has_pump : std::false_type {};

template <typename T>
struct has_pump<T, decltype(void(std::declval<T&>().pump()))> : std::true_type {};

// Self-test of the detector: a type that DOES have `pump()` must be caught,
// otherwise the assertion below is vacuous and green forever (the issue-0196
// class — six mechanisms in this campaign were right and never ran).
struct HasPump {
    void pump() {}
};
struct NoPump {};

static_assert(has_pump<HasPump>::value, "the pump() detector does not detect pump()");
static_assert(!has_pump<NoPump>::value, "the pump() detector fires on a type without pump()");

static_assert(!has_pump<rclcpp::Node>::value,
              "rclcpp::Node has a pump() again -- that is a SECOND dispatch path, driven only "
              "by rclcpp::spin/spin_some, and a file spinning nros::spin_once() would get no "
              "callbacks with no diagnostic possible (phase-417 structural blocker)");

// --- (3) a wall timer is an EXECUTOR timer ----------------------------------
//
// `nros::Timer` holds an executor handle + arena `HandleId`, so a shim wall
// timer owning one is the structural evidence that `create_wall_timer`
// registers rather than schedules. The old shape held a
// `std::chrono::steady_clock::time_point next_fire` instead.

static_assert(
    std::is_same<decltype(std::declval<rclcpp::detail::WallTimer&>().timer), ::nros::Timer>::value,
    "rclcpp::detail::WallTimer no longer owns an nros::Timer -- the wall-timer "
    "schedule has moved back into the wrapper");

static_assert(std::is_base_of<rclcpp::TimerBase, rclcpp::detail::WallTimer>::value,
              "create_wall_timer's return type must stay rclcpp::TimerBase::SharedPtr");

// --- (1) + (4) the mixed shape, driven by the NATIVE spin verbs --------------
//
// This class is deliberately written the way a ported rclcpp file is —
// `create_wall_timer` with a capturing lambda, `create_subscription` with a
// capturing lambda, members typed as the nested `SharedPtr` aliases — and then
// driven by `nros::` spin verbs. Before phase-417 this compiled and dispatched
// NOTHING, which is the whole reason a compile probe cannot be the last word
// here (see "WHAT IT DOES NOT PROVE").

class MixedSpinNode : public rclcpp::Node {
  public:
    MixedSpinNode() : rclcpp::Node("mixed_spin_node") {
        publisher_ = this->create_publisher<CounterMsg>("counter", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            CounterMsg msg;
            msg.data = static_cast<int32_t>(++ticks_);
            publisher_->publish(msg);
        });

        // Capturing lambda: the native callback-style `create_subscription`
        // overload takes `void(*)(const M&)` and cannot bind this.
        subscription_ = this->create_subscription<CounterMsg>(
            "counter", 10, [this](const CounterMsg& msg) { last_ = msg.data; });
    }

    size_t ticks() const { return ticks_; }
    int32_t last() const { return last_; }

  private:
    rclcpp::Publisher<CounterMsg>::SharedPtr publisher_;
    rclcpp::Subscription<CounterMsg>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t ticks_ = 0;
    int32_t last_ = 0;
};

/// The acceptance shape: hold a shim node, drive the executor with the NATIVE
/// verbs, never name `rclcpp::spin` / `rclcpp::spin_some`.
inline void drive_with_native_spin_verbs() {
    auto node = std::make_shared<MixedSpinNode>();

    // A single progress sweep.
    (void)::nros::spin_once(0);
    // A budgeted sweep — the same entry point `rclcpp::Rate::sleep()` forwards
    // onto, which therefore also dispatches this node's timer now.
    (void)::nros::spin(10, 5);

    (void)node->ticks();
    (void)node->last();
}

/// The subscription handle a ported file declares still binds, and is still the
/// nested alias rather than some shim-private type.
inline void returned_handles_keep_their_ported_spellings() {
    auto node = std::make_shared<rclcpp::Node>("handles");
    rclcpp::Subscription<CounterMsg>::SharedPtr sub =
        node->create_subscription<CounterMsg>("counter", rclcpp::QoS(10), [](const CounterMsg&) {});
    rclcpp::TimerBase::SharedPtr timer =
        node->create_wall_timer(std::chrono::milliseconds(100), []() {});
    (void)sub;
    (void)timer;
}

inline void probe_entry_points() {
    drive_with_native_spin_verbs();
    returned_handles_keep_their_ported_spellings();
}

} // namespace nros_cpp_one_dispatch_path_test
