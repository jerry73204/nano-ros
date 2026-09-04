// SPDX-License-Identifier: Apache-2.0
//
// rclcpp_compat.hpp — source-compat header for ROS 2 `rclcpp` code (Phase 209.A).
//
// Goal: let unmodified C++ source written against `rclcpp` compile against
// nano-ros without #ifdef gymnastics. Pull this header in (transitively, via a
// build-system `target_compile_definitions(... -include nros/rclcpp_compat.hpp)`,
// or directly via `#include "nros/rclcpp_compat.hpp"` ahead of a ported file),
// then keep the original rclcpp call-site syntax.
//
// Scope (what works without source edits):
//   - `rclcpp::init(argc, argv)` / `rclcpp::shutdown()` / `rclcpp::ok()`.
//   - `auto node = std::make_shared<rclcpp::Node>("name");`
//   - `node->create_publisher<M>(topic, qos)` → returns
//     `rclcpp::Publisher<M>::SharedPtr`; `publisher->publish(msg)`.
//   - `node->create_subscription<M>(topic, qos, callback)` → returns
//     `rclcpp::Subscription<M>::SharedPtr` (callback signature
//     `void(const M&)` or `void(std::shared_ptr<M>)`).
//   - `rclcpp::spin(node)` / `rclcpp::spin_some(node)` — forwarders onto
//     `nros::spin()` / `nros::spin_once(0)`. Every entity a shim `Node` creates
//     is registered on the executor, so `nros::spin_once()`, `nros::spin()`, an
//     `nros::Executor` driven directly and `rclcpp::Rate::sleep()` all dispatch
//     the same callbacks (phase-417 — there used to be a second, node-local
//     dispatch path and mixing the two silently did nothing).
//   - `rclcpp::QoS(depth)` and the named profiles `SensorDataQoS` /
//     `ServicesQoS` / `ParametersQoS` / `ParameterEventsQoS` / `RosoutQoS` /
//     `ClockQoS`, each transcribed field-by-field from upstream.
//   - `rclcpp::NodeOptions{}` and `Node(name, options)` constructor shapes
//     used by `rclcpp_components` classes. The option SETTERS are refused —
//     see below.
//   - Parameters on the node (`declare_parameter<T>` / `get_parameter<T>` /
//     `set_parameter<T>` / `has_parameter`), `create_service` /
//     `create_client`, and `rclcpp::Rate` / `WallRate` (phase-417 stage 2).
//   - Log macros `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` and the `_STREAM`
//     family, which now carries its message instead of discarding it.
//   - `rclcpp_action::Server<A>`/`Client<A>` aliases over nros action shapes.
//
// REFUSED LOUDLY (RFC-0087 stage 3 — the name exists, using it is a compile
// error whose message names the constraint and the nano-ros alternative):
//   - `rclcpp::init(argc, argv)` — dropped `--ros-args` silently.
//   - every `rclcpp::NodeOptions` option setter/getter — stored, never read.
//   - `rclcpp::SystemDefaultsQoS` — upstream names no concrete policy.
//   - `RCLCPP_*_THROTTLE` — dropped `clock` and the period unevaluated.
//   - `create_service` / `create_client` with upstream's shared_ptr callback.
//
// Out of scope (will need source adapt or follow-up shims):
//   - `rclcpp_lifecycle::LifecycleNode` — Phase 209.H (deferred).
//   - `rclcpp_components::register_node` macro — see `rclcpp_components_compat.hpp`
//     (Phase 209.C, separate header).
//   - Multi-threaded executors with per-callback group affinity — nano-ros has
//     `nros::MultiExecutor` but the rclcpp callback-group API is not aliased
//     here yet.
//   - tf2 / image_transport / pluginlib — out of nano-ros scope.
//
// Convention: this header DOES NOT replace `<rclcpp/rclcpp.hpp>` for an
// ament/colcon build that genuinely links against upstream ROS 2 — it only
// bridges source code so the same .cpp can be reused under nano-ros. The
// CMake module `cmake/compat/NrosRclcppCompat.cmake` (Phase 209.B) wires
// `find_package(rclcpp)` to this surface.

#ifndef NROS_RCLCPP_COMPAT_HPP
#define NROS_RCLCPP_COMPAT_HPP

#include <cstdlib> // std::abort -- the runtime half of RFC-0087 W3.b
#include <cstring> // std::strcmp -- scanning argv for --ros-args

#include "nros/nros.hpp"
#include "nros/node.hpp"
// phase-417 — the two arena-registration entry points this shim's `Node` uses
// so it has no dispatch of its own: `nros::create_subscription_raw` and
// `nros::Timer` / `nros::Node::create_timer`. Reachable transitively through
// `nros.hpp`, named here because they are load-bearing.
#include "nros/component.hpp"
#include "nros/timer.hpp"
#include "nros/publisher.hpp"
#include "nros/subscription.hpp"
#include "nros/service.hpp"
#include "nros/client.hpp"
#include "nros/executor.hpp"
#include "nros/qos.hpp"
#include "nros/log.hpp"
#include "nros/action_server.hpp"
#include "nros/action_client.hpp"
#include "nros/parameter.hpp"
#include "nros/result.hpp"

#include <memory>
#include <string>
#include <functional>
#include <sstream>
#include <type_traits>
#include <vector>
#include <chrono>

/// Capacity of the shim `rclcpp::Node`'s node-local parameter store. Inline
/// storage, so this is RAM in every image that constructs a shim node —
/// override it per build rather than raising the default.
#ifndef NROS_RCLCPP_MAX_PARAMS
#define NROS_RCLCPP_MAX_PARAMS 16
#endif

namespace rclcpp {

// --- REFUSE-LOUD infrastructure (RFC-0087 stage 3) ---------------------------
//
// RFC-0087's rule:
//
//   An upstream name may be adopted only if its observable contract is the
//   same, or strictly weaker in a documented, non-inverting way. A contract
//   that inverts, or silently drops data or configuration, must fail to
//   COMPILE. Never compile and differ.
//
// A refusal is per-CONCEPT, not per-symbol — the ten inert `NodeOptions`
// accessors below share ONE message. The name EXISTS rather than being absent,
// because `no member named 'use_intra_process_comms'` is honest and teaches
// nothing, while a diagnostic that names the constraint AND the nano-ros
// alternative is the migration, delivered at the point of failure.
//
// Why a dependent `static_assert` and not `= delete`: this header is parsed as
// C++14 (`just check cpp` compiles it and its probes with `-std=c++14`), where
// a deleted function carries NO message — `= delete("reason")` is C++26. A
// `static_assert` inside a template body fires on INSTANTIATION, so the name
// stays declarable and only USING it fails, with the full text attached.
namespace detail {

/// Dependent `false`. `static_assert(refuse<T>::value, …)` in a template body
/// is ill-formed only once that template is instantiated — which is exactly
/// "the name exists, calling it fails".
template <typename T> struct refuse {
    static const bool value = false;
};

/// Is this argument a QoS profile rather than a callback?
///
/// The refusing `create_service(name, F, qos)` overload below must not swallow
/// `create_service<S>("name", rclcpp::ServicesQoS())`. Deducing `F` is an exact
/// match while binding `const nros::QoS&` needs a derived-to-base conversion,
/// so without this guard the callback template WINS and a perfectly good
/// poll-style call fails with the shared_ptr-callback diagnostic — a refusal
/// firing on something it does not describe, which is worse than no refusal.
/// Caught by the positive probe on its first compile.
template <typename F>
struct is_qos_arg : ::std::is_base_of<::nros::QoS, typename ::std::decay<F>::type> {};

#if defined(NROS_SYSTEM_PARAM_SERVICES)
// --- launch-seed adoption (issue 0745) --------------------------------------
//
// The generated entry seeds `nros_cpp_declare_param` into the EXECUTOR's store
// before any user code runs. The shim node's store is a different object, so
// without reading the seed back a `declare_parameter("period", 0.15)` would
// return 0.15 while launch said 0.03 — a silently dropped configuration, which
// is exactly what RFC-0087's rule forbids compiling.
//
// Compiled only when the bringup declares `param_services`, which is also what
// links `nros_cpp_get_param_*`. Where it is absent there is no executor store,
// so there is no seed to drop.
//
// KNOWN DUPLICATION, flagged rather than hidden: this is a C++14 restatement of
// `ComponentNode::adopt_launch_seed_` (`component_node.hpp:536-565`), which
// spells the same dispatch with C++17 `if constexpr`. There should be ONE
// helper, in a header both can reach; hoisting it means editing
// `component_node.hpp`, which this file's author does not own. Tracked with
// phase-417 W2.a (issue 0793, "one parameter store").

inline void adopt_executor_param_seed(void* executor, const char* name, bool& def) {
    if (executor == nullptr) return;
    bool v = false;
    if (nros_cpp_get_param_bool(executor, name, &v) == NROS_CPP_RET_OK) def = v;
}

template <typename T>
inline typename ::std::enable_if<::std::is_floating_point<T>::value>::type
adopt_executor_param_seed(void* executor, const char* name, T& def) {
    if (executor == nullptr) return;
    double v = 0.0;
    if (nros_cpp_get_param_double(executor, name, &v) == NROS_CPP_RET_OK) {
        def = static_cast<T>(v);
    }
}

template <typename T>
inline
    typename ::std::enable_if<::std::is_integral<T>::value && !::std::is_same<T, bool>::value>::type
    adopt_executor_param_seed(void* executor, const char* name, T& def) {
    if (executor == nullptr) return;
    int64_t v = 0;
    if (nros_cpp_get_param_integer(executor, name, &v) == NROS_CPP_RET_OK) {
        def = static_cast<T>(v);
    }
}

inline void adopt_executor_param_seed(void* executor, const char* name, ::std::string& def) {
    if (executor == nullptr) return;
    char buf[256] = {0};
    if (nros_cpp_get_param_string(executor, name, buf, sizeof(buf)) == NROS_CPP_RET_OK) {
        def = buf;
    }
}

/// Everything else — `const char*`, `nros::Seq<…>`, `std::vector<…>`. The
/// executor store's FFI carries four scalar types and no more, so there is no
/// seed channel for these and the code default stands. Silent by necessity
/// rather than by choice: a compile error here would refuse a parameter type
/// launch cannot seed in the first place.
template <typename T>
inline typename ::std::enable_if<!::std::is_arithmetic<T>::value>::type
adopt_executor_param_seed(void*, const char*, T&) {}
#endif // NROS_SYSTEM_PARAM_SERVICES

} // namespace detail

// The diagnostics are macros so one literal backs every site that shares a
// concept (C++14 `static_assert` takes a string LITERAL, not a constexpr
// variable, so a `constexpr const char*` cannot be used here).

#define NROS_RCLCPP_REFUSE_NODE_OPTIONS                                                            \
    "rclcpp::NodeOptions' option setters and getters are REFUSED by nano-ros "                     \
    "(RFC-0087, phase-417 W3.f). Each one used to store its argument in a private field that "     \
    "NOTHING read and return *this, so the idiomatic chained call compiled and configured "        \
    "nothing -- a silent drop of configuration, which the compile-or-conform rule requires to "    \
    "fail to compile instead. nano-ros resolves parameters and remaps in the LAUNCHER "            \
    "(`nros launch` / play_launch, RFC-0060) and projects them into the process environment "      \
    "before exec; it has no runtime ComponentManager, no intra-process transport, no topic "       \
    "statistics collector and no /rosout topic, so there is nothing for these knobs to switch. "   \
    "Use: node->declare_parameter<T>(name, default) for parameter overrides; the launch file "     \
    "for remaps; and drop the option chain. `rclcpp::NodeOptions{}` itself still constructs, so "  \
    "the `Node(name, options)` constructor shape a composable node needs keeps compiling."

#define NROS_RCLCPP_REFUSE_INIT_ARGV                                                               \
    "rclcpp::init(argc, argv) was given --ros-args, which nano-ros cannot honour "                 \
    "(RFC-0087, phase-417 W3.b). Proceeding would DISCARD it, so `-r chatter:=/other` would "      \
    "silently become a wrong-topic bug at runtime -- the 'compiles and differs' the rule "         \
    "forbids. Nothing in this process parses --ros-args yet: nros::init_with_launch_auto(argc, "   \
    "argv) discards them too (node.hpp:1025-1027), and honouring them is remap resolution -- "     \
    "RFC-0020 violation class 4 -- so the parser belongs beside nros::resolve_name, not in this "  \
    "shim. Today remaps and parameter overrides come from the LAUNCHER, which projects them into " \
    "the environment before exec. Call the zero-argument rclcpp::init(), or "                      \
    "nros::init_with_launch_auto(0, nullptr, \"my_session\") for the launch-aware entry point."

#define NROS_RCLCPP_REFUSE_SYSTEM_DEFAULTS_QOS                                                     \
    "rclcpp::SystemDefaultsQoS is REFUSED by nano-ros (RFC-0087 W3.f, issue 0829). Upstream's "    \
    "rmw_qos_profile_system_default names NO concrete policy: every field is a sentinel meaning "  \
    "'let the RMW decide', and the two reference RMWs resolve the depth sentinel differently "     \
    "(rmw_cyclonedds_cpp -> KEEP_LAST 1, rmw_zenoh_cpp -> 42). nros::QoS has no sentinel, "        \
    "deliberately: the backend is linked at build time, so there is no middleware to defer to. "   \
    "Any value this could return would be a concrete profile wearing the name of an absent one, "  \
    "and it used to return QoS(10) -- which is rmw_qos_profile_DEFAULT, a different upstream "     \
    "profile. Name the policy you want: rclcpp::QoS(10) for the ROS default, "                     \
    "rclcpp::SensorDataQoS(), rclcpp::ServicesQoS(), or nros::QoS().best_effort().keep_last(1)."

#define NROS_RCLCPP_REFUSE_THROTTLE                                                                \
    "RCLCPP_*_THROTTLE is REFUSED by nano-ros (RFC-0087 W3.a, issue 1019). It expanded to the "    \
    "plain RCLCPP_* macro with `clock` and the period left UNEVALUATED, so a 1 Hz throttle "       \
    "logged at loop rate and a side-effecting clock expression was dropped entirely. There is no " \
    "throttle on the C or C++ logging path; nros-log has one Rust-side and re-exporting it is "    \
    "phase-417 W4.d, so a throttle written here would be a second implementation of behaviour "    \
    "Rust already owns (RFC-0019). Rate-limit at the call site, or use the un-throttled "          \
    "RCLCPP_INFO / RCLCPP_WARN / RCLCPP_ERROR."

#define NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK                                             \
    "the shared_ptr service-callback shape is REFUSED by nano-ros (RFC-0087, phase-417 W2.c). "    \
    "rclcpp's create_service/create_client callback takes std::shared_ptr<Request> and "           \
    "std::shared_ptr<Response> (plus a request header), which needs a per-request heap "           \
    "allocation on the delivery path. nano-ros has no allocator there (RFC-0022) and hands the "   \
    "request and response BY REFERENCE into caller-owned storage instead, so adopting that "       \
    "signature would mean a second delivery path. Change the handler to "                          \
    "void(const S::Request&, S::Response&) for a service, or void(const S::Response&) for a "      \
    "client -- a plain function pointer or a capture-less lambda -- or take the poll-style "       \
    "overload create_service<S>(name, qos) / create_client<S>(name, qos) and drain it from your "  \
    "spin loop."

// --- Type aliases for shapes that map cleanly ---------------------------------

using ::nros::Result;

// phase-417 W1.d — the clock vocabulary. `rclcpp::Time` / `Duration` / `Clock`
// are the spellings a ported node writes (`rclcpp::Time stamp = node->now();`);
// each is the nano-ros type unchanged, not a wrapper over it.
using Time = ::nros::Time;
using Duration = ::nros::Duration;
using Clock = ::nros::Clock;

// rclcpp::QoS subclasses nros::QoS to add the `QoS(depth)` integer ctor every
// ported source uses; the chainable setters (`reliable()`, `best_effort()`,
// `keep_last(n)`, …) are inherited. Implicit-converts to `nros::QoS` (used in
// the create_publisher/subscription overloads below).
class QoS : public ::nros::QoS {
  public:
    constexpr QoS() = default;
    // NOLINTNEXTLINE(google-explicit-constructor)
    constexpr QoS(::size_t depth) : ::nros::QoS() { keep_last(static_cast<int>(depth)); }
    // NOLINTNEXTLINE(google-explicit-constructor)
    constexpr QoS(const ::nros::QoS& other) : ::nros::QoS(other) {}
};

template <typename M> using Publisher = ::nros::Publisher<M>;

template <typename M> using Subscription = ::nros::Subscription<M>;

template <typename S> using Service = ::nros::Service<S>;

template <typename S> using Client = ::nros::Client<S>;

// `rclcpp::Publisher<M>::SharedPtr` — rclcpp users index types this way, and
// as of phase-417 W1.a they can: the nested `SharedPtr` / `ConstSharedPtr` /
// `UniquePtr` aliases live on `nros::Publisher` / `Subscription` /
// `PollingSubscription` / `Service` / `Client` / `Timer` themselves, so the
// alias templates above carry them through without help from here.
//
// `detail::SharedPtrTrait` was written for this job in phase 209 and never
// wired up — one occurrence in all of `packages/api/`, its own definition. It
// is DELETED rather than adopted: a trait cannot make `T::SharedPtr` resolve
// (that spelling has to be a member of `T`), so it could never have done the
// job it was named for.

// --- Named QoS profiles (RFC-0087 W3.f) --------------------------------------
//
// TRANSCRIBED from upstream field by field, not approximated. Two of these
// shipped WRONG, which is the evidence for why the transcription is pinned by a
// test rather than by review:
//
//   * `ParametersQoS()` returned `QoS(10)` where `rmw_qos_profile_parameters`
//     is KEEP_LAST/**1000** — a hundredfold history difference under a name
//     that claims to be the ROS 2 profile, costing samples under load with
//     nothing to read.
//   * `SystemDefaultsQoS()` returned `QoS(10)`, which is
//     `rmw_qos_profile_default` — a DIFFERENT upstream profile. It is now
//     REFUSE-LOUD; see the class below for why no value can be right.
//
// Sources, read 2026-09-04 against ROS 2 Humble as installed:
//   /opt/ros/humble/include/rmw/rmw/qos_profiles.h    :25,38,51,64,77,90
//   /opt/ros/humble/include/rcl/rcl/logging_rosout.h  :37   (rosout)
//   /opt/ros/humble/include/rclcpp/rclcpp/qos.hpp     :351-489
//
// These are CLASSES, as upstream's are, so `rclcpp::SensorDataQoS{}` — the
// brace form this header's own scope comment used to advertise — finally
// compiles alongside `rclcpp::SensorDataQoS()`. They are `constexpr`, so the
// table-driven check lives in `static_assert`s
// (`tests/compile/ros2_qos_named_profiles.cpp`) rather than in a runtime test
// no embedded lane runs.
//
// Three fields are the same in EVERY upstream profile above and are therefore
// not spelled per profile:
//   * `avoid_ros_namespace_conventions` is `false` upstream, `0` in a default
//     `nros::QoS`.
//   * liveliness is `RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT` upstream;
//     `nros::QoS` defaults to `LivelinessAutomatic`, which is what every
//     reference RMW folds that sentinel to.
//   * deadline and the liveliness lease are `RMW_QOS_*_DEFAULT` (infinite)
//     upstream and `0` here, which `nros::detail::qos_window_ms` documents as
//     infinite.

/// `rmw_qos_profile_sensor_data` — KEEP_LAST(5), BEST_EFFORT, VOLATILE. ADOPT.
class SensorDataQoS : public QoS {
  public:
    constexpr SensorDataQoS() : QoS(::nros::QoS().best_effort().keep_last(5)) {}
};

/// `rmw_qos_profile_services_default` — KEEP_LAST(10), RELIABLE, VOLATILE.
/// ADOPT. (Verified rather than assumed: this one was already right, but it
/// was right by way of `QoS(10)`, which states the depth and leaves the
/// reliability to the default. It now states both.)
class ServicesQoS : public QoS {
  public:
    constexpr ServicesQoS() : QoS(::nros::QoS().reliable().keep_last(10)) {}
};

/// `rmw_qos_profile_parameters` — KEEP_LAST(**1000**), RELIABLE, VOLATILE.
/// ADOPT, at the corrected depth. This is one of RFC-0087's two live
/// inversions: it returned `QoS(10)`.
class ParametersQoS : public QoS {
  public:
    constexpr ParametersQoS() : QoS(::nros::QoS().reliable().keep_last(1000)) {}
};

/// `rmw_qos_profile_parameter_events` — KEEP_LAST(1000), RELIABLE, VOLATILE.
/// ADOPT. New here; a ported node that publishes parameter events names it.
class ParameterEventsQoS : public QoS {
  public:
    constexpr ParameterEventsQoS() : QoS(::nros::QoS().reliable().keep_last(1000)) {}
};

/// `rcl_qos_profile_rosout_default` — KEEP_LAST(1000), RELIABLE,
/// TRANSIENT_LOCAL, lifespan 10 s. ADOPT.
///
/// ADOPT-BOUNDED on one point, and it is about the TOPIC, not the profile: the
/// profile's four policies are transcribed exactly, but nano-ros publishes no
/// `/rosout` topic (logging goes to `nros_log`'s per-platform sink), so this
/// names a profile you can apply to a topic of your own rather than one the
/// runtime is already using.
class RosoutQoS : public QoS {
  public:
    constexpr RosoutQoS()
        : QoS(::nros::QoS().reliable().transient_local().keep_last(1000).lifespan(
              ::nros::Duration::from_nanoseconds(10000000000LL))) {}
};

/// `rclcpp::ClockQoS` — KEEP_LAST(1), BEST_EFFORT, VOLATILE. ADOPT.
///
/// Upstream builds it from `rmw_qos_profile_sensor_data` with `KeepLast(1)`
/// (`rclcpp/qos.hpp:351-357`), which is why the depth differs from
/// `SensorDataQoS`'s 5 and the reliability does not.
class ClockQoS : public QoS {
  public:
    constexpr ClockQoS() : QoS(::nros::QoS().best_effort().keep_last(1)) {}
};

/// `rclcpp::SystemDefaultsQoS` — **REFUSE-LOUD**.
///
/// Every field of `rmw_qos_profile_system_default` is a sentinel meaning "let
/// the RMW decide", and issue 0829 measured the two reference RMWs resolving
/// the depth sentinel to different numbers (Cyclone 1, zenoh 42). `nros::QoS`
/// has no sentinel, deliberately — the backend is linked at build time, so
/// there is nothing to defer to. Any concrete value here would be a different
/// profile wearing this name, which is precisely what the old `QoS(10)` was.
class SystemDefaultsQoS : public QoS {
  public:
    template <typename T = void> SystemDefaultsQoS() {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_SYSTEM_DEFAULTS_QOS);
    }
};

/// `rclcpp::KeepLast(depth)`.
///
/// ADOPT-BOUNDED: upstream returns a `QoSInitialization` — a history/depth pair
/// a `QoS` is then built from — and this returns a whole `QoS` carrying the
/// default reliability and durability. The porting spellings that matter,
/// `rclcpp::QoS(rclcpp::KeepLast(10))` and passing it straight to
/// `create_publisher`, both resolve to the same profile either way. What does
/// NOT carry over is using it as an initialiser for a profile whose other
/// policies you meant to keep — `rclcpp::SensorDataQoS(rclcpp::KeepLast(1))`
/// has no equivalent here; write `nros::QoS().best_effort().keep_last(1)`.
constexpr QoS KeepLast(::size_t depth) {
    return QoS(depth);
}

/// `rclcpp::KeepAll()`. Same ADOPT-BOUNDED note as `KeepLast`.
constexpr QoS KeepAll() {
    return QoS(::nros::QoS().keep_all());
}

// --- NodeOptions -------------------------------------------------------------
//
// The TYPE survives; every OPTION on it is REFUSE-LOUD (RFC-0087 W3.f).
//
// What was here: ten setters that stored their argument in a private field
// nothing read, and returned `*this` so the idiomatic chain compiled and
// configured nothing. The header called them "intentionally inert today". That
// is the second of RFC-0087's two live inversions, and it is the one a rename
// would inherit most quietly — `NodeOptions().use_intra_process_comms(true)`
// correlates `same` against upstream by name AND shape, so the parity
// instrument cannot see the defect it is measuring.
//
// It is not fixable by implementing them, either: nano-ros has no runtime
// ComponentManager, no intra-process transport, no topic-statistics collector
// and no `/rosout` topic, and its parameters and remaps are resolved by the
// LAUNCHER and projected into the environment before exec (RFC-0060). There is
// nothing behind these knobs to switch. So they refuse, with one message.
//
// The DEFAULT CONSTRUCTOR stays, because `rclcpp::NodeOptions{}` and
// `Node(name, options)` are the load-bearing shapes for a composable node and
// neither claims anything. `get_node_options()` on the node stays for the same
// reason: handing back the (empty) options object states nothing false.
//
// The getters refuse alongside the setters. A getter that reports `false` for
// a policy nothing implements reads as "intra-process is off", which is a
// claim about a switch that does not exist — the same silent difference one
// step further from the call site.

class NodeOptions {
  public:
    NodeOptions() = default;

    // --- REFUSED: see NROS_RCLCPP_REFUSE_NODE_OPTIONS -----------------------
    //
    // Each pair is `template <typename T = void>` purely so the
    // `static_assert` is dependent and therefore fires on USE rather than on
    // include. Callers write them exactly as upstream does; deduction picks
    // `T = void` and the assertion reports the migration.

    template <typename T = void> NodeOptions& arguments(const std::vector<std::string>&) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> const std::vector<std::string>& arguments() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return no_arguments_();
    }

    template <typename T = void> NodeOptions& use_global_arguments(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool use_global_arguments() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& enable_rosout(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool enable_rosout() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& start_parameter_services(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool start_parameter_services() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& start_parameter_event_publisher(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool start_parameter_event_publisher() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& allow_undeclared_parameters(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool allow_undeclared_parameters() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void>
    NodeOptions& automatically_declare_parameters_from_overrides(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool automatically_declare_parameters_from_overrides() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& use_intra_process_comms(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool use_intra_process_comms() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& enable_topic_statistics(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool enable_topic_statistics() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

    template <typename T = void> NodeOptions& enable_logger_service(bool) {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return *this;
    }
    template <typename T = void> bool enable_logger_service() const {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_NODE_OPTIONS);
        return false;
    }

  private:
    /// Never reached — the `static_assert` above fires first. It exists only so
    /// the refused `arguments()` has a return expression of the right type,
    /// keeping the diagnostic to ONE error instead of two.
    static const std::vector<std::string>& no_arguments_() {
        static const std::vector<std::string> empty;
        return empty;
    }
};

// --- Logger surface ----------------------------------------------------------
//
// `rclcpp::Logger` in upstream is a pull-through to the rcl logger. Here it is
// a name-only sentinel; the log macros below dispatch through NROS_*, which
// already carry the file/line. The logger NAME is lost (nros has no per-logger
// dispatch yet). Documented; a follow-up can teach nros::log a tag.

class Logger {
  public:
    explicit Logger(const char* name = "") : name_(name) {}
    const char* get_name() const { return name_; }

  private:
    const char* name_;
};

inline Logger get_logger(const char* name) {
    return Logger(name);
}
inline Logger get_logger(const std::string& name) {
    return Logger(name.c_str());
}

// --- Process-level lifecycle -------------------------------------------------
//
// `rclcpp::init()` is a process-level handshake → `nros::init()`.
// `rclcpp::shutdown()` → `nros::shutdown()`. `rclcpp::ok()` → `nros::ok()`
// (nros tracks the shutdown flag).
//
// The TWO-ARGUMENT form is REFUSE-LOUD (RFC-0087 W3.b). It used to forward to
// the same `nros::init()` and discard `argv`, so `--ros-args -r
// chatter:=/other` — the single most common thing a ported `main` passes —
// silently became a wrong-topic bug at runtime. Honouring it is remap
// resolution, RFC-0020 violation class 4, and belongs beside
// `nros::resolve_name` rather than in this shim; until it lands, the honest
// answer is a compile error carrying the migration.

namespace detail {
/// Does this argv carry `--ros-args`? Factored out of `init` so the decision is
/// TESTABLE: an abort inside `init` can only be exercised by a process that then
/// dies, which is the shape of a check nothing ever runs.
/// `constexpr`, and that is the point: it makes the predicate assertable with a
/// `static_assert`, so its cases run in every `check cpp` with no link and no
/// process. `std::strcmp` is not constexpr in C++14, hence the open-coded compare.
constexpr bool is_ros_args_flag(char const* s, int i = 0) {
    return s == nullptr              ? false
           : "--ros-args"[i] == '\0' ? s[i] == '\0'
           : s[i] != "--ros-args"[i] ? false
                                     : is_ros_args_flag(s, i + 1);
}

constexpr bool argv_has_ros_args(int argc, char const* const* argv, int i = 0) {
    return i >= argc                   ? false
           : argv == nullptr           ? false
           : is_ros_args_flag(argv[i]) ? true
                                       : argv_has_ros_args(argc, argv, i + 1);
}
} // namespace detail

/// `rclcpp::init(argc, argv)` — ADOPT-BOUNDED, refused at RUNTIME when it must be.
///
/// **Why this is not a `static_assert`.** Whether the process was given
/// `--ros-args` is a value, not a type: the compiler cannot see it. A compile-time
/// refusal would reject every caller — including the overwhelmingly common
/// embedded one that passes `argc`/`argv` straight through from `main` and has no
/// ROS arguments at all — to catch a case that may never occur, and it would make
/// upstream's own tutorial `main` unportable for a reason unrelated to what the
/// program does.
///
/// So the refusal fires where the information is. RFC-0087's rule is that a
/// contract must never silently drop configuration; it is satisfied by being
/// LOUD, and compile time is simply the earliest point loudness is available. When
/// only the value carries the defect, the earliest point is the call.
///
/// * no `--ros-args` in `argv` → identical to `rclcpp::init()`. Nothing is
///   dropped, because nothing was passed.
/// * `--ros-args` present → the process ABORTS with a diagnostic naming the flag,
///   rather than proceeding with a remap it did not apply. A wrong-topic bug that
///   surfaces three hours into a run is the outcome this exists to prevent.
///
/// Remaps and parameter overrides reach a nano-ros process from the LAUNCHER,
/// which projects them into the environment before exec. Honouring them from
/// `argv` is remap resolution — RFC-0020 violation class 4 — so the parser
/// belongs beside `nros::resolve_name`, and phase-417 W3.b tracks it. Until it
/// lands this call is honest about what it cannot do.
inline void init(int argc, char const* const* argv) {
    if (detail::argv_has_ros_args(argc, argv)) {
        // `std::abort`, not a return code: this call site has nowhere to put a
        // failure -- upstream's `init` returns void, and the ported `main` does
        // not check it. Continuing is the one outcome the rule forbids.
        NROS_ERROR("%s", NROS_RCLCPP_REFUSE_INIT_ARGV);
        ::std::abort();
    }
    (void)::nros::init();
}

/// `rclcpp::init()` — the zero-argument form. ADOPT.
inline void init() {
    (void)::nros::init();
}
inline bool shutdown() {
    ::nros::shutdown();
    return true;
}
inline bool ok() {
    return ::nros::ok();
}

// --- Node shim ---------------------------------------------------------------
//
// rclcpp users write `auto n = std::make_shared<rclcpp::Node>("name");` and
// then `n->create_publisher<M>(topic, qos)` returning a shared_ptr. nros's
// Node is created via an `Executor` and exposes out-ref `create_*` member
// functions. Wrap that to match the rclcpp call shape.
//
// A shim `Node` does NOT own an executor. It opens its `nros::Node` on the
// GLOBAL one `rclcpp::init()` created (issue 0465 — one session per image), and
// since phase-417 every entity it creates — publisher, subscription, wall
// timer, service, client — is registered THERE. So this class holds no dispatch
// state and runs no loop: it is a call-shape adapter over `nros::Node`, which
// is all RFC-0019 permits a wrapper to be, and it is the property that lets the
// stage-6 rename merge the two node types safely.
//
// Threading: callbacks fire on whatever thread services a spin verb, mirroring
// the rclcpp default. An `nros::Node` cannot move between executors (RFC-0002,
// one executor per RTOS task), which is why the executor is decided at
// construction and never afterwards.

// --- Timer surface ----------------------------------------------------------
//
// rclcpp users typically store a `rclcpp::TimerBase::SharedPtr` and only care
// that it stays alive as long as the timer should fire. The dispatch happens
// through `Node::create_wall_timer(period, callback)`.
//
// phase-417 — ONE DISPATCH PATH. `create_wall_timer` registers an EXECUTOR
// timer via `nros::Node::create_timer` (`node.hpp:718`), so the period
// arithmetic, the missed-deadline policy and the clock are the executor's,
// Rust-side. Until this landed, `WallTimer` carried its own
// `std::chrono::steady_clock` deadline and a `Node::pump()` fired it from
// `rclcpp::spin` / `spin_some` ONLY — a ported file driving `nros::spin_once()`
// or an `nros::Executor` got zero callbacks and no diagnostic, and no
// diagnostic could be written for it because both spin spellings are
// legitimate and which one is wrong depends on the node object the file holds.
// Scheduling in the wrapper is RFC-0019/RFC-0020 violation class 2; this is the
// structural fix RFC-0087 makes a prerequisite for the stage-6 rename.
//
// ADOPT-BOUNDED, and both halves of the envelope come with the executor:
//
//   * PERIOD RESOLUTION IS ONE MILLISECOND. `nros_cpp_timer_create` takes a
//     `uint64_t period_ms`, so `create_wall_timer(500us, …)` truncates to 0 and
//     fires every spin. Activations land on spin boundaries either way, so the
//     achievable cadence was already the spin period — the truncation only
//     makes the floor explicit.
//   * MISSED DEADLINES CATCH UP, where rcl's DROP. `TimerState::fire` keeps the
//     overshoot (`elapsed_ms -= period_ms`, `nros-node/src/timer.rs:298`), so
//     after a stall the callback runs once per `spin_once` until the backlog is
//     drained and the mean cadence is preserved. `rcl_timer_call` instead skips
//     whole missed periods and re-phases onto the grid, firing once. This
//     header used to copy rcl. Closing the gap means a missed-deadline POLICY
//     on the executor's timer — Rust-side work, not a loop re-added here.
class TimerBase {
  public:
    /// phase-417 W1.a — `rclcpp::TimerBase::SharedPtr timer_;` is how upstream
    /// source declares a timer member. `<memory>` is unconditional in this
    /// header (the compat surface is hosted-STL by construction — see
    /// `cmake/compat/NrosRclcppCompat.cmake`, which refuses to force-include it
    /// on Zephyr for exactly that reason), so these need no gate.
    using SharedPtr = std::shared_ptr<TimerBase>;
    using ConstSharedPtr = std::shared_ptr<const TimerBase>;
    using UniquePtr = std::unique_ptr<TimerBase>;

    virtual ~TimerBase() = default;
};

namespace detail {

/// An executor-registered timer plus the heap cell holding the user's callable.
///
/// TYPE ERASURE IS THE ONLY THING THIS ADDS. The executor's callback slot is
/// `nros_cpp_timer_callback_t` — `void(*)(void* ctx)` — and a ported rclcpp
/// timer callback is a capturing lambda or a `std::bind` result, which cannot
/// convert to a function pointer. `trampoline` recovers the cell from `ctx` and
/// calls it. That is a spelling, not a second code path (RFC-0087 §"Who
/// implements an adopted name"); no schedule, no clock read, no ordering.
///
/// LIFETIME: the arena stores `this` as the dispatch context and nothing
/// unregisters it, so the cell has to outlive the registration. `Node::timers_`
/// holds a `shared_ptr` for the node's lifetime, and the MEMBER ORDER below is
/// load-bearing — members destruct in reverse declaration order, so `timer`
/// goes first and `~nros::Timer` cancels the arena slot (`timer.hpp:70`) before
/// `callback` is destroyed. Declared the other way round, a tick landing
/// between the two destructions would run a destroyed `std::function`.
class WallTimer : public TimerBase {
  public:
    static void trampoline(void* ctx) {
        auto* self = static_cast<WallTimer*>(ctx);
        if (self != nullptr && self->callback) {
            self->callback();
        }
    }

    std::function<void()> callback; // destroyed LAST
    ::nros::Timer timer;            // destroyed FIRST — cancels the arena slot
};

/// A shim subscription's type-erased callable, and the raw trampoline the
/// executor arena dispatches into.
///
/// Same split as `WallTimer`: the arena's callback slot carries `(bytes, len,
/// ctx)`, the wrapper supplies the `ctx` and the `M::ffi_deserialize` call the
/// generated header already provides, and the executor owns the subscriber and
/// decides when the callback runs. This mirrors `nros::bind_subscription`
/// (`component.hpp:107`) with a heap cell in place of its member-pointer
/// template parameter — a ported rclcpp callback captures, so there is no
/// member pointer to template on.
template <typename M> struct SubscriptionCallback {
    static void trampoline(const uint8_t* data, size_t len, void* ctx) {
        auto* self = static_cast<SubscriptionCallback*>(ctx);
        if (self == nullptr || !self->fn) return;
        M msg;
        if (M::ffi_deserialize(data, len, &msg) != 0) return;
        self->fn(msg);
    }

    std::function<void(const M&)> fn;
    /// The object `create_subscription` hands back. The executor owns the real
    /// subscriber, so this is the ported source's `rclcpp::Subscription<M>::
    /// SharedPtr member_;` keep-alive and nothing more — see the note on
    /// `Node::create_subscription`.
    ::nros::Subscription<M> handle;
};

} // namespace detail

class Node : public std::enable_shared_from_this<Node> {
  public:
    using SharedPtr = std::shared_ptr<Node>;

    explicit Node(const std::string& name) { initialize(name, nullptr); }

    explicit Node(const std::string& name, const NodeOptions& options) : node_options_(options) {
        initialize(name, nullptr);
    }

    Node(const std::string& name, const std::string& namespace_,
         const NodeOptions& options = NodeOptions())
        : node_options_(options) {
        initialize(name, namespace_.c_str());
    }

    ~Node() {
        // Issue 0465 — nothing to tear down per Node any more: the session is
        // the GLOBAL executor's, and `rclcpp::shutdown()` → `nros::shutdown()`
        // owns its lifetime. Shutting down here would have closed the shared
        // session out from under any sibling Node.
    }

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    const NodeOptions& get_node_options() const { return node_options_; }

    const ::nros::Node& nros_node() const { return node_; }
    ::nros::Node& nros_node() { return node_; }

    bool initialized() const { return initialized_; }

    // --- phase-417 W1.d — identity + clock -----------------------------------
    //
    // All four already exist on `nros::Node` (`node.hpp:217,223,249,260`);
    // these are one-line forwarders, which is all RFC-0087 §"Who implements an
    // adopted name" permits the wrapper to be. An uninitialized node answers
    // `""` / the node's own default-constructed clock, matching what
    // `nros::Node` does — this shim adds no behaviour of its own.

    /// `rclcpp::Node::get_name()` — the node's name.
    const char* get_name() const { return node_.get_name(); }

    /// `rclcpp::Node::get_namespace()` — the node's namespace.
    const char* get_namespace() const { return node_.get_namespace(); }

    /// `rclcpp::Node::get_clock()`.
    ///
    /// ADOPT-BOUNDED: upstream hands back a `rclcpp::Clock::SharedPtr`; there
    /// is no allocator here (RFC-0022), so the clock is a member of the
    /// underlying `nros::Node` and this returns a pointer to it. The call
    /// spelling `node->get_clock()->now()` is unchanged; the pointer is valid
    /// for as long as the node is, and must not be freed.
    ::nros::Clock* get_clock() { return node_.get_clock(); }
    /// Const overload of `get_clock()`.
    const ::nros::Clock* get_clock() const { return node_.get_clock(); }

    /// `rclcpp::Node::now()` — the current time on the node's clock.
    ///
    /// Shorthand for `get_clock()->now()`, and the call a ported publisher
    /// makes to stamp a header. See `nros::Clock` for what ROS time does and
    /// does not yet do here (issue 0789).
    ::nros::Time now() const { return node_.now(); }

    Logger get_logger() const { return Logger("nros.compat"); }

  private:
    void initialize(const std::string& name, const char* namespace_) {
        // Bring up the underlying nros::Node on the GLOBAL executor — the one
        // `rclcpp::init()` already created. Initialization failures leave the
        // node marked uninitialized rather than throwing (nros-cpp is
        // freestanding by default — no `<stdexcept>`), so a caller using
        // `std::make_shared<rclcpp::Node>("n")` still mirrors rclcpp's
        // "constructor never returns an error code" contract and subsequent
        // `create_*` fail visibly.
        //
        // Issue 0465 — this used to call `Executor::create(executor_)`, giving
        // every shim Node its OWN executor and therefore its own RMW session.
        // A non-bridge application has exactly one session; two is the bridge
        // shape. With `ZPICO_MAX_SESSIONS` at its default of 1 the second open
        // exhausted the pool and returned -1, surfacing as
        // `Transport(ConnectionFailed)` — the same text a real connection
        // failure gives, which is why it read as one for two months. Raising
        // the pool would have hidden the design error behind memory spent on
        // every embedded target.
        //
        // `::nros::create_node` targets `Node::global_storage()`, so N shim
        // Nodes now share the single session, which is also what rclcpp's own
        // process-level Context model implies.
        ::nros::Result r = ::nros::create_node(node_, name.c_str(), namespace_);
        initialized_ = r.ok();
    }

  public:
    // create_publisher<M>(topic, qos)
    //
    // QoS arg is `const QoS&` OR an integer depth (`10`) — both bind via
    // `nros::QoS(uint32_t)`'s implicit conversion.
    template <typename M>
    std::shared_ptr<::nros::Publisher<M>> create_publisher(const std::string& topic,
                                                           const ::nros::QoS& qos) {
        auto p = std::make_shared<::nros::Publisher<M>>();
        (void)node_.create_publisher(*p, topic.c_str(), qos);
        return p;
    }

    template <typename M>
    std::shared_ptr<::nros::Publisher<M>> create_publisher(const std::string& topic,
                                                           ::size_t depth) {
        return create_publisher<M>(topic, QoS(static_cast<::size_t>(depth)));
    }

    // create_subscription<M>(topic, qos, callback)
    //
    // Accepts ANY callable (capturing lambda, std::function, member-fn bind,
    // plain fn ptr).
    //
    // phase-417 — ONE DISPATCH PATH. This ARENA-REGISTERS the subscription
    // through `nros::create_subscription_raw` (`component.hpp:23`), the same
    // `nros_cpp_subscription_register` call the native callback-style
    // `Node::create_subscription` makes, so the executor owns the subscriber
    // and dispatches the callback during `spin_once` — whichever spin verb the
    // caller drives. It used to create a POLL-mode subscription and drain it
    // from `Node::pump()`, which only `rclcpp::spin` / `spin_some` called: a
    // file that spun `nros::spin_once()` instead got zero callbacks and no
    // diagnostic (RFC-0087 §"There is also one mismatch the rename makes
    // strictly worse").
    //
    // Why not `node_.create_subscription(*s, topic, cb, qos)`: that overload is
    // SFINAE-restricted to `void(*)(const M&)` — a plain function pointer with
    // NO context slot — and every ported rclcpp callback captures. The typed
    // ctx-carrying delivery exists one layer down (`Subscription<M>::
    // user_fn_ctx_` + `user_ctx_`, and the `message_trampoline` branch that
    // reads them, `subscription.hpp:563`) but NO `create_*` sets those fields,
    // so the raw register with a type-erased ctx is the reachable shape. See
    // the phase report: a `create_subscription` overload taking
    // `(void(*)(const M&, void*), void* ctx)` would let this call
    // `nros::Node` instead, and is C++-side work.
    //
    // The receive-buffer hint is `rx_buffer_capacity<M>` — the same number the
    // poll path's `take()` stacks — and NOT the strict `rx_size_bound<M>`,
    // whose unbounded-type arm is a deliberate compile error (issue 0964). A
    // ported file must not stop compiling because its message has an unbounded
    // string.
    //
    // ONE THING THIS LOSES, stated rather than left silent:
    // `create_subscription_raw` hardcodes an EMPTY type hash (it takes no such
    // parameter), which `normalize_type_hash` turns into
    // `"TypeHashNotSupported"`. The poll-mode create this replaced passed
    // `M::TYPE_HASH`. It does not affect DELIVERY — a subscriber's data
    // keyexpr puts `*` in the hash slot (`keyexpr.rs:51`), and on the default
    // Humble edition the publisher's is the literal `TypeHashNotSupported`
    // anyway (`keyexpr.rs:30`) — but under `ros-iron` / `ros-jazzy` the
    // subscription's LIVELINESS token now advertises the placeholder instead of
    // the real hash, so `ros2 topic info --verbose` reads differently. The fix
    // is a `type_hash` parameter on `create_subscription_raw`, defaulted to
    // `""` so no existing caller moves; it is C++-header work in
    // `component.hpp` and is recorded with the phase rather than worked around
    // by calling the FFI from here.
    //
    // OWNERSHIP: the arena stores the cell's address as its dispatch context
    // and there is no unregister, so the cell must outlive the registration.
    // `owned_entities_` holds it for the node's lifetime — the same rule the
    // callback-style `create_service` below states — and the returned
    // `shared_ptr` is a co-owner via the aliasing constructor, so dropping it
    // is safe.
    //
    // WHAT THE RETURNED POINTER IS: a keep-alive, which is all upstream source
    // does with it (`rclcpp::Subscription<M>::SharedPtr sub_;`). The executor
    // owns the real subscriber, so `sub->take(msg)` on it answers
    // `NotInitialized` — the sample went to your callback. That IS a change
    // from the pre-phase-417 shim, where the returned object was additionally a
    // poll-mode subscription; no in-tree caller took from it, and the two
    // cannot both exist over one topic registration.
    template <typename M, typename Cb>
    std::shared_ptr<::nros::Subscription<M>> create_subscription(const std::string& topic,
                                                                 const ::nros::QoS& qos, Cb cb) {
        auto cell = std::make_shared<detail::SubscriptionCallback<M>>();
        cell->fn = std::move(cb);
        (void)::nros::create_subscription_raw(
            node_, topic.c_str(), M::TYPE_NAME, &detail::SubscriptionCallback<M>::trampoline,
            cell.get(), qos, ::nros::rx_buffer_capacity<M>::value);
        owned_entities_.push_back(cell);
        return std::shared_ptr<::nros::Subscription<M>>(cell, &cell->handle);
    }

    template <typename M, typename Cb>
    std::shared_ptr<::nros::Subscription<M>> create_subscription(const std::string& topic,
                                                                 ::size_t depth, Cb cb) {
        return create_subscription<M>(topic, QoS(static_cast<::size_t>(depth)), std::move(cb));
    }

    /// `create_wall_timer(period, callback)` — fires `callback()` every
    /// `period`, dispatched by the EXECUTOR during `spin_once`, i.e. under
    /// whichever spin verb the caller drives: `rclcpp::spin`, `spin_some`,
    /// `nros::spin_once`, `nros::spin`, `rclcpp::Rate::sleep`, or an
    /// `nros::Executor` driven directly.
    ///
    /// The `std::chrono::duration` → milliseconds conversion is the only work
    /// this function does beyond delegating; that is ergonomics and permitted,
    /// the schedule is not. See the envelope on `TimerBase` for the two things
    /// it costs (millisecond resolution, and catch-up rather than rcl's
    /// drop-the-backlog on a missed deadline).
    template <typename Rep, typename Period, typename Cb>
    std::shared_ptr<TimerBase> create_wall_timer(std::chrono::duration<Rep, Period> period, Cb cb) {
        auto t = std::make_shared<detail::WallTimer>();
        t->callback = std::move(cb);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(period).count();
        (void)node_.create_timer(t->timer, ms > 0 ? static_cast<uint64_t>(ms) : uint64_t(0),
                                 &detail::WallTimer::trampoline, t.get());
        // The arena holds `t.get()`; the node keeps the cell alive, and
        // `~nros::Timer` cancels the slot when it finally drops.
        timers_.push_back(t);
        return std::static_pointer_cast<TimerBase>(t);
    }

    // --- phase-417 W2.b — parameters on the node -----------------------------
    //
    // Forwarders onto `nros::ParameterServer` (`parameter.hpp`), which is a
    // thin C++ face over the `nros_parameter_*` C store — the typing, the
    // storage and the string/sequence pools all live below the FFI, so nothing
    // here is a second implementation. The shape is `nros::ComponentNode`'s
    // (`component_node.hpp:568-659`), deliberately: two rclcpp-shaped parameter
    // facades in one package that disagree would be worse than none.
    //
    // ADOPT-BOUNDED, and the envelope is the store's SCOPE. This server is
    // NODE-LOCAL: it is not the executor's store, so `ros2 param list` does not
    // see it and a sibling node does not share it. Converging the three
    // arrangements onto the executor's store is phase-417 W2.a / issue 0793,
    // and it cannot be done from this header — the executor store exposes four
    // typed GETTERS across the FFI (`nros_cpp_get_param_*`) and no setter and
    // no existence check, so `set_parameter` and `has_parameter` have nothing
    // to call. That is Rust-side work, not wrapper work.
    //
    // What the envelope does NOT include is silently ignoring a launch
    // override: where the executor's store exists (`NROS_SYSTEM_PARAM_SERVICES`
    // — the bringup declared `param_services`, which is also what links the
    // FFI), `declare_parameter` adopts the seeded value over the code default,
    // exactly as `ComponentNode` does for issue 0745. Without that a launch
    // parameter would be dead weight, which is the "silently drops
    // configuration" the rule forbids.

    /// `rclcpp::Node::declare_parameter<T>(name, default)` — declare, then read
    /// back, returning the value in effect.
    ///
    /// Re-declaring is not an error: a launch-seeded parameter is DECLARED
    /// before user code runs, and upstream's contract is that `declare` adopts
    /// the override. On any other failure the code default is returned.
    template <typename T> T declare_parameter(const char* name, T default_value = T{}) {
#if defined(NROS_SYSTEM_PARAM_SERVICES)
        detail::adopt_executor_param_seed(node_.executor_handle(), name, default_value);
#endif
        ::nros::Result r = params_.template declare_parameter<T>(name, default_value);
        if (!r.ok() && r.raw() != NROS_RET_ALREADY_EXISTS) {
            return default_value;
        }
        T out{};
        if (!params_.template get_parameter<T>(name, out).ok()) {
            return default_value;
        }
        return out;
    }

    /// `rclcpp::Node::get_parameter<T>(name, out)` — upstream's two-argument
    /// form. Returns whether the parameter was found.
    template <typename T> bool get_parameter(const char* name, T& out) const {
        return params_.template get_parameter<T>(name, out).ok();
    }

    /// Value-returning read, matching `nros::ComponentNode::get_parameter<T>`.
    /// Returns `T{}` when the name is undeclared — declare before you get.
    template <typename T> T get_parameter(const char* name) const {
        T out{};
        (void)params_.template get_parameter<T>(name, out);
        return out;
    }

    /// Set a declared parameter.
    ///
    /// NOT upstream's signature — `rclcpp::Node::set_parameter` takes a single
    /// `rclcpp::Parameter` and returns a `SetParametersResult`, and neither of
    /// those types exists here (both are generated ROS messages). This is the
    /// nano-ros spelling, forwarded to `nros::ParameterServer::set_parameter`;
    /// `rclcpp::Parameter` stays ABSENT rather than being half-modelled.
    template <typename T>::nros::Result set_parameter(const char* name, T value) {
        return params_.template set_parameter<T>(name, value);
    }

    /// `rclcpp::Node::has_parameter(name)`.
    bool has_parameter(const char* name) const { return params_.has_parameter(name); }

    /// `std::string`-keyed overloads. rclcpp keys on `std::string`, which does
    /// not implicitly convert to `const char*`, so a ported call site needs
    /// these to bind at all.
    template <typename T> T declare_parameter(const std::string& name, T default_value = T{}) {
        return this->template declare_parameter<T>(name.c_str(), default_value);
    }
    template <typename T> bool get_parameter(const std::string& name, T& out) const {
        return this->template get_parameter<T>(name.c_str(), out);
    }
    template <typename T> T get_parameter(const std::string& name) const {
        return this->template get_parameter<T>(name.c_str());
    }
    template <typename T>::nros::Result set_parameter(const std::string& name, T value) {
        return this->template set_parameter<T>(name.c_str(), value);
    }
    bool has_parameter(const std::string& name) const { return has_parameter(name.c_str()); }

    /// The node-local parameter store, for the C-API helpers that take one
    /// (`nros_parameter_server_t*`) — e.g. ROS 2 parameter-service
    /// registration. Same escape hatch `nros::ComponentNode` offers.
    ::nros::ParameterServer<NROS_RCLCPP_MAX_PARAMS>& parameters() { return params_; }
    const ::nros::ParameterServer<NROS_RCLCPP_MAX_PARAMS>& parameters() const { return params_; }

    // --- phase-417 W2.c — services and clients -------------------------------
    //
    // Forwarders onto `nros::Node::create_service` / `create_client`, in the
    // rclcpp call shape (name first, `shared_ptr` back). Two overloads each:
    //
    //   * POLL-STYLE `create_service<S>(name, qos)` — no callback, drained with
    //     `service->take_request(...)` from your spin loop. Not an upstream
    //     signature (upstream requires a callback), so it claims nothing.
    //   * CALLBACK-STYLE `create_service<S>(name, callback, qos)` — upstream's
    //     shape, with nano-ros's handler signature. The callback runs during
    //     `spin_once`, on the executor arena — as every other shim-node entity
    //     now does (phase-417). Services were already the correct shape here;
    //     timers and subscriptions were the two that were not.
    //
    // A callback of upstream's `shared_ptr<Request>, shared_ptr<Response>`
    // shape is REFUSE-LOUD rather than "no matching function": that signature
    // needs a per-request heap allocation on the delivery path, which is a
    // second delivery path, not a spelling.
    //
    // OWNERSHIP: the callback-style overloads arena-register the entity, and
    // the arena stores `&entity` as its dispatch context — so the object must
    // outlive the node whatever the caller does with the returned pointer.
    // `owned_entities_` keeps a reference for the node's lifetime; the returned
    // `shared_ptr` is a co-owner, not the only one. Dropping it is safe.

    /// Poll-style service server. Drain with `service->take_request(...)`.
    template <typename S>
    std::shared_ptr<::nros::Service<S>> create_service(const std::string& name,
                                                       const ::nros::QoS& qos = ServicesQoS()) {
        auto s = std::make_shared<::nros::Service<S>>();
        (void)node_.template create_service<S>(*s, name.c_str(), qos);
        return s;
    }

    /// Callback-style service server (`void(const S::Request&, S::Response&)`).
    template <typename S, typename F,
              typename = typename std::enable_if<std::is_convertible<
                  F, void (*)(const typename S::Request&, typename S::Response&)>::value>::type>
    std::shared_ptr<::nros::Service<S>> create_service(const std::string& name, F callback,
                                                       const ::nros::QoS& qos = ServicesQoS()) {
        auto s = std::make_shared<::nros::Service<S>>();
        owned_entities_.push_back(s);
        (void)node_.template create_service<S>(*s, name.c_str(), callback, qos);
        return s;
    }

    /// **REFUSED** — upstream's `shared_ptr` handler shape. See
    /// `NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK`.
    template <typename S, typename F,
              typename = typename std::enable_if<
                  !detail::is_qos_arg<F>::value &&
                  !std::is_convertible<F, void (*)(const typename S::Request&,
                                                   typename S::Response&)>::value>::type,
              typename = void>
    std::shared_ptr<::nros::Service<S>> create_service(const std::string&, F,
                                                       const ::nros::QoS& = ServicesQoS()) {
        static_assert(detail::refuse<F>::value, NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK);
        return std::shared_ptr<::nros::Service<S>>();
    }

    /// Future-style service client — pair with `spin_until_future_complete`.
    template <typename S>
    std::shared_ptr<::nros::Client<S>> create_client(const std::string& name,
                                                     const ::nros::QoS& qos = ServicesQoS()) {
        auto c = std::make_shared<::nros::Client<S>>();
        (void)node_.template create_client<S>(*c, name.c_str(), qos);
        return c;
    }

    /// Callback-style service client (`void(const S::Response&)`).
    template <typename S, typename F,
              typename = typename std::enable_if<
                  std::is_convertible<F, void (*)(const typename S::Response&)>::value>::type>
    std::shared_ptr<::nros::Client<S>> create_client(const std::string& name, F callback,
                                                     const ::nros::QoS& qos = ServicesQoS()) {
        auto c = std::make_shared<::nros::Client<S>>();
        owned_entities_.push_back(c);
        (void)node_.template create_client<S>(*c, name.c_str(), callback, qos);
        return c;
    }

    /// **REFUSED** — upstream's `shared_ptr` handler shape.
    template <typename S, typename F,
              typename = typename std::enable_if<
                  !detail::is_qos_arg<F>::value &&
                  !std::is_convertible<F, void (*)(const typename S::Response&)>::value>::type,
              typename = void>
    std::shared_ptr<::nros::Client<S>> create_client(const std::string&, F,
                                                     const ::nros::QoS& = ServicesQoS()) {
        static_assert(detail::refuse<F>::value, NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK);
        return std::shared_ptr<::nros::Client<S>>();
    }

    // phase-417 — `pump()` IS GONE. It ran this node's wall timers and drained
    // its polling subscriptions, and only `rclcpp::spin` / `spin_some` called
    // it, so a shim node driven by any other spin verb dispatched nothing.
    // Every entity a shim `Node` creates is now registered on the executor
    // arena, so there is nothing left for a node-local sweep to do and mixing
    // spin spellings is harmless. Do not reintroduce one: a second dispatch
    // path here is the RFC-0019 violation the whole item was about, and it
    // cannot be made visible by a diagnostic.

  private:
    ::nros::Node node_;
    NodeOptions node_options_;
    bool initialized_ = false;
    // Wall-timer cells. The executor arena dispatches them and holds each
    // cell's address as its callback context, so the node keeps them alive —
    // see `detail::WallTimer` for the destruction-order rule.
    std::vector<std::shared_ptr<detail::WallTimer>> timers_;
    // Node-local parameter store — see `declare_parameter` above.
    ::nros::ParameterServer<NROS_RCLCPP_MAX_PARAMS> params_;
    // Co-ownership of arena-registered services / clients / subscription
    // callback cells. The executor arena holds a raw pointer as its dispatch
    // context, so the entity must outlive the node even if the caller drops the
    // `shared_ptr` we handed back.
    std::vector<std::shared_ptr<void>> owned_entities_;
};

// --- spin / spin_some --------------------------------------------------------
//
// phase-417 — FORWARDERS, and nothing else. `rclcpp::spin(node)` is
// `nros::spin()` (block until `ok()` goes false, `nros.hpp:164`) and
// `rclcpp::spin_some(node)` is one 0-timeout `nros::spin_once`. Both used to
// call `node->pump()` first, which was the ONLY thing that ran a shim node's
// timers and subscriptions; now those live on the executor, so these two
// dispatch nothing of their own and a file that reaches for `nros::spin_once()`
// or drives an `nros::Executor` gets exactly the same callbacks. That
// equivalence is the structural prerequisite for the stage-6 rename: after it
// both node types share a name, and a mismatch would be invisible.
//
// One behaviour change for an existing caller: `nros::spin()` RETURNS on the
// first failing `spin_once`, where this loop used to discard the result and
// keep going. A dead session now ends the spin instead of looking alive
// forever, which is what `nros::spin()` and `Executor::spin` already promise.
//
// The `node` argument is still checked, and still otherwise unused: there is
// one session per image (issue 0465), so every shim `Node` is on the global
// executor these verbs drive. Upstream takes the node for the same reason and
// spins the executor it belongs to.

inline void spin(const Node::SharedPtr& node) {
    if (!node || !node->initialized()) {
        return;
    }
    (void)::nros::spin();
}

inline void spin_some(const Node::SharedPtr& node) {
    if (!node || !node->initialized()) {
        return;
    }
    (void)::nros::spin_once(0);
}

// --- Rate / WallRate (phase-417 W2.d) ----------------------------------------
//
// A FORWARDER onto `nros::spin(remaining_ms, poll_ms)` (`nros.hpp:183`), which
// budgets by wall clock inside `nros_cpp_spin_for` — Rust-side, once.
//
// The obvious fifteen-line class with its own sleep loop is NOT admissible
// here, and the distinction is the whole point of this item: a loop in the
// wrapper that spins the executor is RFC-0020 violation class 2, and a wrapper
// that BLOCKS without spinning is what RFC-0021 forbids. Same capability, and
// only one of the two shapes is allowed. So `sleep()` computes a deadline and
// makes exactly one call into the entry point that already exists.
//
// ADOPT-BOUNDED, and the envelope is load-bearing enough that a ported loop
// behaves DIFFERENTLY here even though it compiles unchanged:
//
//   * **Your callbacks RUN during the sleep.** `nros::spin` drives the
//     executor, so subscriptions and timers registered on the arena fire while
//     `rate.sleep()` is blocked. Under a single-threaded rclcpp executor they
//     do not — `Rate::sleep()` there is a pure `sleep_for`, and callbacks only
//     run when you call `spin_some`. The nano-ros behaviour is the one a
//     `while (ok()) { work(); rate.sleep(); }` loop usually WANTS (it is why
//     that loop needs a second thread upstream), but it is not the same
//     contract, and a callback that races your loop body is the way it shows.
//     Since phase-417 that includes a shim `Node`'s OWN timers and
//     subscriptions: they are executor entities now, so `rate.sleep()` runs
//     them. The carve-out this bullet used to carry — "a `create_subscription`
//     callback needs `rclcpp::spin_some(node)`, not `rate.sleep()`" — was
//     exactly the two-dispatch-path defect, and it is gone.
//   * `Rate` and `WallRate` are the SAME clock. Upstream's `Rate` measures ROS
//     time and `WallRate` steady time; both here read the monotonic
//     `nros_cpp_time_ns()`, so a `Rate` does not slow down under a sim clock.
//     `WallRate` is faithful; `Rate` is `WallRate` under a second name.
//   * Resolution is ONE MILLISECOND — the FFI budget is `uint32_t` ms — and the
//     remaining time is rounded UP, so a period below 1 ms cannot be met and a
//     rate faster than 1 kHz is not expressible.
//   * Before `nros::init()` there is no executor to spin, so `sleep()` returns
//     immediately rather than blocking. A rate loop that runs before init busy
//     spins; upstream would have slept.

/// `rclcpp::Rate` — a periodic loop rate, driven by the nano-ros executor.
/// See the envelope above; the differences from upstream are real.
class Rate {
  public:
    /// Construct from a frequency in Hz. `0` or negative disables the rate:
    /// `sleep()` becomes a no-op returning `false`.
    explicit Rate(double frequency_hz)
        : period_ns_(frequency_hz > 0.0 ? static_cast<int64_t>(1000000000.0 / frequency_hz) : 0) {
        reset();
    }

    /// Construct from a period — `rclcpp::Rate(std::chrono::milliseconds(100))`.
    template <typename Rep, typename Period>
    explicit Rate(std::chrono::duration<Rep, Period> period)
        : period_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(period).count()) {
        if (period_ns_ < 0) period_ns_ = 0;
        reset();
    }

    /// Spin the executor until the next tick is due.
    ///
    /// @return `true` if there was time left to wait, `false` if the loop had
    ///         already overrun the period (upstream's contract) or the rate is
    ///         disabled.
    bool sleep() {
        if (period_ns_ <= 0) {
            return false;
        }
        const uint64_t now_ns = nros_cpp_time_ns();
        if (now_ns >= next_tick_ns_) {
            // Overran. Re-anchor on now rather than firing a burst to catch up
            // — upstream's `Rate` contract, and the choice `Node::pump()` used
            // to make for wall timers before they became executor timers (which
            // catch up instead; see the envelope on `TimerBase`).
            next_tick_ns_ = now_ns + static_cast<uint64_t>(period_ns_);
            return false;
        }
        const uint64_t remaining_ns = next_tick_ns_ - now_ns;
        // Round UP: truncating a 0.4 ms remainder to 0 would turn `sleep()`
        // into a busy loop.
        uint32_t remaining_ms = static_cast<uint32_t>((remaining_ns + 999999ull) / 1000000ull);
        if (remaining_ms == 0) remaining_ms = 1;
        next_tick_ns_ += static_cast<uint64_t>(period_ns_);
        const int32_t poll_ms = remaining_ms < 10u ? static_cast<int32_t>(remaining_ms) : 10;
        (void)::nros::spin(remaining_ms, poll_ms);
        return true;
    }

    /// `rclcpp::Rate::reset()` — anchor the next tick one period from now.
    void reset() { next_tick_ns_ = nros_cpp_time_ns() + static_cast<uint64_t>(period_ns_); }

    /// The configured period.
    std::chrono::nanoseconds period() const { return std::chrono::nanoseconds(period_ns_); }

  private:
    int64_t period_ns_;
    uint64_t next_tick_ns_;
};

/// `rclcpp::WallRate` — the steady-clock rate. Identical to `Rate` here; see
/// the "same clock" bullet in the envelope above.
using WallRate = Rate;

/// Mirror of `rclcpp::FutureReturnCode` (issue 0339).
///
/// `spin_until_future_complete` used to return `void`, so a caller could not
/// tell success from timeout and the standard idiom
///
/// ```cpp
/// if (rclcpp::spin_until_future_complete(node, fut) == rclcpp::FutureReturnCode::SUCCESS) { … }
/// ```
///
/// could not be written against the shim at all.
enum class FutureReturnCode {
    SUCCESS,
    /// The deadline passed with the future still pending.
    TIMEOUT,
    /// `::nros::ok()` went false (shutdown) before the future was ready.
    INTERRUPTED,
};

// Future type is templated rather than `const auto& future` so the header
// stays parseable under `-std=c++14` (the C++20 abbreviated-function-template
// syntax breaks `just check cpp`'s freestanding probe).
//
// issue 0339 — the bounded branch used to call `Executor::spin(timeout_ms)`
// and never consult the future, so it BURNED THE WHOLE TIMEOUT even when the
// future completed on the first spin: a `wait_for_service` / `send_request`
// sequence ported from rclcpp paid the full timeout on every SUCCESSFUL call.
// The unbounded branch directly below already had the right shape; both now
// share it, differing only in whether a deadline exists.
template <typename Future>
inline FutureReturnCode spin_until_future_complete(const Node::SharedPtr& node,
                                                   const Future& future, int32_t timeout_ms = -1) {
    if (!node || !node->initialized()) {
        return FutureReturnCode::INTERRUPTED;
    }
    // Poll slice: same 10 ms the unbounded loop always used.
    constexpr int32_t kPollMs = 10;
    const bool bounded = timeout_ms >= 0;
    const uint64_t start_ns = nros_cpp_time_ns();
    const uint64_t budget_ns = bounded ? static_cast<uint64_t>(timeout_ms) * 1000000ull : 0ull;

    while (::nros::ok()) {
        if (future.is_ready()) {
            return FutureReturnCode::SUCCESS;
        }
        (void)::nros::spin_once(kPollMs);
        // Re-check before the deadline test: a future that became ready on the
        // spin just above must report SUCCESS even if the budget expired in
        // the same slice.
        if (future.is_ready()) {
            return FutureReturnCode::SUCCESS;
        }
        if (bounded && nros_cpp_time_ns() - start_ns >= budget_ns) {
            return FutureReturnCode::TIMEOUT;
        }
    }
    return FutureReturnCode::INTERRUPTED;
}

namespace detail {

/// **REFUSED** — the target of every `RCLCPP_*_THROTTLE` macro. Variadic so
/// the macro can forward `logger`, `clock`, the period and the whole format
/// pack, which means the arguments are still parsed and type-checked; only the
/// `static_assert` stops the build, with the migration attached.
template <typename Logger, typename Clock, typename Period, typename... Rest>
void throttle_is_refused(Logger&&, Clock&&, Period&&, Rest&&...) {
    static_assert(refuse<Logger>::value, NROS_RCLCPP_REFUSE_THROTTLE);
}

} // namespace detail

} // namespace rclcpp

// --- rclcpp_action ------------------------------------------------------------
//
// Just type aliases — the call shapes (`send_goal_async`, callbacks) need their
// own shim and are gated behind further 209.D-style work.

namespace rclcpp_action {

template <typename A> using Server = ::nros::ActionServer<A>;

template <typename A> using Client = ::nros::ActionClient<A>;

} // namespace rclcpp_action

// --- Log macros --------------------------------------------------------------
//
// Same call shape as rclcpp.
//
// ADOPT-BOUNDED on ONE point, shared by the whole family: the logger argument
// is evaluated and then DISCARDED. `rclcpp::Logger` here is a name-only
// sentinel and the sink is keyed on `__FILE__` / `__LINE__`, so two loggers in
// one file are indistinguishable in the output and a per-logger level cannot be
// set. Named loggers exist Rust-side; re-exporting them is phase-417 W4.d.
// Nothing is lost that the call site did not already have — the message and its
// arguments all reach the sink.
//
// `_STREAM` no longer discards its message (issue 1019). It used to expand to
// `RCLCPP_INFO(logger, "%s", "")` with `args` NEVER REFERENCED, which is the
// worst outcome available: the call compiled, the level was right, the file and
// line were right, and the text was gone. It now formats through
// `std::ostringstream` and hands the result to the same sink — a string
// conversion that copies and calls through, which RFC-0087 §"Who implements an
// adopted name" allows in the wrapper. `<sstream>` is unconditional because
// this header is hosted-STL by construction (`<memory>` / `<string>` already
// are, and `cmake/compat/NrosRclcppCompat.cmake` refuses to force-include it on
// Zephyr for exactly that reason).
//
// `_THROTTLE` is REFUSE-LOUD. See `NROS_RCLCPP_REFUSE_THROTTLE`.

// NROS_INFO is a do-while(0) block; the comma-operator wrapper around it was
// invalid C++. Use a do-while wrapper so RCLCPP_INFO is a single statement.
#ifndef RCLCPP_INFO
#define RCLCPP_INFO(logger, ...)                                                                   \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_INFO(__VA_ARGS__);                                                                    \
    } while (0)
#define RCLCPP_WARN(logger, ...)                                                                   \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_WARN(__VA_ARGS__);                                                                    \
    } while (0)
#define RCLCPP_ERROR(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_ERROR(__VA_ARGS__);                                                                   \
    } while (0)
#define RCLCPP_DEBUG(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_DEBUG(__VA_ARGS__);                                                                   \
    } while (0)
#define RCLCPP_FATAL(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_ERROR(__VA_ARGS__);                                                                   \
    } while (0)

// The stream family, carrying its message. `NROS_RCLCPP_STREAM_` builds the
// text once and forwards it as a single `%s` argument, so a `%` inside the
// user's text can never be read as a conversion.
#define NROS_RCLCPP_STREAM_(macro, logger, ...)                                                    \
    do {                                                                                           \
        ::std::ostringstream nros_rclcpp_stream_;                                                  \
        nros_rclcpp_stream_ << __VA_ARGS__;                                                        \
        macro(logger, "%s", nros_rclcpp_stream_.str().c_str());                                    \
    } while (0)

#define RCLCPP_DEBUG_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_DEBUG, logger, __VA_ARGS__)
#define RCLCPP_INFO_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_INFO, logger, __VA_ARGS__)
#define RCLCPP_WARN_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_WARN, logger, __VA_ARGS__)
#define RCLCPP_ERROR_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_ERROR, logger, __VA_ARGS__)
#define RCLCPP_FATAL_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_FATAL, logger, __VA_ARGS__)

// REFUSED. The arguments are still forwarded so they are parsed and
// type-checked — a refusal should not also hide a typo in the format pack.
#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...)                                        \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, period_ms, ...)                                        \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_FATAL_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#endif // RCLCPP_INFO

#endif // NROS_RCLCPP_COMPAT_HPP
