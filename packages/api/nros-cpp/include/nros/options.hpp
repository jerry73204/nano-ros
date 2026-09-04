// nros-cpp: entity named-options structs (Phase 189.M3)
// Freestanding C++ — no exceptions, no STL required

/**
 * @file options.hpp
 * @ingroup grp_node
 * @brief rclcpp-style named-options structs for entity creation.
 *
 * Mirrors `rclcpp::SubscriptionOptions` / `rclcpp::PublisherOptions`:
 * the options struct sits **alongside** the `QoS` argument (rclcpp
 * convention — QoS is its own positional parameter) and carries the
 * non-QoS creation axes. The `QoS` value class lives in `qos.hpp`.
 *
 * Phase 189.M3.1 introduces a single live field — `sched_context` — on
 * `SubscriptionOptions`, plus a reserved `message_info` flag. The
 * `PublisherOptions` struct is deliberately empty-with-comment: a
 * publisher has no callback, so it has neither a scheduling context nor
 * a message-info axis. It exists for rclcpp symmetry and as the future
 * home for intra-process / loaned-message knobs.
 */

#ifndef NROS_CPP_OPTIONS_HPP
#define NROS_CPP_OPTIONS_HPP

#include <cstdint>
// phase-417 stage 6 step A — `rclcpp::detail::refuse` and
// `NROS_RCLCPP_REFUSE_NODE_OPTIONS`, used by `rclcpp::NodeOptions` below.
#include "nros/log.hpp"

namespace nros {

/// Sentinel meaning "no scheduling context selected" — the entity
/// inherits the executor / Node default `Fifo` context. Matches the
/// `int sched_context = -1` unset convention; valid SchedContext ids are
/// `0..=255` (the FFI `nros_cpp_bind_handle_to_sched_context` takes a
/// `uint8_t`). `0` is the auto-created default `Fifo` SC
/// (`nros_cpp_default_sched_context_id()`), so the sentinel is `-1`, not
/// `0`, to keep "bind to the default explicitly" expressible.
static constexpr int SCHED_CONTEXT_UNSET = -1;

/// rclcpp-style named options for `Node::create_subscription<M>()`.
///
/// Sits alongside the positional `QoS` argument:
/// ```cpp
/// nros::SubscriptionOptions opts;
/// opts.sched_context = my_sc_id;
/// NROS_TRY(node.create_subscription<Msg>(sub, "/topic",
///                                         nros::QoS::default_profile(),
///                                         opts));
/// ```
///
/// Every existing 2-/3-arg `create_subscription` call keeps compiling —
/// the options parameter defaults to `{}` (all fields at their unset /
/// reserved defaults), which is observably identical to the pre-M3
/// behaviour.
struct SubscriptionOptions {
    /// Scheduling-context id to bind this subscription's dispatch onto.
    ///
    /// `SCHED_CONTEXT_UNSET` (the default) leaves the entity on the
    /// executor / Node default `Fifo` context — no bind call is made.
    /// A value in `0..=255` lowers to
    /// `nros_cpp_bind_handle_to_sched_context(executor, handle, sc)`
    /// after the subscription is created (create-then-bind); a failing
    /// bind surfaces as the create call's `Result`.
    ///
    /// NOTE (Phase 189.M3.1): the C++ subscription is a *thin wrapper*
    /// over a bare `RmwSubscriber` polled via `take_serialized` — it does
    /// **not** register a callback entry in the executor, so today's
    /// `nros_cpp_subscription_create` exposes no bindable executor
    /// `HandleId`. The lowering therefore only fires when a handle id is
    /// available (see `Subscription<M>::sched_handle_id_`); for the
    /// poll-style thin wrapper it is a documented no-op until a
    /// handle-returning create FFI lands (tracked alongside M3.4). The
    /// option field + overload are wired now so the rclcpp-shaped call
    /// site is stable and the bind path activates transparently once the
    /// FFI grows a handle id.
    int sched_context = SCHED_CONTEXT_UNSET;

    /// RESERVED — not yet implemented.
    ///
    /// When wired, this selects the with-`MessageInfo` delivery path so
    /// callbacks observe per-sample metadata (source timestamp, GID,
    /// sequence number). That requires a new "with-info" subscription
    /// create FFI + a `SubBufferedRawInfoCEntry`-style arena entry in
    /// `nros-node` — none of which exists yet.
    ///
    /// TODO(M3.4): wire `message_info` to the new arena with-info path.
    /// Setting it today has no effect (it is intentionally ignored).
    bool message_info = false;
};

/// rclcpp-style named options for `Node::create_publisher<M>()`.
///
/// Deliberately empty (reserved): a publisher has no callback, so it
/// carries neither a scheduling context nor a message-info axis. The
/// struct exists for rclcpp symmetry and is the planned home for future
/// intra-process / loaned-message publisher knobs. Passing `{}` is
/// observably identical to the pre-M3 `qos`-only create.
struct PublisherOptions {
    // Reserved for future use (intra-process, loaned-message tuning).
    // No live fields today.
};

/// rclcpp-style named options for `Node::create_action_server<A>()`.
///
/// Phase 189.M3.3.c — `sched_context` is **functional** here (unlike the
/// poll-style `SubscriptionOptions` no-op): the C++ action server is
/// arena-registered (`nros_cpp_action_server_register` →
/// `Executor::register_action_server_raw`), so it owns a real executor
/// `HandleId` and its goal/cancel callbacks are executor-dispatched during
/// `spin_once`. A value in `0..=255` lowers to a bind of that goal-service
/// handle to the scheduling context, governing the priority/policy its
/// callback dispatch runs on. `SCHED_CONTEXT_UNSET` (default) leaves it on
/// the executor / Node default `Fifo` context. A failing bind surfaces as
/// the create call's `Result`.
///
/// NOTE: there is intentionally no `ServiceOptions` / `ClientOptions` with a
/// `sched_context`. C++ services and clients are *poll-style* (a bare
/// `RmwServiceServer` / `RmwServiceClient` the user drives via `take`),
/// so they have no executor-dispatched callback to schedule — a sched
/// context is N/A by design, not merely unwired. Converting them to a
/// callback-style (arena-registered) form is a separate feature.
struct ActionServerOptions {
    /// Scheduling-context id to bind this action server's goal-service
    /// dispatch onto. `SCHED_CONTEXT_UNSET` = executor / Node default.
    int sched_context = SCHED_CONTEXT_UNSET;
};

/// rclcpp-style named options for the **callback-style**
/// `Node::create_service<S>(out, name, callback, qos, options)` overload
/// (Phase 189.M3.3.e).
///
/// `sched_context` is **functional** for callback-style services: that path
/// arena-registers the service (`nros_cpp_service_server_register`), so it owns
/// a real executor `HandleId` whose request dispatch runs during `spin_once` and
/// can be bound to a scheduling context. (It is N/A for the *poll-style*
/// `create_service(out, name, qos)` overload — that has no dispatched callback;
/// see `ActionServerOptions` for the poll-vs-callback rationale.)
struct ServiceOptions {
    /// Scheduling-context id to bind this service's request dispatch onto.
    /// `SCHED_CONTEXT_UNSET` = executor / Node default.
    int sched_context = SCHED_CONTEXT_UNSET;
};

/// rclcpp-style named options for the **callback-style**
/// `Node::create_client<S>(out, name, callback, qos, options)` overload
/// (Phase 189.M3.3.f). `sched_context` is functional for callback-style clients
/// (arena-registered → real executor handle whose response dispatch runs during
/// `spin_once`); N/A for the future-style `create_client(out, name, qos)`.
struct ClientOptions {
    /// Scheduling-context id to bind this client's response dispatch onto.
    /// `SCHED_CONTEXT_UNSET` = executor / Node default.
    int sched_context = SCHED_CONTEXT_UNSET;
};

} // namespace nros

// ============================================================================
// rclcpp::NodeOptions (RFC-0089 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`. This header is where the
// rclcpp-shaped named-options structs live, so it is where the node's own
// options object belongs.
//
// The TYPE survives; every OPTION on it is REFUSE-LOUD (RFC-0089 W3.f).
//
// What was here: ten setters that stored their argument in a private field
// nothing read, and returned `*this` so the idiomatic chain compiled and
// configured nothing. The header called them "intentionally inert today". That
// is the second of RFC-0089's two live inversions, and it is the one a rename
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
//
// `<string>` / `<vector>` are GATED (issue 0112 — `__has_include`, rationale in
// `publisher.hpp`), because this header is on the freestanding path. Upstream's
// `arguments()` signature is spelled in terms of `std::vector<std::string>`, so
// where those types are absent the class is too; a freestanding build has no
// `rclcpp::Node` to hand it to either.
#if defined(NROS_CPP_STD)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#elif defined(__has_include)
#if __has_include(<string>)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#endif
#endif

#if defined(NROS_CPP_STD)
#include <vector>
#define NROS_CPP_HAS_STD_VECTOR 1
#elif defined(__has_include)
#if __has_include(<vector>)
#include <vector>
#define NROS_CPP_HAS_STD_VECTOR 1
#endif
#endif

#if defined(NROS_CPP_HAS_STD_STRING) && defined(NROS_CPP_HAS_STD_VECTOR)

namespace rclcpp {

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

} // namespace rclcpp

#endif // NROS_CPP_HAS_STD_STRING && NROS_CPP_HAS_STD_VECTOR

#endif // NROS_CPP_OPTIONS_HPP
