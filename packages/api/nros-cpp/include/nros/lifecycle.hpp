// nros-cpp: LifecycleNode API (REP-2002 managed nodes)
// Freestanding C++ — no exceptions, no RTTI, no STL required.
//
// Phase 270 (#103) — an rclcpp-shape managed-node wrapper over the executor's
// REP-2002 lifecycle state machine. Inherit `nros::LifecycleNode` and override
// the `on_*` transition hooks (matching
// `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface`); the base binds
// the REP-2002 services and bridges each transition to your override. The C
// state machine (`nros_executor_lifecycle_*`) does all the work — this class is
// a thin, allocation-free wrapper (RFC-0019).

/**
 * @file lifecycle.hpp
 * @ingroup grp_lifecycle
 * @brief Phase 270 — `nros::LifecycleNode` (REP-2002 managed node).
 */

#ifndef NROS_CPP_LIFECYCLE_HPP
#define NROS_CPP_LIFECYCLE_HPP

#include <cstddef>
#include <cstdint>

#include "nros/result.hpp"
// phase-417 stage 2b (RFC-0089) — `nros::TopicEndpointInfo` and the visitor
// typedef used by the graph forwarders below.
#include "nros/graph.hpp"

#include "nros_cpp_ffi.h" // lifecycle FFI: register_lifecycle_services / get_current_state /
                          // change_state / autostart / register_on_* (+ the
                          // nros_cpp_lifecycle_callback_t typedef), all cbindgen-generated
                          // from nros-cpp/src/lifecycle_shim.rs.

namespace nros {

/// REP-2002 primary states. Values match `nros_cpp_lifecycle_get_current_state()` /
/// `nros_core::lifecycle::LifecycleState` (`Unknown` is the `0` sentinel returned for a
/// null executor or before services are registered).
enum class LifecycleState : uint8_t {
    Unknown = 0,
    Unconfigured = 1,
    Inactive = 2,
    Active = 3,
    Finalized = 4,
    ErrorProcessing = 5,
};

/// Transition-callback outcome (rclcpp `CallbackReturn` shape). `Failure` rolls
/// the transition back; `Error` routes to the error-processing transition.
enum class CallbackReturn : uint8_t {
    Success = 0,
    Failure = 1,
    Error = 2,
};

/// rclcpp-shape managed node (REP-2002).
///
/// Inherit and override the `on_*` hooks, then call `register_services()` (or
/// `autostart()`). Transitions are driven either externally (`ros2 lifecycle set`
/// against the registered services) or programmatically (`configure()`,
/// `activate()`, …). The `previous` argument to each hook is the state being left.
///
/// Freestanding-safe: the virtuals are non-pure with defaults (no
/// `__cxa_pure_virtual`), and the class uses no exceptions / RTTI / heap.
class LifecycleNode {
  public:
    /// @param executor_handle Raw executor handle from `Executor::handle()`.
    explicit LifecycleNode(void* executor_handle) : exec_(executor_handle) {}

    /// Two-phase construction for the component model: nano-ros components are
    /// constructed before the executor handle exists, so a component that inherits
    /// `LifecycleNode` default-constructs here and calls `bind()` from its install
    /// hook (`configure(Node&)`, where `node.executor_handle()` is available) before
    /// `register_services()`. Until bound, `get_current_state()` reads `Unconfigured` and the
    /// register/transition calls return `InvalidArgument` rather than trapping.
    LifecycleNode() : exec_(nullptr) {}

    virtual ~LifecycleNode() = default;

    LifecycleNode(const LifecycleNode&) = delete;
    LifecycleNode& operator=(const LifecycleNode&) = delete;

    /// Bind the executor handle for a default-constructed node (two-phase init).
    /// Call once, before `register_services()` / `autostart()`.
    void bind(void* executor_handle) { exec_ = executor_handle; }

    // Transition hooks — override the ones you need. Defaults: Success
    // (on_error: Failure), matching rclcpp.
    virtual CallbackReturn on_configure(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Success;
    }
    virtual CallbackReturn on_activate(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Success;
    }
    virtual CallbackReturn on_deactivate(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Success;
    }
    virtual CallbackReturn on_cleanup(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Success;
    }
    virtual CallbackReturn on_shutdown(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Success;
    }
    virtual CallbackReturn on_error(LifecycleState previous) {
        (void)previous;
        return CallbackReturn::Failure;
    }

    /// Register the five REP-2002 services and bind the `on_*` trampolines. Call
    /// once during setup; afterwards `ros2 lifecycle set|get|list` drives this node.
    Result register_services() {
        Result r = Result(nros_cpp_register_lifecycle_services(exec_));
        if (!r) {
            return r;
        }
        nros_cpp_lifecycle_register_on_configure(exec_, &LifecycleNode::tramp_configure, this);
        nros_cpp_lifecycle_register_on_activate(exec_, &LifecycleNode::tramp_activate, this);
        nros_cpp_lifecycle_register_on_deactivate(exec_, &LifecycleNode::tramp_deactivate, this);
        nros_cpp_lifecycle_register_on_cleanup(exec_, &LifecycleNode::tramp_cleanup, this);
        nros_cpp_lifecycle_register_on_shutdown(exec_, &LifecycleNode::tramp_shutdown, this);
        nros_cpp_lifecycle_register_on_error(exec_, &LifecycleNode::tramp_error, this);
        return Result();
    }

    /// Register services (binding the `on_*` trampolines) then drive the node to
    /// `target` at boot: `Inactive` = configure; `Active` = configure + activate.
    /// Unlike the raw `nros_cpp_lifecycle_autostart` FFI, this binds the callbacks
    /// first, so your overrides fire during the autostart transitions.
    Result autostart(LifecycleState target) {
        Result r = register_services();
        if (!r) {
            return r;
        }
        if (target == LifecycleState::Inactive || target == LifecycleState::Active) {
            r = configure();
            if (!r) {
                return r;
            }
        }
        if (target == LifecycleState::Active) {
            r = activate();
            if (!r) {
                return r;
            }
        }
        return Result();
    }

    /// Current REP-2002 state.
    LifecycleState get_current_state() const {
        return static_cast<LifecycleState>(nros_cpp_lifecycle_get_current_state(exec_));
    }

    // Programmatic transitions (REP-2002 transition ids).
    Result configure() { return trigger_transition(1); }
    Result activate() { return trigger_transition(2); }
    Result deactivate() { return trigger_transition(3); }
    Result cleanup() { return trigger_transition(4); }
    Result shutdown() { return trigger_transition(5); }

    /// Drive an arbitrary REP-2002 transition by id — rclcpp's
    /// `LifecycleNode::trigger_transition(uint8_t)`, and PUBLIC for the same
    /// reason: it is how a ported node reaches a transition that has no named
    /// helper above. The five helpers are this call with the id filled in.
    ///
    /// Phase 379 W5: this was the `protected` `trigger(uint8_t)`, which a
    /// ported rclcpp node could not call at all.
    Result trigger_transition(uint8_t transition_id) {
        return Result(nros_cpp_lifecycle_change_state(exec_, transition_id));
    }

    // ---- Graph queries — phase-417 stage 2b (RFC-0089) --------------------
    //
    // rclcpp_lifecycle's `LifecycleNode` carries the same graph surface as
    // `rclcpp::Node`, so a ported managed node reaches these on `this`. Each
    // one FORWARDS to the executor this node is bound to — the graph's
    // receiver, because there is one session per image (RFC-0002) — and does
    // nothing else: no state, no loop, no caching, no name construction.
    // RFC-0019 keeps the behaviour in Rust. The bodies are the same one-line
    // forwards `nros::Node` carries; see `node.hpp` for the per-call
    // documentation.
    //
    // On an UNBOUND node (default-constructed, `bind()` not yet called) they
    // return `InvalidArgument`, matching the register/transition calls above
    // rather than trapping.
    //
    // The envelope every one of them shares (ADOPT-BOUNDED, RFC-0089): they
    // report what has been DISCOVERED and never block, so an empty result
    // means "nobody seen yet" and never "nobody exists". `ErrorCode::
    // Unsupported`, which is what a backend with no graph at all returns, is a
    // DIFFERENT answer from an empty one and must not be collapsed into zero.

    /// Every node on the graph, with its namespace — rclcpp's
    /// `get_node_names()`. `enclave` is `nullptr` where the backend tracks
    /// none; strings are BORROWED for the call; return `false` to stop.
    Result get_node_names(nros_cpp_node_visit_fn visit, void* ctx) const {
        return Result(nros_cpp_executor_get_node_names(exec_, visit, ctx));
    }

    /// Every topic on the graph, with the types on it — rclcpp's
    /// `get_topic_names_and_types()`. One visit per distinct TOPIC.
    Result get_topic_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) const {
        return Result(nros_cpp_executor_get_topic_names_and_types(exec_, visit, ctx));
    }

    /// Every service on the graph, with its types — rclcpp's
    /// `get_service_names_and_types()`, over servers and clients.
    Result get_service_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) const {
        return Result(nros_cpp_executor_get_service_names_and_types(exec_, visit, ctx));
    }

    /// How many publishers are visible on `topic_name` — rclcpp's
    /// `count_publishers()`. The name is used as given: not remapped, not
    /// expanded. A zero is never a proof of absence.
    Result count_publishers(const char* topic_name, size_t* out_count) const {
        return Result(nros_cpp_executor_count_publishers(exec_, topic_name, out_count));
    }

    /// How many subscribers are visible on `topic_name` — rclcpp's
    /// `count_subscribers()`. See [`count_publishers`].
    Result count_subscribers(const char* topic_name, size_t* out_count) const {
        return Result(nros_cpp_executor_count_subscribers(exec_, topic_name, out_count));
    }

    /// What one named node PUBLISHES, with the types.
    Result get_publisher_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                                 nros_cpp_names_and_types_visit_fn visit,
                                                 void* ctx) const {
        return Result(nros_cpp_executor_get_publisher_names_and_types_by_node(
            exec_, node_name, node_namespace, visit, ctx));
    }

    /// What one named node SUBSCRIBES to, with the types. `subscription`, not
    /// `subscriber` — the C++ surface takes rclcpp's vocabulary and this
    /// matches `nros::Node` / `nros::Executor` rather than adding a third
    /// spelling.
    Result get_subscription_names_and_types_by_node(const char* node_name,
                                                    const char* node_namespace,
                                                    nros_cpp_names_and_types_visit_fn visit,
                                                    void* ctx) const {
        return Result(nros_cpp_executor_get_subscription_names_and_types_by_node(
            exec_, node_name, node_namespace, visit, ctx));
    }

    /// What services one named node SERVES, with the types — servers only,
    /// not clients, as upstream.
    Result get_service_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                               nros_cpp_names_and_types_visit_fn visit,
                                               void* ctx) const {
        return Result(nros_cpp_executor_get_service_names_and_types_by_node(
            exec_, node_name, node_namespace, visit, ctx));
    }

    /// What services one named node CALLS, with the types.
    Result get_client_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                              nros_cpp_names_and_types_visit_fn visit,
                                              void* ctx) const {
        return Result(nros_cpp_executor_get_client_names_and_types_by_node(
            exec_, node_name, node_namespace, visit, ctx));
    }

    /// The publishers discovered on `topic_name`, one visit each — rclcpp's
    /// `get_publishers_info_by_topic()`.
    ///
    /// The endpoint carries NO QoS profile: rclcpp's `qos_profile()` reports
    /// the GRANTED profile, no backend behind this API can read a remote's
    /// granted profile back, and reporting the remote's DECLARED one instead
    /// would be a confident wrong answer to the question ("why is nothing
    /// arriving?") the field exists to answer.
    ///
    /// rclcpp also takes `no_mangle`; there is no such parameter here, because
    /// accepting one and ignoring it would silently drop configuration.
    Result get_publishers_info_by_topic(const char* topic_name,
                                        nros_cpp_endpoint_info_visit_fn visit, void* ctx) const {
        return Result(
            nros_cpp_executor_get_publishers_info_by_topic(exec_, topic_name, visit, ctx));
    }

    /// The publishers on `topic_name`, visited as [`nros::TopicEndpointInfo`]
    /// — the rclcpp-shaped overload of the call above, a pure conversion over
    /// the same query.
    Result get_publishers_info_by_topic(const char* topic_name, TopicEndpointInfoVisitFn visit,
                                        void* ctx) const {
        detail::EndpointInfoTrampoline tramp{visit, ctx};
        return Result(nros_cpp_executor_get_publishers_info_by_topic(
            exec_, topic_name, &detail::EndpointInfoTrampoline::thunk, &tramp));
    }

    /// The subscriptions discovered on `topic_name`, one visit each —
    /// rclcpp's `get_subscriptions_info_by_topic()`. See
    /// [`get_publishers_info_by_topic`] for the QoS and `no_mangle` envelopes.
    Result get_subscriptions_info_by_topic(const char* topic_name,
                                           nros_cpp_endpoint_info_visit_fn visit, void* ctx) const {
        return Result(
            nros_cpp_executor_get_subscriptions_info_by_topic(exec_, topic_name, visit, ctx));
    }

    /// The subscriptions on `topic_name`, visited as
    /// [`nros::TopicEndpointInfo`] — the rclcpp-shaped overload.
    Result get_subscriptions_info_by_topic(const char* topic_name, TopicEndpointInfoVisitFn visit,
                                           void* ctx) const {
        detail::EndpointInfoTrampoline tramp{visit, ctx};
        return Result(nros_cpp_executor_get_subscriptions_info_by_topic(
            exec_, topic_name, &detail::EndpointInfoTrampoline::thunk, &tramp));
    }

  protected:
    /// @deprecated Use `trigger_transition(uint8_t)`, which is public.
    ///
    /// Kept only for a DERIVED class written against the old name: `trigger`
    /// was `protected`, so no caller outside the hierarchy can exist, but
    /// inheriting from `LifecycleNode` is the whole point of the class, so a
    /// user subclass calling `trigger(6)` is a real (if narrow) case. Nothing
    /// in this tree calls it.
    [[deprecated("LifecycleNode::trigger(uint8_t) is deprecated; use "
                 "LifecycleNode::trigger_transition(uint8_t)")]] Result
    trigger(uint8_t transition_id) {
        return trigger_transition(transition_id);
    }

    void* exec_;

  private:
    // Trampolines: `previous` = get_current_state() at callback entry (the SM invokes the
    // callback before committing the new state), then dispatch to the virtual.
    static uint8_t tramp_configure(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_configure(n->get_current_state()));
    }
    static uint8_t tramp_activate(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_activate(n->get_current_state()));
    }
    static uint8_t tramp_deactivate(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_deactivate(n->get_current_state()));
    }
    static uint8_t tramp_cleanup(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_cleanup(n->get_current_state()));
    }
    static uint8_t tramp_shutdown(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_shutdown(n->get_current_state()));
    }
    static uint8_t tramp_error(void* self) {
        auto* n = static_cast<LifecycleNode*>(self);
        return static_cast<uint8_t>(n->on_error(n->get_current_state()));
    }
};

} // namespace nros

#endif // NROS_CPP_LIFECYCLE_HPP
