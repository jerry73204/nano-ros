// nros-cpp: Node class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file node.hpp
 * @ingroup grp_node
 * @brief `nros::Node` and global session helpers.
 */

#ifndef NROS_CPP_NODE_HPP
#define NROS_CPP_NODE_HPP

#include <cstdint>
#include <cstddef>
#include <type_traits> // Phase 189.M3.3.e — SFINAE on the callback-style create_service
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
#include <cstdlib> // getenv — Phase 123.B.3 env-aware init
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
#include <cstdio> // fopen — Phase 212.L.5 init_with_launch path-exists check
#endif
#endif

// Phase 118.D: ffi.h MUST come before qos.hpp so qos.hpp's
// `#ifndef NROS_CPP_FFI_H` guard sees the canonical types and skips
// its local redefinitions.
#include "nros_cpp_ffi.h"

#include "nros/result.hpp"
#include "nros/nros_cpp_config_generated.h"
#include "nros/qos.hpp"
// Phase 189.M3.1 — rclcpp-style named-options structs
// (`SubscriptionOptions` / `PublisherOptions`) used by the 4-arg
// `create_subscription` / `create_publisher` overloads below.
#include "nros/options.hpp"
// Phase 84.G8: heavy entity headers (publisher / subscription / service /
// client / action_server / action_client) are no longer pulled in here.
// Each entity header provides the out-of-line definition of its
// corresponding `Node::create_X<T>()` template and includes `node.hpp`
// itself. Consumers that #include `nros/nros.hpp` (the umbrella) still
// get every entity + every create method via that path; consumers that
// only want lightweight Node access can include this header directly
// and pay for only the light entities (timer, guard_condition,
// executor) below.
#include "nros/timer.hpp"
// Issue 0789 — `Node::get_clock()` / `Node::now()`. `clock.hpp` pulls in
// `time.hpp` and `duration.hpp`, so a node that stamps a header needs no
// further include.
#include "nros/clock.hpp"
#include "nros/guard_condition.hpp"
#include "nros/executor.hpp"
// phase-417 stage 2b (RFC-0089) — `nros::TopicEndpointInfo` and the visitor
// typedef used by the graph forwarders below.
#include "nros/graph.hpp"
// Phase 273 (RFC-0047) — callback-group token (value type, no heap).
#include "nros/callback_group.hpp"

#ifdef NROS_RMW_CYCLONEDDS
extern "C" int32_t nros_rmw_cyclonedds_register(void);
#endif
#if defined(NROS_RMW_XRCE) || defined(NROS_RMW_XRCE_CFFI)
extern "C" int32_t nros_rmw_xrce_register(void);
#endif
#ifdef NROS_RMW_ZENOH_CFFI
extern "C" int32_t nros_rmw_zenoh_register(void);
#endif
#ifdef NROS_RMW_UORB
extern "C" int32_t nros_rmw_uorb_register(void);
#endif

// Issue #229 pin (cross-space, C++-FFI half): ErrorCode must stay
// value-identical to the NROS_CPP_RET_* codes (nros_cpp_ffi.h, included
// above) that every shim below feeds into Result().
static_assert(NROS_CPP_RET_NOT_FOUND == -4 && NROS_CPP_RET_ALREADY_EXISTS == -5 &&
                  NROS_CPP_RET_FULL == -6 && NROS_CPP_RET_NOT_INIT == -7 &&
                  NROS_CPP_RET_TRY_AGAIN == -14 && NROS_CPP_RET_REENTRANT == -15 &&
                  NROS_CPP_RET_UNSUPPORTED == -16,
              "nros_cpp_ret_t diverged from the shared numbering (issue #229)");

namespace nros {

// Phase 84.G8: forward declarations of the heavy entity class
// templates. Full definitions live in the corresponding `*.hpp`,
// which also provide the out-of-line `Node::create_X<>` template
// bodies — consumers only pay for the entities they #include.
template <typename M> class Publisher;
template <typename M> class Subscription;
template <typename S> class Service;
template <typename S> class Client;
template <typename A> class ActionServer;
template <typename A> class ActionClient;
// Phase 122.3.d.b — L1 polling-mode action wrappers.
template <typename A> class PollingActionServer;
template <typename A> class PollingActionClient;
template <typename M> class PollingSubscription;
// Phase 242.1 (RFC-0044) — rclcpp-faithful IS-A-node base. It wraps an owned
// `Node` and creates that node against an executor-bound handle in its ctor, so
// it needs friend access to set `executor_handle_` + call `Node::create`
// (the same private-create pattern `Executor` / `NodeBuilder` already use).
class ComponentNode;

/// Issue #227 — pass as `domain_id` to request an EXPLICIT domain 0. Plain
/// `0` is the UNSET sentinel (defers to `ROS_DOMAIN_ID` env on hosted, then
/// the baked `NROS_ENTRY_DOMAIN_ID` macro, then the default — the #206
/// model-A ladder), so a literal domain 0 is otherwise unreachable once the
/// image bakes a nonzero domain. Valid domains cap at 232, so 255 is
/// unambiguous. Mirrors `NROS_DOMAIN_ID_EXPLICIT_ZERO` in the C API
/// (nros_generated.h); hosted env still overrides it under model A.
// `constexpr` at namespace scope is implicitly const, so it already has internal
// linkage per TU — `inline` bought nothing and cost C++14 compatibility, which
// nano-ros otherwise keeps (see `just check cpp`'s freestanding c++14 syntax
// gate). PX4 builds every module with -std=gnu++14 -Werror, so an inline
// variable here made <nros/nros.hpp> uncompilable in a PX4 module (phase-325 W2).
constexpr uint8_t kDomainIdExplicitZero = 255;

/// Initialize an nros session.
///
/// Opens a middleware connection. Must be called before creating nodes.
/// Call `shutdown()` to clean up.
///
/// @param locator  Middleware locator (e.g., "tcp/127.0.0.1:7447"), or nullptr for default.
/// @param domain_id  ROS domain ID (0-232). `0` = unset (env > baked macro >
///                   default decide); `nros::kDomainIdExplicitZero` (255) =
///                   explicitly domain 0 (issue #227).
/// @return Result indicating success or failure.
inline Result init(const char* locator = nullptr, uint8_t domain_id = 0);

/// Initialize the nros session with an explicit session name.
///
/// `session_name` is the *process-wide* identifier used by the
/// XRCE-DDS RMW backend to derive a unique session key. Two
/// processes connecting to the same XRCE Agent MUST use distinct
/// session names — otherwise the agent treats them as the same
/// client and topic publishes don't cross-route. For zenoh / DDS
/// backends the value is informational only.
///
/// Pick a name that's stable for the process and distinct from
/// every other nros process you intend to share an agent with.
/// Typical choice: the process's primary node name (e.g.
/// `"talker"`, `"listener"`).
///
/// @param locator       Middleware locator, or nullptr for default.
/// @param domain_id     ROS domain ID (0-232); `0` = unset,
///                      `nros::kDomainIdExplicitZero` = explicit domain 0.
/// @param session_name  Per-process session identifier. Must not be nullptr.
/// @return Result indicating success or failure.
inline Result init(const char* locator, uint8_t domain_id, const char* session_name);

/// Issue 1050 defect (3) — `init` with an explicit RMW backend name.
///
/// `rmw` is the BAKED rung of RFC-0045's precedence model A: a hosted
/// `$NROS_RMW` still wins over it, and `nullptr` (or `""`) means "this image
/// names no backend", which resolves only when exactly one is registered.
///
/// Use it when the image knows which backend it wants and the registry cannot
/// be trusted to contain only that one. On a hosted target it cannot: a Rust
/// backend compiled into `libnros_cpp.a` registers from its `.init_array` ctor,
/// which runs BEFORE `main` and therefore before the generated
/// `nros_app_register_backends()` — so an archive carrying a backend the image
/// never declared registers first. That is how a PX4 module declaring
/// `BACKENDS uorb` came to open zenoh.
///
/// The plain `init` overloads reach this automatically when the build bakes
/// `NROS_ENTRY_RMW` (`nano_ros_entry(... RMW <name>)`); call it directly only
/// to choose at run time.
inline Result init_with_rmw(const char* rmw, const char* locator = nullptr, uint8_t domain_id = 0,
                            const char* session_name = "node");

/// Phase 212.L.5 Pattern 2 — launch-aware init.
///
/// Resolves runtime knobs (domain id, locator, RMW choice) in this order:
/// 1. `$NROS_RUNTIME_OVERLAY` (JSON sidecar emitted by
///    `nros launch --emit-runtime-overlay`). NOT yet consumed —
///    placeholder for the follow-up wave.
/// 2. Launch XML at `<CARGO_MANIFEST_DIR>/launch/*.xml`. NOT yet parsed —
///    the runtime trusts the launcher to project params/remaps into the
///    child env before exec().
/// 3. Env vars: `ROS_DOMAIN_ID`, `NROS_LOCATOR`, `RMW_IMPLEMENTATION` /
///    `NROS_RMW`. This is the active overlay channel today.
///
/// `argc` / `argv` are reserved for the structured `--ros-args` parse
/// that lands with the runtime-overlay wave. They are accepted and
/// ignored for forward-compat.
///
/// `session_name` falls back to `"nros_cpp"` when null (matches the
/// 2-arg `init` overload).
inline Result init_with_launch_auto(int argc = 0, char** argv = nullptr,
                                    const char* session_name = nullptr);

/// Phase 212.L.5 Pattern 2 — explicit-path variant of
/// [`init_with_launch_auto`].
///
/// Verifies `path` exists (so misspelled paths fail fast) but does NOT
/// yet parse the XML — the env overlay is the active source today. See
/// the auto variant's notes for the follow-up plan.
inline Result init_with_launch(const char* path, int argc = 0, char** argv = nullptr,
                               const char* session_name = nullptr);

/// Shut down the nros session.
///
/// Closes the middleware connection and frees all resources.
inline Result shutdown();

/// Node — the primary interface for creating ROS entities.
///
/// Mirrors `rclcpp::Node`. Entities (publishers, subscriptions, services,
/// etc.) are created through the node. The node holds a reference to the
/// parent executor session.
///
/// Usage:
/// ```cpp
/// nros::Node node;
/// NROS_TRY(nros::Node::create(node, "my_node"));
/// ```
class Node {
  public:
    /// Default constructor — creates an uninitialized node.
    Node()
        : handle_(), initialized_(false), executor_handle_(nullptr), clock_(NROS_CLOCK_ROS_TIME) {}

    /// Create a new node.
    ///
    /// @param out   Receives the initialized node.
    /// @param name  Node name (null-terminated).
    /// @param ns    Node namespace (null-terminated), or nullptr for "/".
    /// @return Result indicating success or failure.
    static Result create(Node& out, const char* name, const char* ns = nullptr) {
        if (!out.executor_handle_) {
            return Result(ErrorCode::NotInitialized);
        }

        nros_cpp_ret_t ret = nros_cpp_node_create(out.executor_handle_, name, ns, &out.handle_);

        if (ret == 0) {
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Get the node name.
    const char* get_name() const {
        if (!initialized_) return "";
        return nros_cpp_node_get_name(&handle_);
    }

    /// Get the node namespace.
    const char* get_namespace() const {
        if (!initialized_) return "";
        return nros_cpp_node_get_namespace(&handle_);
    }

    /// RFC-0088 D4 — the serialization format this node's backend speaks
    /// (`"cdr"`, `"uorb"`), as its cross-image identity string.
    ///
    /// Per SESSION, not per image. A node created with an explicit
    /// `rmw` option sits on its own session, so an image linking two
    /// backends gets two answers here — the case where a compile-time
    /// constant has none.
    ///
    /// Returns `const char*`, deliberately, and never `std::string` or
    /// `std::string_view`: this header must compile against Zephyr's
    /// minimal libcpp, where `<string>` does not exist and `<string_view>`
    /// is not reliably there either (the `NROS_CPP_STD` guard exists for
    /// exactly that). The pointer is static storage owned by the backend;
    /// callers must NOT free it.
    ///
    /// Returns `nullptr` on an uninitialized node, or when the backend
    /// does not declare a format. `nullptr` does not mean `"cdr"`.
    const char* serialization_format() const {
        if (!initialized_) return nullptr;
        return nros_cpp_node_get_serialization_format(&handle_);
    }

    /// Phase 88.12 — return the `nros_log::Logger` keyed on this
    /// node's name. The returned opaque handle is passed to the
    /// `NROS_LOG_*` macros in `<nros/log.hpp>`. Lifetime is
    /// `'static`; callers must NOT free.
    ///
    /// Returns NULL on an uninitialized node.
    const void* get_logger() const {
        if (!initialized_) return nullptr;
        return nros_cpp_node_get_logger(&handle_);
    }

    /// The node's clock — `rclcpp::Node::get_clock()`.
    ///
    /// rclcpp hands back a `rclcpp::Clock::SharedPtr`. There is no allocator
    /// and no `shared_ptr` here (RFC-0022), so the clock is a member of the
    /// node and this returns a pointer to it: `node.get_clock()->now()` and
    /// `node->get_clock()->now()` both keep their rclcpp spelling. The pointer
    /// is valid for as long as the node is.
    ///
    /// The clock is ROS time, as rclcpp's node clock is. See `nros::Clock` for
    /// what ROS time does and does not yet do here (issue 0789).
    Clock* get_clock() { return &clock_; }
    /// Const overload of `get_clock()`.
    const Clock* get_clock() const { return &clock_; }

    /// The current time on the node's clock — `rclcpp::Node::now()`.
    ///
    /// Shorthand for `get_clock()->now()`, and the call a ported publisher
    /// makes to stamp a header:
    /// ```cpp
    /// node.now().to_msg(msg.header.stamp);
    /// ```
    Time now() const { return clock_.now(); }

    /// Check if the node is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Phase 235.A — internal: raw FFI node handle for the Entry-pkg
    /// NodeContext runtime.
    ///
    /// The declarative `NodeContextOps` boundary (`<nros/node_pkg.hpp>`)
    /// is **type-erased** — entities arrive as descriptor *strings*
    /// (`type_name` / `type_hash`), with no message type `M` available
    /// at the op-function-pointer callsite. The native runtime in
    /// `<nros/main.hpp>` therefore constructs publishers / subscriptions
    /// through the raw `nros_cpp_{publisher,subscription}_create` FFI,
    /// which takes a `const nros_cpp_node_t*`. This accessor hands that
    /// pointer to the runtime. Not part of the public rclcpp-style
    /// surface — user code creates entities via the typed
    /// `create_publisher<M>` / `create_subscription<M>` templates.
    ///
    /// Returns `nullptr` on an uninitialized node.
    const nros_cpp_node_t* ffi_handle() const { return initialized_ ? &handle_ : nullptr; }

    /// Phase 211.H (issue #52) — install the per-topic QoS override table the
    /// deploy plan lowered from `qos_overrides.<topic>.<role>.<policy>` launch
    /// params. Every publisher/subscription created on this node afterwards
    /// folds the matching `(topic, role)` entries into its QoS before the
    /// backend-compat check — the C++ mirror of Rust's
    /// `NodeHandle::set_qos_overrides`. Call once, before creating entities (a
    /// generated/hand-written entry does this before `configure(node)`).
    ///
    /// `overrides` must outlive the node (e.g. a `static` array in the entry).
    /// Pass `len == 0` to clear. No-op on an uninitialized node.
    void set_qos_overrides(const nros_cpp_qos_override_t* overrides, size_t len) {
        if (initialized_) {
            ::nros_cpp_node_set_qos_overrides(&handle_, overrides, len);
        }
    }

    /// Phase 240.5 (RFC-0043) — the opaque executor handle this node was opened
    /// against (from `nros_cpp_init`). The component layer needs it for the raw
    /// FFI that is executor- rather than node-scoped (action server register /
    /// complete_goal / publish_feedback) — `Node::create_*` use it internally,
    /// but a stateful component binding those transports raw needs direct access.
    /// `nullptr` on an uninitialized node.
    void* executor_handle() const { return initialized_ ? executor_handle_ : nullptr; }

    // ---- Graph queries — phase-417 stage 2b (RFC-0089) --------------------
    //
    // rclcpp puts these on the node. `nros::Executor` owns them here because
    // one session per image makes the executor the graph's receiver
    // (RFC-0002), so each method below FORWARDS to the executor this node was
    // opened against and nothing else: no state, no loop, no caching, no name
    // construction. RFC-0019 — the behaviour is Rust's and stays there.
    //
    // The envelope every one of them shares (ADOPT-BOUNDED, RFC-0089): they
    // report what has been DISCOVERED and never block, so an empty result
    // means "nobody seen yet" and never "nobody exists" — poll rather than
    // calling once and concluding. `ErrorCode::Unsupported`, which is what a
    // backend with no graph at all returns, is a DIFFERENT answer from an
    // empty one and must not be collapsed into zero.

    /// Every node on the graph, with its namespace — rclcpp's
    /// `Node::get_node_names()`.
    ///
    /// `visit(ctx, name, ns, enclave)` runs once per node; `enclave` is
    /// `nullptr` where the backend tracks none, which is what lets one call
    /// answer both `rmw_get_node_names` forms. Every string is BORROWED for
    /// the duration of the call; return `false` to stop early.
    ///
    /// rclcpp hands back a `std::vector<std::string>`. There is no allocator
    /// here (RFC-0022), so the same enumeration streams through a visitor —
    /// a plain function pointer plus `ctx`, because these headers compile
    /// `-nostdinc++` against Zephyr's minimal libcpp, where `<functional>`
    /// does not exist (issue 0112).
    ///
    /// Reports what has been DISCOVERED and never blocks: an empty
    /// enumeration means "nobody seen yet", not "nobody exists", and
    /// `ErrorCode::Unsupported` from a backend with no graph stays distinct
    /// from it.
    Result get_node_names(nros_cpp_node_visit_fn visit, void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_node_names(executor_handle_, visit, ctx));
    }

    /// Every topic on the graph, with the types on it — rclcpp's
    /// `Node::get_topic_names_and_types()`.
    ///
    /// `visit(ctx, name, types, types_count)` runs once per distinct TOPIC: a
    /// topic carrying two types is one call with two entries, not two calls.
    /// `types_count` may legitimately be 0 on a partially discovered graph.
    /// Same discovery envelope as [`get_node_names`].
    Result get_topic_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_topic_names_and_types(executor_handle_, visit, ctx));
    }

    /// Every service on the graph, with its types — rclcpp's
    /// `Node::get_service_names_and_types()`. As
    /// [`get_topic_names_and_types`], over servers and clients.
    Result get_service_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_service_names_and_types(executor_handle_, visit, ctx));
    }

    /// How many publishers are visible on `topic_name` — rclcpp's
    /// `Node::count_publishers()`.
    ///
    /// `topic_name` is a ROS name (`"/chatter"`), used as given: it is not
    /// remapped and not expanded against this node's namespace, which is what
    /// rclcpp documents for this call too. The count reflects what has been
    /// DISCOVERED, so it can be low right after startup and a zero is never a
    /// proof of absence.
    Result count_publishers(const char* topic_name, size_t* out_count) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_count_publishers(executor_handle_, topic_name, out_count));
    }

    /// How many subscribers are visible on `topic_name` — rclcpp's
    /// `Node::count_subscribers()`. See [`count_publishers`] for the caveats.
    Result count_subscribers(const char* topic_name, size_t* out_count) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_count_subscribers(executor_handle_, topic_name, out_count));
    }

    /// What one named node PUBLISHES, with the types — rclcpp's
    /// `NodeGraph::get_publisher_names_and_types_by_node()`. Same discovery
    /// envelope as [`get_node_names`].
    Result get_publisher_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                                 nros_cpp_names_and_types_visit_fn visit,
                                                 void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_publisher_names_and_types_by_node(
            executor_handle_, node_name, node_namespace, visit, ctx));
    }

    /// What one named node SUBSCRIBES to, with the types.
    ///
    /// `subscription`, not `subscriber`: the C++ surface takes rclcpp's
    /// vocabulary (`create_subscription`, `Subscription<T>`,
    /// `get_subscriptions_info_by_topic`), and `nros::Executor` spells it that
    /// way — this forwarder keeps our two spellings identical rather than
    /// introducing a third. The C surface says `subscriber` because rcl does,
    /// and the vtable slot because upstream rmw does.
    ///
    /// Same discovery envelope as [`get_node_names`].
    Result get_subscription_names_and_types_by_node(const char* node_name,
                                                    const char* node_namespace,
                                                    nros_cpp_names_and_types_visit_fn visit,
                                                    void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_subscription_names_and_types_by_node(
            executor_handle_, node_name, node_namespace, visit, ctx));
    }

    /// What services one named node SERVES, with the types — rclcpp's
    /// `Node::get_service_names_and_types_by_node()`. Servers only, not
    /// clients, as upstream. Same discovery envelope as [`get_node_names`].
    Result get_service_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                               nros_cpp_names_and_types_visit_fn visit,
                                               void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_service_names_and_types_by_node(
            executor_handle_, node_name, node_namespace, visit, ctx));
    }

    /// What services one named node CALLS, with the types — rclcpp's
    /// `NodeGraph::get_client_names_and_types_by_node()`. Same discovery
    /// envelope as [`get_node_names`].
    Result get_client_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                              nros_cpp_names_and_types_visit_fn visit,
                                              void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_client_names_and_types_by_node(
            executor_handle_, node_name, node_namespace, visit, ctx));
    }

    /// The publishers discovered on `topic_name`, one visit each — rclcpp's
    /// `Node::get_publishers_info_by_topic()`.
    ///
    /// The endpoint carries NO QoS profile: rclcpp's `qos_profile()` reports
    /// the GRANTED profile, no backend behind this API can read a remote's
    /// granted profile back, and reporting the remote's DECLARED one instead
    /// would be a confident wrong answer to the question ("why is nothing
    /// arriving?") the field exists to answer. See [`nros::TopicEndpointInfo`].
    ///
    /// rclcpp also takes `no_mangle`; there is no such parameter here, because
    /// accepting one and ignoring it would silently drop configuration —
    /// exactly what RFC-0089's rule forbids.
    ///
    /// Same discovery envelope as [`get_node_names`].
    Result get_publishers_info_by_topic(const char* topic_name,
                                        nros_cpp_endpoint_info_visit_fn visit, void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_publishers_info_by_topic(executor_handle_, topic_name,
                                                                     visit, ctx));
    }

    /// The publishers on `topic_name`, visited as [`nros::TopicEndpointInfo`]
    /// — the rclcpp-shaped overload of the call above.
    ///
    /// A pure conversion over the same query: the visitor sees an endpoint
    /// with rclcpp's accessor names (`node_name()`, `endpoint_type()`,
    /// `endpoint_gid()`) instead of the raw C struct. Every caveat of the
    /// `nros_cpp_endpoint_info_visit_fn` overload applies unchanged, including
    /// the absent QoS profile and the borrowed strings.
    Result get_publishers_info_by_topic(const char* topic_name, TopicEndpointInfoVisitFn visit,
                                        void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        detail::EndpointInfoTrampoline tramp{visit, ctx};
        return Result(nros_cpp_executor_get_publishers_info_by_topic(
            executor_handle_, topic_name, &detail::EndpointInfoTrampoline::thunk, &tramp));
    }

    /// The subscriptions discovered on `topic_name`, one visit each —
    /// rclcpp's `Node::get_subscriptions_info_by_topic()`. See
    /// [`get_publishers_info_by_topic`] for the QoS, `no_mangle` and discovery
    /// envelopes.
    Result get_subscriptions_info_by_topic(const char* topic_name,
                                           nros_cpp_endpoint_info_visit_fn visit, void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_subscriptions_info_by_topic(executor_handle_,
                                                                        topic_name, visit, ctx));
    }

    /// The subscriptions on `topic_name`, visited as
    /// [`nros::TopicEndpointInfo`] — the rclcpp-shaped overload of the call
    /// above.
    Result get_subscriptions_info_by_topic(const char* topic_name, TopicEndpointInfoVisitFn visit,
                                           void* ctx) const {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        detail::EndpointInfoTrampoline tramp{visit, ctx};
        return Result(nros_cpp_executor_get_subscriptions_info_by_topic(
            executor_handle_, topic_name, &detail::EndpointInfoTrampoline::thunk, &tramp));
    }

    /// Create a publisher for a topic.
    ///
    /// @tparam M  Message type (must define TYPE_NAME and TYPE_HASH).
    /// @param out    Receives the initialized publisher.
    /// @param topic  Topic name (null-terminated).
    /// @param qos    QoS profile (default: reliable, keep-last(10)).
    template <typename M>
    Result create_publisher(Publisher<M>& out, const char* topic,
                            const QoS& qos = QoS::default_profile());

    /// Create a publisher with rclcpp-style named options (Phase 189.M3.1).
    ///
    /// `options` sits alongside `qos` (rclcpp convention). `PublisherOptions`
    /// is currently a reserved/empty struct, so this overload is observably
    /// identical to the `qos`-only form — it exists for API symmetry and as
    /// the seam for future intra-process / loaned-message knobs.
    ///
    /// @tparam M  Message type (must define TYPE_NAME and TYPE_HASH).
    /// @param out      Receives the initialized publisher.
    /// @param topic    Topic name (null-terminated).
    /// @param qos      QoS profile.
    /// @param options  Named publisher options.
    template <typename M>
    Result create_publisher(Publisher<M>& out, const char* topic, const QoS& qos,
                            const PublisherOptions& options);

    /// Create a subscription for a topic.
    ///
    /// @tparam M  Message type (must define TYPE_NAME and TYPE_HASH).
    /// @param out    Receives the initialized subscription.
    /// @param topic  Topic name (null-terminated).
    /// @param qos    QoS profile (default: reliable, keep-last(10)).
    template <typename M>
    Result create_subscription(Subscription<M>& out, const char* topic,
                               const QoS& qos = QoS::default_profile());

    /// Create a subscription with rclcpp-style named options (Phase 189.M3.1).
    ///
    /// `options` sits alongside `qos` (rclcpp convention) and carries the
    /// non-QoS creation axes. `options.sched_context` (when set) lowers to a
    /// create-then-bind via `nros_cpp_bind_handle_to_sched_context`;
    /// `options.message_info` is reserved (M3.4). See `SubscriptionOptions`.
    ///
    /// @tparam M  Message type (must define TYPE_NAME and TYPE_HASH).
    /// @param out      Receives the initialized subscription.
    /// @param topic    Topic name (null-terminated).
    /// @param qos      QoS profile.
    /// @param options  Named subscription options.
    template <typename M>
    Result create_subscription(Subscription<M>& out, const char* topic, const QoS& qos,
                               const SubscriptionOptions& options);

    /// Create a **callback-style** subscription (rclcpp dispatch model; Phase
    /// 189.M3.x). The executor arena owns the subscriber and invokes `callback`
    /// during `spin_once()` on each new sample, so `options.sched_context` is
    /// functional (poll-style subscriptions have no dispatched callback to
    /// schedule). `callback` must be convertible to `void(const M&)` (a plain
    /// function pointer or empty-capture lambda); the SFINAE guard keeps the
    /// poll-style overloads unambiguous (a `QoS` is not convertible to the
    /// handler type).
    ///
    /// CONSTRAINT: do not move `out` after this returns — the executor arena
    /// holds `&out` as the dispatch context.
    ///
    /// @tparam M  Message type (must define TYPE_NAME, TYPE_HASH, ffi_deserialize).
    /// @param out       Receives the initialized subscription (callback mode).
    /// @param topic     Topic name (null-terminated).
    /// @param callback  Handler invoked as `callback(const M&)` per sample.
    /// @param qos       QoS profile.
    /// @param options   Named subscription options (e.g. sched_context).
    template <
        typename M, typename F,
        typename = typename std::enable_if<std::is_convertible<F, void (*)(const M&)>::value>::type>
    Result create_subscription(Subscription<M>& out, const char* topic, F callback,
                               const QoS& qos = QoS::default_profile(),
                               const SubscriptionOptions& options = {});

    /// Create a **callback-style** subscription that also delivers each sample's
    /// wire **attachment** (Phase 189.M3.4 — the callback analogue of
    /// `Subscription::take_serialized_with_attachment`). Arena-registered like the
    /// callback overload above (so `options.sched_context` is functional), but the
    /// handler is invoked as `callback(const M&, const uint8_t* attachment, size_t
    /// attachment_len)`; `attachment_len == 0` means the sample carried none.
    /// Cross-RMW bridges read the `bridge_origin` tag from the attachment.
    template <typename M, typename F,
              typename = typename std::enable_if<
                  std::is_convertible<F, void (*)(const M&, const uint8_t*, size_t)>::value>::type>
    Result create_subscription_with_info(Subscription<M>& out, const char* topic, F callback,
                                         const QoS& qos = QoS::default_profile(),
                                         const SubscriptionOptions& options = {});

#if defined(NANO_ROS_SAFETY_E2E)
    /// Phase 269 W3 — Create a **callback-style** subscription that surfaces the
    /// sample's E2E integrity status (CRC + sequence gap/dup) alongside the typed
    /// message — the C++ component-callback analog of Rust's
    /// `create_subscription_…_with_safety` / `CallbackCtx::integrity()`.
    ///
    /// Arena-registered like the `create_subscription_with_info` overload above
    /// (so `options.sched_context` is functional). The handler is invoked as
    /// `callback(const M&, const nros_cpp_integrity_status_t&)` on each new sample.
    ///
    /// Requires `NANO_ROS_SAFETY_E2E=ON` (lowered from
    /// `[system].features = ["safety"]` via `NanoRosCapabilities.cmake`).
    ///
    /// CONSTRAINT: do not move `out` after this returns — the executor arena
    /// holds `&out` as the trampoline context.
    template <typename M, typename F,
              typename = typename std::enable_if<std::is_convertible<
                  F, void (*)(const M&, const nros_cpp_integrity_status_t&)>::value>::type>
    Result create_subscription_with_safety(Subscription<M>& out, const char* topic, F callback,
                                           const QoS& qos = QoS::default_profile(),
                                           const SubscriptionOptions& options = {});
#endif // NANO_ROS_SAFETY_E2E

    /// Create a service server.
    ///
    /// @tparam S  Service type (must define nested Request and Response with TYPE_NAME/TYPE_HASH).
    /// @param out           Receives the initialized service server.
    /// @param service_name  Service name (null-terminated).
    /// @param qos           QoS profile (default: services preset).
    template <typename S>
    Result create_service(Service<S>& out, const char* service_name,
                          const QoS& qos = QoS::services());

    /// Create a **callback-style** service server (rclcpp dispatch model;
    /// Phase 189.M3.3.e). Unlike the poll-style overload above, this
    /// arena-registers the service so it owns a real executor handle and its
    /// request handler runs during `spin_once` — making `options.sched_context`
    /// functional. `callback` must be convertible to
    /// `void(const S::Request&, S::Response&)` (a plain function pointer or
    /// empty-capture lambda); it fills `response` from `request`. The SFINAE
    /// guard keeps the poll-style 3-arg overload unambiguous (a `QoS` is not
    /// convertible to the handler type).
    ///
    /// CONSTRAINT: do not move `out` after this returns — the executor arena
    /// holds `&out` as the dispatch context.
    template <typename S, typename F,
              typename = typename std::enable_if<std::is_convertible<
                  F, void (*)(const typename S::Request&, typename S::Response&)>::value>::type>
    Result create_service(Service<S>& out, const char* service_name, F callback,
                          const QoS& qos = QoS::services(), const ServiceOptions& options = {});

    /// Create a service client.
    ///
    /// @tparam S  Service type (must define nested Request and Response with TYPE_NAME/TYPE_HASH).
    /// @param out           Receives the initialized service client.
    /// @param service_name  Service name (null-terminated).
    /// @param qos           QoS profile (default: services preset).
    template <typename S>
    Result create_client(Client<S>& out, const char* service_name,
                         const QoS& qos = QoS::services());

    /// Create a **callback-style** service client (rclcpp async dispatch;
    /// Phase 189.M3.3.f). Arena-registered, so it owns a real executor handle and
    /// its response handler runs during `spin_once` — making
    /// `options.sched_context` functional. `callback` must be convertible to
    /// `void(const S::Response&)`; send requests with
    /// `Client<S>::async_send_request`. The SFINAE guard keeps the future-style
    /// 3-arg overload unambiguous.
    ///
    /// CONSTRAINT: do not move `out` after this returns — the executor arena
    /// holds `&out` as the response dispatch context.
    template <typename S, typename F,
              typename = typename std::enable_if<
                  std::is_convertible<F, void (*)(const typename S::Response&)>::value>::type>
    Result create_client(Client<S>& out, const char* service_name, F callback,
                         const QoS& qos = QoS::services(), const ClientOptions& options = {});

    /// Create an action server.
    ///
    /// Goals are auto-accepted during spin_once(). Use try_recv_goal() to poll.
    ///
    /// @tparam A  Action type (must define nested Goal, Result, Feedback with TYPE_NAME/TYPE_HASH).
    /// @param out          Receives the initialized action server.
    /// @param action_name  Action name (null-terminated).
    /// @param qos          QoS profile (default: services preset).
    /// @param options      Named options; `options.sched_context` (M3.3.c) binds
    ///                     the goal-service dispatch onto a scheduling context.
    template <typename A>
    Result create_action_server(ActionServer<A>& out, const char* action_name,
                                const QoS& qos = QoS::services(),
                                const ActionServerOptions& options = {});

    /// Create an action client.
    ///
    /// @tparam A  Action type (must define nested Goal, Result, Feedback with TYPE_NAME/TYPE_HASH).
    /// @param out          Receives the initialized action client.
    /// @param action_name  Action name (null-terminated).
    /// @param qos          QoS profile (default: services preset).
    template <typename A>
    Result create_action_client(ActionClient<A>& out, const char* action_name,
                                const QoS& qos = QoS::services());

    /// Phase 122.3.d.b — Create an L1 polling-mode action server.
    /// Caller drives the lifecycle (no executor callback). See
    /// `polling_action_server.hpp` for usage.
    template <typename A>
    Result create_polling_action_server(PollingActionServer<A>& out, const char* action_name);

    /// Phase 122.3.d.b — Create an L1 polling-mode action client.
    template <typename A>
    Result create_polling_action_client(PollingActionClient<A>& out, const char* action_name);

    /// Issue 0278 — Create a latest-value polling subscription (the nano-ros
    /// analog of `autoware_utils::InterProcessPollingSubscriber`): a poll-mode
    /// subscription plus a retained last value, read repeatably via
    /// `PollingSubscription<M>::take_data()` / `take_new_data()`. See
    /// `polling_subscription.hpp`.
    template <typename M>
    Result create_polling_subscription(PollingSubscription<M>& out, const char* topic,
                                       const QoS& qos = QoS::default_profile());

    /// Create a repeating timer on a CLOCK — `rclcpp::create_timer(node, clock,
    /// period, callback)`, phase-425 W4.
    ///
    /// `create_wall_timer` is the steady one: it advances with the executor's
    /// monotonic spin delta, so no simulator can slow it down. This one advances
    /// with `clock`, which is what a node in a simulation or a bag replay wants:
    /// a `NROS_CLOCK_ROS_TIME` timer stops while `/clock` is paused, tracks the
    /// replay rate, and restarts its period on a backwards jump rather than
    /// stalling for the length of it.
    ///
    /// With no `/clock` source installed a ROS-time clock reads system time, the
    /// same fallback `rclcpp::Clock` has — a node written for simulation still
    /// runs standalone.
    ///
    /// The node's own clock (`get_clock()`) is ROS time, as in rclcpp, so
    /// `node.create_timer(t, *node.get_clock(), 100, on_tick)` is the usual
    /// call.
    ///
    /// @param out        Receives the initialized timer.
    /// @param clock      The clock that advances the timer.
    /// @param period_ms  Timer period in milliseconds, ON THAT CLOCK.
    /// @param callback   C function pointer invoked on each tick.
    /// @param context    User context passed to the callback (may be nullptr).
    Result create_timer(Timer& out, const Clock& clock, uint64_t period_ms,
                        nros_cpp_timer_callback_t callback, void* context = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        size_t handle_id = 0;
        nros_cpp_ret_t ret = nros_cpp_timer_create_on_clock(
            executor_handle_, static_cast<uint8_t>(clock.get_clock_type()), period_ms, callback,
            context, &handle_id);
        if (ret == 0) {
            out.executor_ = executor_handle_;
            out.handle_id_ = handle_id;
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Create a repeating WALL timer — `rclcpp::Node::create_wall_timer`.
    ///
    /// The callback fires during `spin_once()` at the specified period, measured
    /// on the platform's monotonic clock. For a timer that follows simulated
    /// time, see `create_timer` above.
    ///
    /// @param out        Receives the initialized timer.
    /// @param period_ms  Timer period in milliseconds.
    /// @param callback   C function pointer invoked on each tick.
    /// @param context    User context passed to the callback (may be nullptr).
    Result create_wall_timer(Timer& out, uint64_t period_ms, nros_cpp_timer_callback_t callback,
                             void* context = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        size_t handle_id = 0;
        nros_cpp_ret_t ret =
            nros_cpp_timer_create(executor_handle_, period_ms, callback, context, &handle_id);
        if (ret == 0) {
            out.executor_ = executor_handle_;
            out.handle_id_ = handle_id;
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Create a one-shot timer.
    ///
    /// The callback fires once after the specified delay.
    ///
    /// @param out       Receives the initialized timer.
    /// @param delay_ms  Delay in milliseconds before the callback fires.
    /// @param callback  C function pointer invoked once.
    /// @param context   User context passed to the callback (may be nullptr).
    Result create_timer_oneshot(Timer& out, uint64_t delay_ms, nros_cpp_timer_callback_t callback,
                                void* context = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        size_t handle_id = 0;
        nros_cpp_ret_t ret = nros_cpp_timer_create_oneshot(executor_handle_, delay_ms, callback,
                                                           context, &handle_id);
        if (ret == 0) {
            out.executor_ = executor_handle_;
            out.handle_id_ = handle_id;
            out.initialized_ = true;
        }
        return Result(ret);
    }

    // -- Phase 273 (RFC-0047) — Callback-group API -------------------------

    /// Create a named callback-group token.
    ///
    /// The returned `CallbackGroup` may be passed to `create_timer_in`,
    /// `create_subscription_in`, or `create_publisher_in` to associate entities
    /// with the group's SchedContext (resolved via `group_sched_table`).
    ///
    /// @param name  Group name — must be a string literal or static-lifetime
    ///              string; the pointer is stored directly (no copy).
    CallbackGroup create_callback_group(const char* name) { return CallbackGroup{name}; }

    /// Create a repeating timer **in** a callback group (RFC-0047).
    ///
    /// Like `create_wall_timer` but associates the timer with `group` so the
    /// executor binds it to the group's SchedContext via `group_sched_table`.
    /// `group.get_name() == nullptr` or empty falls back to node default.
    ///
    /// @param group      Callback group (from `create_callback_group`).
    /// @param out        Receives the initialized timer.
    /// @param period_ms  Timer period in milliseconds.
    /// @param callback   C function pointer invoked on each tick.
    /// @param context    User context passed to the callback (may be nullptr).
    Result create_timer_in(const CallbackGroup& group, Timer& out, uint64_t period_ms,
                           nros_cpp_timer_callback_t callback, void* context = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        size_t handle_id = 0;
        nros_cpp_ret_t ret = nros_cpp_timer_create_in_group(
            executor_handle_, &handle_, period_ms, callback, context, group.get_name(), &handle_id);
        if (ret == 0) {
            out.executor_ = executor_handle_;
            out.handle_id_ = handle_id;
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Create a **callback-style** subscription **in** a callback group (RFC-0047).
    ///
    /// Like the callback-style `create_subscription` but associates the
    /// subscription with `group` so the executor binds it to the group's
    /// SchedContext via `group_sched_table`. Out-of-line definition in
    /// `subscription.hpp`.
    ///
    /// @tparam M  Message type (must define TYPE_NAME, TYPE_HASH, ffi_deserialize).
    /// @param group      Callback group.
    /// @param out        Receives the initialized subscription (callback mode).
    /// @param topic      Topic name (null-terminated).
    /// @param callback   Handler invoked as `callback(const M&)` per sample.
    /// @param qos        QoS profile.
    /// @param options    Named subscription options.
    template <
        typename M, typename F,
        typename = typename std::enable_if<std::is_convertible<F, void (*)(const M&)>::value>::type>
    Result create_subscription_in(const CallbackGroup& group, Subscription<M>& out,
                                  const char* topic, F callback,
                                  const QoS& qos = QoS::default_profile(),
                                  const SubscriptionOptions& options = {});

    /// Create a publisher **in** a callback group (API symmetry; RFC-0047).
    ///
    /// Publishers have no dispatched callback — the group parameter is accepted
    /// for API symmetry but has no scheduling effect (documented in RFC-0047 §OQ1
    /// follow-up).
    ///
    /// @tparam M  Message type.
    /// @param group  Callback group (accepted for symmetry; no scheduling effect).
    /// @param out    Receives the initialized publisher.
    /// @param topic  Topic name.
    /// @param qos    QoS profile.
    template <typename M>
    Result create_publisher_in(const CallbackGroup& /* group */, Publisher<M>& out,
                               const char* topic, const QoS& qos = QoS::default_profile()) {
        return create_publisher<M>(out, topic, qos);
    }

    /// Create a guard condition for cross-thread signaling.
    ///
    /// The callback fires during `spin_once()` when `guard.trigger()` is called.
    ///
    /// @param out       Receives the initialized guard condition.
    /// @param callback  C function pointer invoked when triggered.
    /// @param context   User context passed to the callback (may be nullptr).
    Result create_guard_condition(GuardCondition& out, nros_cpp_guard_callback_t callback,
                                  void* context = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        nros_cpp_ret_t ret =
            nros_cpp_guard_condition_create(executor_handle_, callback, context, out.storage_);
        if (ret == 0) {
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Destructor — releases node resources.
    ~Node() {
        if (initialized_) {
            nros_cpp_node_destroy(&handle_);
            initialized_ = false;
        }
    }

    // Move semantics (non-copyable)
    Node(Node&& other)
        : handle_(other.handle_), initialized_(other.initialized_),
          executor_handle_(other.executor_handle_), clock_(other.clock_) {
        other.initialized_ = false;
        other.executor_handle_ = nullptr;
    }

    Node& operator=(Node&& other) {
        if (this != &other) {
            if (initialized_) {
                nros_cpp_node_destroy(&handle_);
            }
            handle_ = other.handle_;
            initialized_ = other.initialized_;
            executor_handle_ = other.executor_handle_;
            clock_ = other.clock_;
            other.initialized_ = false;
            other.executor_handle_ = nullptr;
        }
        return *this;
    }

  private:
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    nros_cpp_node_t handle_;
    bool initialized_;
    void* executor_handle_; // Set by nros::init() via friendship
    // Issue 0789 — the node's own clock, ROS time as in rclcpp. Constructing
    // it touches no platform service (only a steady clock records an epoch),
    // so a Node in static storage stays as cheap to create as it was.
    Clock clock_;

    friend class Executor;
    friend class NodeBuilder;
    friend class ComponentNode; // Phase 242.1 — ctor-creates the owned node
    friend Result init(const char* locator, uint8_t domain_id);
    friend Result init(const char* locator, uint8_t domain_id, const char* session_name);
    friend Result init_with_rmw(const char* rmw, const char* locator, uint8_t domain_id,
                                const char* session_name);
    friend Result shutdown();
    friend bool ok();
    friend Result create_node(Node& out, const char* name, const char* ns);
    friend Result create_node_on(Node& out, void* executor_handle, const char* name,
                                 const char* ns);
    friend Result spin_once(int32_t timeout_ms);
    friend Result spin();
    friend Result spin(uint32_t duration_ms, int32_t poll_ms);
    friend void* global_handle();

    // Global executor inline storage for init/shutdown free functions.
    //
    // Use a template-static-member trick instead of a function-local static.
    // Function-local statics need __cxa_guard_acquire/release on first-call
    // initialisation; on NuttX the resulting guard logic returns NULL for
    // the storage pointer (observed empirically with LTO on armv7a-nuttx-eabihf,
    // even with constant-initialisation `= {}`). A template static member is
    // emitted into .bss like a file-scope variable and gets COMDAT-folded by
    // the linker, sidestepping the guarded-init path entirely.
    template <int = 0> struct GlobalStorageHolder {
        alignas(8) static uint8_t storage[NROS_CPP_EXECUTOR_STORAGE_SIZE];
        static bool initialized;
    };
    static uint8_t* global_storage() { return GlobalStorageHolder<>::storage; }
    static bool& global_initialized() { return GlobalStorageHolder<>::initialized; }
};

// Out-of-class definitions for Node::GlobalStorageHolder<> — the template
// machinery means these get emitted as COMDAT symbols, so multiple TUs
// including this header all collapse to a single .bss allocation.
template <int N>
alignas(8) uint8_t Node::GlobalStorageHolder<N>::storage[NROS_CPP_EXECUTOR_STORAGE_SIZE] = {};
template <int N> bool Node::GlobalStorageHolder<N>::initialized = false;

// -- Free function implementations --

inline Result init(const char* locator, uint8_t domain_id) {
    // Issue 0329 — forward RAW to the 3-arg overload, which resolves the whole
    // ladder (baked `NROS_ENTRY_LOCATOR`/`NROS_ENTRY_DOMAIN_ID` rungs, hosted
    // default, then the env overlay in the Rust resolver behind `nros_cpp_init`).
    // This overload previously re-resolved the locator rung itself — a second,
    // partial copy of the same ladder (it never applied `NROS_ENTRY_DOMAIN_ID`),
    // duplicating what the 3-arg already does. Phase 266: unified default session
    // name "node".
    return init(locator, domain_id, "node");
}

inline Result init(const char* locator, uint8_t domain_id, const char* session_name) {
    // NROS_CPP_RET_INVALID_ARGUMENT = -3 (defined in nros_cpp_ffi.h
    // which isn't included from this header — duplicate the value
    // inline; generated header is the source of truth).
    if (session_name == nullptr) {
        return Result(-3);
    }
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
    // Issue #39 — apply the same `$NROS_LOCATOR` / `$ROS_DOMAIN_ID` env
    // fallback as the 2-arg `init()` when `locator` is null / `domain_id` is
    // 0. This makes `init_with_launch_auto()` (which delegates here with a
    // null locator) honor the env overlay instead of passing a null locator
    // to the backend → TransportError / degraded session.
    // Issue 0330 — the hard local default is GONE entirely (it was a zenoh
    // fact in an RMW-blind header); the backend now supplies it.
    // RFC-0045 / issue #206 — the env overlay (NROS_LOCATOR / ROS_DOMAIN_ID /
    // NROS_NODE_NAME) moved into the shared Rust resolver behind
    // nros_cpp_init (precedence model A: hosted env > this baked chain >
    // compiled default; malformed or >232 ROS_DOMAIN_ID is an init ERROR,
    // never a silent domain 0). This header only assembles the baked rung.
#endif
    // Phase-287 W6 — compile-time connect defaults, so ONE portable source
    // works native + embedded. `NROS_ENTRY_LOCATOR` / `NROS_ENTRY_DOMAIN_ID`
    // are target compile definitions the embedded board gate bakes
    // (NanoRosEntry.cmake; Kconfig on Zephyr via <nros/main.hpp>); on native
    // they are undefined. Precedence (model A, RFC-0045/#206): env (hosted,
    // applied in the Rust resolver) > explicit arg > baked macro > default.
#ifdef NROS_ENTRY_LOCATOR
    if (locator == nullptr) {
        locator = NROS_ENTRY_LOCATOR;
    }
#endif
#ifdef NROS_ENTRY_DOMAIN_ID
    // Only the UNSET sentinel (0) folds the baked macro in; an explicit
    // argument — including kDomainIdExplicitZero (255) for a literal
    // domain 0 (issue #227) — passes through untouched.
    if (domain_id == 0) {
        domain_id = static_cast<uint8_t>(NROS_ENTRY_DOMAIN_ID);
    }
#endif
    // Issue 0330 — there is deliberately NO hosted "tcp/127.0.0.1:7447"
    // fallback here. That value is a *zenoh* fact and this header is
    // RMW-blind; a cyclonedds or xrce build must not carry it. A null
    // locator flows through `nros_cpp_init` (→ `None` → the RFC-0045
    // resolver's empty bottom rung) to whichever backend is linked, and
    // that backend applies its own default (zenoh:
    // `nros_rmw_zenoh::DEFAULT_LOCATOR`; xrce: its agent default;
    // cyclonedds: ignores the locator). Precedence is otherwise unchanged:
    // hosted env > explicit arg > baked macro > backend default.

    // Phase 128.C.1 / phase-241.D3-rev — RMW-blind init. The selected
    // backend registers itself before `main` via its `.init_array` ctor
    // (RFC-0042 §D3.3), and `nros_cpp_init` additionally calls the weak
    // `nros_app_register_backends()` board-override hook for RTOS targets
    // where `.init_array` ctors do not run. No `#ifdef NROS_RMW_*` chain
    // here, no CMake-driven fan-out — the user's
    // `target_link_libraries(... NanoRos::Rmw::<name>)` is the only
    // selector.
    // Issue 1050 defect (3) — the RMW selector's BAKED rung. `NROS_ENTRY_RMW`
    // is a target compile definition the entry gate bakes, exactly like
    // `NROS_ENTRY_LOCATOR` / `NROS_ENTRY_DOMAIN_ID` above; undefined means the
    // image names no backend and the registry must contain exactly one.
    //
    // This is what the paragraph above could not express. "The user's
    // `target_link_libraries(... NanoRos::Rmw::<name>)` is the only selector"
    // is true of the LINK and false of the REGISTRY: a hosted archive
    // registers whatever it carries, from `.init_array`, before `main`.
#ifdef NROS_ENTRY_RMW
    const char* rmw = NROS_ENTRY_RMW;
#else
    const char* rmw = nullptr;
#endif
    nros_cpp_ret_t ret =
        nros_cpp_init_rmw(rmw, locator, domain_id, session_name, nullptr, Node::global_storage());
    if (ret == 0) {
        Node::global_initialized() = true;
    }
    return Result(ret);
}

inline Result init_with_rmw(const char* rmw, const char* locator, uint8_t domain_id,
                            const char* session_name) {
    // NROS_CPP_RET_INVALID_ARGUMENT = -3; see the 3-arg overload for why the
    // value is duplicated here rather than included.
    if (session_name == nullptr) {
        return Result(-3);
    }
#ifdef NROS_ENTRY_LOCATOR
    if (locator == nullptr) {
        locator = NROS_ENTRY_LOCATOR;
    }
#endif
#ifdef NROS_ENTRY_DOMAIN_ID
    if (domain_id == 0) {
        domain_id = static_cast<uint8_t>(NROS_ENTRY_DOMAIN_ID);
    }
#endif
    // No `NROS_ENTRY_RMW` fallback here: an explicit argument that resolves to
    // nullptr is the caller saying "no selector", and quietly substituting the
    // bake would make this overload unable to express that.
    nros_cpp_ret_t ret =
        nros_cpp_init_rmw(rmw, locator, domain_id, session_name, nullptr, Node::global_storage());
    if (ret == 0) {
        Node::global_initialized() = true;
    }
    return Result(ret);
}

inline Result shutdown() {
    if (!Node::global_initialized()) {
        return Result::success();
    }
    nros_cpp_ret_t ret = nros_cpp_fini(Node::global_storage());
    Node::global_initialized() = false;
    return Result(ret);
}

// -- Phase 212.L.5 launch-aware init --
//
// Both `init_with_launch_auto` and `init_with_launch(path)` delegate to
// the existing 3-arg `init` after resolving the launch overlay (today:
// env vars only — see header docs for the follow-up plan). The session
// name falls back to `"nros_cpp"` so existing callsites keep working.

inline Result init_with_launch_auto(int argc, char** argv, const char* session_name) {
    (void)argc;
    (void)argv;
    // TODO (Phase 212.L.5 follow-up):
    //   1. If $NROS_RUNTIME_OVERLAY is set, read the JSON sidecar and
    //      fold its params/remaps/env into the init call.
    //   2. Else walk <CARGO_MANIFEST_DIR>/launch/* and parse the XML
    //      in-process.
    // For now the env overlay (NROS_LOCATOR / ROS_DOMAIN_ID consumed by
    // the 2-arg `init`) is the only channel.
    const char* name = (session_name != nullptr) ? session_name : "nros_cpp";
    return init(nullptr, 0, name);
}

inline Result init_with_launch(const char* path, int argc, char** argv, const char* session_name) {
    (void)argc;
    (void)argv;
    // NROS_CPP_RET_INVALID_ARGUMENT = -3 (mirrors the 3-arg init guard).
    if (path == nullptr) {
        return Result(-3);
    }
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
    // Verify the file exists so misspelled paths fail fast at init time
    // instead of surfacing as a silently-empty overlay later.
    if (FILE* f = std::fopen(path, "rb")) {
        std::fclose(f);
    } else {
        return Result(ErrorCode::NotInitialized);
    }
#endif
    // TODO (Phase 212.L.5 follow-up): parse `path` as launch XML and
    // fold params/remaps/env into the init call. Today the env overlay
    // is the only channel.
    const char* name = (session_name != nullptr) ? session_name : "nros_cpp";
    return init(nullptr, 0, name);
}

/// Check if the nros session is initialized.
inline bool ok() {
    return Node::global_initialized();
}

/// Create a node (convenience — uses the global executor).
///
/// This is the primary way to create nodes after calling `nros::init()`.
///
/// @param out   Receives the initialized node.
/// @param name  Node name.
/// @param ns    Node namespace, or nullptr for "/".
inline Result create_node(Node& out, const char* name, const char* ns = nullptr) {
    if (!Node::global_initialized()) {
        return Result(ErrorCode::NotInitialized);
    }
    out.executor_handle_ = Node::global_storage();
    return Node::create(out, name, ns);
}

/// Phase 274.W2 — create a node on an explicit executor handle.
///
/// Used by per-tier setup functions (emitted by `nros codegen entry --lang
/// cpp` for multi-tier workspaces) where each tier's setup runs on the
/// tier's borrowed executor, not the global one. The executor handle is the
/// `void*` passed to the tier's `setup(void* executor)` callback.
///
/// @param out              Receives the initialized node.
/// @param executor_handle  Explicit executor handle (from a tier setup param).
/// @param name             Node name.
/// @param ns               Node namespace, or nullptr for "/".
inline Result create_node_on(Node& out, void* executor_handle, const char* name,
                             const char* ns = nullptr) {
    if (executor_handle == nullptr) {
        return Result(ErrorCode::NotInitialized);
    }
    out.executor_handle_ = executor_handle;
    return Node::create(out, name, ns);
}

/// Phase 123.B.4 — value-returning factory. Wraps `create_node`
/// in the `Expected<Node>` envelope so users can write
/// `auto n = nros::make_node("foo");` in the rclcpp-style.
inline Expected<Node> make_node(const char* name, const char* ns = nullptr) {
    Node n;
    Result r = create_node(n, name, ns);
    if (!r.ok()) return Expected<Node>::error(r);
    return Expected<Node>::ok(::std::move(n));
}

// -- Executor::create_node implementation (requires full Node definition) --

inline Result Executor::create_node(Node& out, const char* name, const char* ns) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    out.executor_handle_ = storage_;
    return Node::create(out, name, ns);
}

// -- Phase 104.C.9 — NodeBuilder ----------------------------------------
//
// Mirrors Rust's `Executor::node_builder(name).rmw(...).locator(...).
// domain_id(...).namespace(...).sched(...).build()` chain. The C++
// wrapper is value-typed and stack-allocated; it accumulates options
// into an inline `nros_cpp_node_options_t` and ships it to
// `nros_cpp_node_create_ex` on `.build()`.
//
// Usage:
// ```cpp
// nros::Node node;
// NROS_TRY(executor.node_builder("egress")
//              .rmw("cyclonedds")
//              .domain_id(0)
//              .build(node));
// ```

class NodeBuilder {
  public:
    NodeBuilder(void* executor_handle, const char* name)
        : executor_handle_(executor_handle), name_(name),
          options_(nros_cpp_node_get_default_options()) {}

    /// Bind this Node to the named RMW backend. The name must match a
    /// backend registered with `nros_rmw_cffi_register_named` (or its
    /// auto-ctor equivalent). Empty/nullptr selects the first-
    /// registered backend — the single-backend convenience path.
    NodeBuilder& rmw(const char* name) {
        copy_bounded(name, options_.rmw_name, &options_.rmw_name_len, NROS_CPP_RMW_NAME_LEN);
        return *this;
    }

    /// Override the Node's locator (`tcp/...`, `udp/...`, `serial:...`).
    /// Empty/nullptr inherits the executor's locator.
    NodeBuilder& locator(const char* loc) {
        copy_bounded(loc, options_.locator, &options_.locator_len, NROS_CPP_LOCATOR_LEN);
        return *this;
    }

    /// Override the Node's domain ID. Pass `NROS_CPP_DOMAIN_ID_INHERIT`
    /// (the default) to inherit from the executor.
    NodeBuilder& domain_id(uint32_t id) {
        options_.domain_id_override = id;
        return *this;
    }

    /// Set the Node's namespace (mirrors `rclcpp::Node`'s ctor). Empty
    /// or nullptr defaults to `"/"` at build time.
    NodeBuilder& namespace_(const char* ns) {
        copy_bounded(ns, options_.namespace_, &options_.namespace_len, NROS_CPP_NAMESPACE_LEN);
        return *this;
    }

    /// Bind every handle created via this Node to `sc_id` as its
    /// default SchedContext. 0 = executor default Fifo.
    NodeBuilder& sched(uint8_t sc_id) {
        options_.sched_context_id = sc_id;
        return *this;
    }

    /// Materialize the Node.
    Result build(Node& out) const {
        if (!executor_handle_) return Result(ErrorCode::NotInitialized);
        out.executor_handle_ = executor_handle_;
        nros_cpp_ret_t ret =
            nros_cpp_node_create_ex(executor_handle_, name_, &options_, &out.handle_);
        if (ret == 0) {
            out.initialized_ = true;
        }
        return Result(ret);
    }

  private:
    static void copy_bounded(const char* src, uint8_t* dst, size_t* dst_len, size_t cap) {
        size_t n = 0;
        if (src != nullptr) {
            while (src[n] != '\0' && n < cap) {
                dst[n] = static_cast<uint8_t>(src[n]);
                ++n;
            }
        }
        // Zero out the tail so stale bytes don't leak across reuses.
        for (size_t i = n; i < cap; ++i) {
            dst[i] = 0;
        }
        *dst_len = n;
    }

    void* executor_handle_;
    const char* name_;
    nros_cpp_node_options_t options_;
};

inline NodeBuilder Executor::node_builder(const char* name) {
    return NodeBuilder(initialized_ ? handle() : nullptr, name);
}

} // namespace nros

#endif // NROS_CPP_NODE_HPP
