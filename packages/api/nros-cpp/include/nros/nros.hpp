// nros-cpp: Umbrella header
// Include this single header to get the full nros C++ API.
//
// Freestanding C++ compatible — no STL, no exceptions, no RTTI required.

/**
 * @file nros.hpp
 * @ingroup grp_init
 * @brief Umbrella header — pulls in every public C++ API surface.
 */

#ifndef NROS_CPP_HPP
#define NROS_CPP_HPP

// Phase 118.D — pull cbindgen-generated FFI before any wrapper hpp so
// `nros/qos.hpp`'s `#ifndef NROS_CPP_FFI_H` guard skips its local
// fallback definitions in favor of the canonical types.
#include "nros_cpp_ffi.h"

#include "nros/log.hpp"
#include "nros/result.hpp"
// Issue 0789 — the clock / time / duration surface. `node.now()` and
// `node.get_clock()->now()` are what a ported rclcpp publisher calls to
// stamp a header, so the umbrella carries all three. Phase 379 W5 moved
// `duration.hpp` ABOVE `qos.hpp`: the QoS deadline / lifespan / lease
// accessors take and return `nros::Duration`. `qos.hpp` also includes it
// directly, so this order is legibility rather than load-bearing.
#include "nros/duration.hpp"
#include "nros/time.hpp"
#include "nros/clock.hpp"
#include "nros/qos.hpp"
#include "nros/options.hpp"
#include "nros/future.hpp"
#include "nros/stream.hpp"
// RFC-0088 D5 — nros::SerializationFormat / format_of<M> / linked_format().
#include "nros/serialization_format.hpp"
// Phase 84.G8: node.hpp no longer pulls in the heavy entity headers —
// each entity header carries its own out-of-line `Node::create_X<>()`
// template definition. The umbrella pulls in every entity explicitly so
// `#include <nros/nros.hpp>` still yields the full API.
#include "nros/node.hpp"
#include "nros/publisher.hpp"
#include "nros/subscription.hpp"
#include "nros/service.hpp"
#include "nros/client.hpp"
#include "nros/action_server.hpp"
#include "nros/action_client.hpp"
#include "nros/polling_action_server.hpp"
#include "nros/polling_action_client.hpp"
#include "nros/polling_subscription.hpp"
#include "nros/parameter.hpp"
#include "nros/tick_ctx.hpp"
#include "nros/lifecycle.hpp"
// phase-417 W2.b — the component model carries the rclcpp-shaped, value-returning
// `declare_parameter<T>` / `get_parameter<T>` / `has_parameter` (RFC-0044). It sat
// OUTSIDE this umbrella, so a ported rclcpp node reaching through <nros/nros.hpp>
// got `nros::Node`, which has no parameter method at all, while the faithful
// surface was one include away and only the generated entry pulled it in. That is
// 26 ledger rows filed as gaps in a capability we ship. Freestanding-safe: its
// `<string>` use is gated on NROS_CPP_STD (issue 0112) and its placement-new
// shim is gated on Zephyr's stub <new>.
#include "nros/component_node.hpp"
// phase-417 stage 6 step A — `nros::create_subscription_raw`, the
// arena-registration entry point `rclcpp::Node::create_subscription`
// below uses so it has no dispatch of its own.
#include "nros/component.hpp"

namespace nros {

/// Get the global executor handle for Future::wait().
///
/// Returns the raw storage pointer used by the global `init()`/`spin_once()`
/// free functions. Use with `Future::wait(nros::global_handle(), ...)`.
///
/// @return Executor handle, or nullptr if not initialized.
inline void* global_handle() {
    if (!Node::global_initialized()) return nullptr;
    return Node::global_storage();
}

/// Drive transport I/O and dispatch callbacks.
///
/// Call this periodically so subscriptions can receive data.
/// When using manual-poll (no callbacks), this drives the network layer.
///
/// @param timeout_ms  Maximum time to block waiting for I/O (default: 10ms).
/// @return Result indicating success or failure.
inline Result spin_once(int32_t timeout_ms = 10) {
    if (!Node::global_initialized()) {
        return Result(ErrorCode::NotInitialized);
    }
    return Result(nros_cpp_spin_once(Node::global_storage(), timeout_ms));
}

/// Register a callback to run BEFORE the global session's entities are torn
/// down — issue 0790.
///
/// rclcpp hangs the shutdown hooks on `Context`, which nano-ros does not have
/// (phase-379's init stage records the collapse into one support object), so
/// they live on the executor — here, the global one `nros::init()` opened and
/// `nros::shutdown()` closes.
///
/// This is the phase with no workaround: the callback runs while every entity
/// still works, so a node can publish a final state, answer a last request,
/// park an actuator or release a bus. `nros::on_shutdown` below runs after
/// teardown, when none of that is possible any more.
///
/// A CLEAN-STOP facility: a watchdog reset, a hard fault or an abort does not
/// come through `nros::shutdown()`, so it does not come through here either.
///
/// @param callback  Function to invoke. Must not be null.
/// @param context   Opaque pointer handed back to `callback`. Must stay valid
///                  until the callback runs or is removed.
/// @param out       Receives the removal handle. Optional.
inline Result pre_shutdown(ShutdownCallback callback, void* context = nullptr,
                           PreShutdownCallbackHandle* out = nullptr) {
    void* executor = global_handle();
    if (executor == nullptr) {
        return Result(ErrorCode::NotInitialized);
    }
    nros_cpp_shutdown_callback_handle_t raw = NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID;
    nros_cpp_ret_t ret = nros_cpp_add_pre_shutdown_callback(executor, callback, context, &raw);
    if (out != nullptr) *out = PreShutdownCallbackHandle(raw);
    return Result(ret);
}

/// Register a callback to run AFTER the global session's entities are torn
/// down — `rclcpp::on_shutdown`. Issue 0790.
///
/// The entities are gone by the time it runs; use [`pre_shutdown`] for
/// anything that needs the wire.
///
/// @see pre_shutdown for the parameter and error contract.
inline Result on_shutdown(ShutdownCallback callback, void* context = nullptr,
                          OnShutdownCallbackHandle* out = nullptr) {
    void* executor = global_handle();
    if (executor == nullptr) {
        return Result(ErrorCode::NotInitialized);
    }
    nros_cpp_shutdown_callback_handle_t raw = NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID;
    nros_cpp_ret_t ret = nros_cpp_add_on_shutdown_callback(executor, callback, context, &raw);
    if (out != nullptr) *out = OnShutdownCallbackHandle(raw);
    return Result(ret);
}

/// Remove a callback registered with [`pre_shutdown`]. `true` when `handle`
/// named a live one — "it was not there" is an ordinary answer, as in rclcpp.
inline bool remove_pre_shutdown_callback(PreShutdownCallbackHandle handle) {
    void* executor = global_handle();
    if (executor == nullptr) return false;
    return nros_cpp_remove_pre_shutdown_callback(executor, handle.value()) == 0;
}

/// Remove a callback registered with [`on_shutdown`].
/// @see remove_pre_shutdown_callback
inline bool remove_on_shutdown_callback(OnShutdownCallbackHandle handle) {
    void* executor = global_handle();
    if (executor == nullptr) return false;
    return nros_cpp_remove_on_shutdown_callback(executor, handle.value()) == 0;
}

/// Phase 123.B.2 — block until `nros::ok()` returns false.
///
/// Mirror of `rclcpp::spin(node)`. The typical pattern in user
/// code is: install a SIGINT handler that calls `nros::shutdown()`
/// (which flips `ok()` to false), then `nros::spin()` from `main`.
///
/// Returns the first non-success `spin_once` result, or
/// `Result::success()` after a clean shutdown.
inline Result spin() {
    if (!Node::global_initialized()) {
        return Result(ErrorCode::NotInitialized);
    }
    Result last = Result::success();
    while (ok()) {
        last = Result(nros_cpp_spin_once(Node::global_storage(), 10));
        if (!last.ok()) return last;
    }
    return last;
}

/// Spin for a duration (blocking).
///
/// Repeatedly calls `spin_once()` until `duration_ms` has elapsed.
/// Convenience wrapper around the global executor.
///
/// @param duration_ms  Total time to spin, in milliseconds.
/// @param poll_ms      Individual spin_once timeout (default: 10ms).
/// @return Result from the last spin_once call.
inline Result spin(uint32_t duration_ms, int32_t poll_ms = 10) {
    if (!Node::global_initialized()) {
        return Result(ErrorCode::NotInitialized);
    }
    // Issue 0329 — forward to the single budgeted-spin CFFI entry point. This
    // loop previously budgeted by ITERATION count (`elapsed += timeout`), which
    // collapsed to milliseconds when `spin_once` returned early on a signaled
    // wake — the exact bug `Executor::spin` fixed in Phase 118.C. The correct
    // wall-clock budget now lives once, Rust-side, in `nros_cpp_spin_for`.
    return Result(nros_cpp_spin_for(Node::global_storage(), duration_ms, poll_ms));
}

} // namespace nros

#ifdef NROS_CPP_STD
#include "nros/std_compat.hpp"
#endif

// ============================================================================
// rclcpp:: — the process-level surface (RFC-0089 stage 6, step A)
// ============================================================================
//
// `rclcpp::init` / `shutdown` / `ok` / `Node` / `spin` / `spin_some` / `Rate` /
// `spin_until_future_complete` used to live in `nros/rclcpp_compat.hpp`, a
// separate header a ported file had to be force-included with. RFC-0089
// §"Naming: replace, with alias as the migration step" makes the ROS 2 spelling
// a FIRST-CLASS name declared by the API headers themselves; §"End state: no
// compat layer survives" is what follows from it — with one spelling per
// concept there is nothing left for a shim to bridge, so it dissolves by
// construction rather than having to be argued away. `nros::` is untouched and
// both spellings work; deprecating `nros::` and migrating the in-tree call
// sites is step B.
//
// They land HERE, in the umbrella, and not in `node.hpp`, because that is where
// their dependencies are. `rclcpp::Node` reaches `nros::create_subscription_raw`
// (`component.hpp`), `nros::Timer`, `nros::ParameterServer`, `nros::Service` and
// `nros::Client`, every one of which includes `node.hpp` itself; `Rate::sleep`
// and `spin_until_future_complete` drive the `nros::spin` / `nros::spin_once`
// free functions defined a few lines above. This IS the header they were
// shimming: the process-level verbs `rclcpp::init` mirrors — `nros::init`,
// `nros::spin`, `nros::spin_once`, `nros::on_shutdown` — are all reached
// through `<nros/nros.hpp>`.
//
// FREESTANDING. Unlike the shim it replaces, this is not hosted-STL by
// construction. `init` / `shutdown` / `ok` and the `--ros-args` predicate need
// nothing but `<cstdlib>`, so they reach a `no_std` C++ build. `Node`, the spin
// forwarders and `Rate` are gated on the standard-library pieces their
// signatures are spelled in (`__has_include`, never `__STDC_HOSTED__` — issue
// 0112, rationale in `publisher.hpp`), so on a minimal libcpp they are absent
// rather than a parse error.

/// Capacity of `rclcpp::Node`'s node-local parameter store. Inline storage, so
/// this is RAM in every image that constructs one — override it per build
/// rather than raising the default.
#ifndef NROS_RCLCPP_MAX_PARAMS
#define NROS_RCLCPP_MAX_PARAMS 16
#endif

#include <cstdlib>     // std::abort -- the runtime half of RFC-0089 W3.b
#include <type_traits> // the SFINAE guards on create_service / create_client

#if defined(NROS_CPP_STD)
#include <chrono>
#define NROS_CPP_HAS_STD_CHRONO 1
#elif defined(__has_include)
#if __has_include(<chrono>)
#include <chrono>
#define NROS_CPP_HAS_STD_CHRONO 1
#endif
#endif

namespace rclcpp {

// --- Process-level lifecycle -------------------------------------------------
//
// `rclcpp::init()` is a process-level handshake → `nros::init()`.
// `rclcpp::shutdown()` → `nros::shutdown()`. `rclcpp::ok()` → `nros::ok()`
// (nros tracks the shutdown flag).
//
// The TWO-ARGUMENT form is REFUSE-LOUD (RFC-0089 W3.b). It used to forward to
// the same `nros::init()` and discard `argv`, so `--ros-args -r
// chatter:=/other` — the single most common thing a ported `main` passes —
// silently became a wrong-topic bug at runtime. Honouring it is remap
// resolution, RFC-0020 violation class 4, and belongs beside
// `nros::resolve_name` rather than here; until it lands, the honest answer is a
// loud abort carrying the migration.

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
/// So the refusal fires where the information is. RFC-0089's rule is that a
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

} // namespace rclcpp

// The node call-shape adapter and the verbs that take one. `std::shared_ptr` is
// in `rclcpp::Node`'s every signature — `std::make_shared<rclcpp::Node>(…)` is
// how a ported file constructs it and `create_publisher` hands one back — so
// where `<memory>` / `<string>` / `<vector>` / `<functional>` are absent the
// type is absent with them.
#if defined(NROS_CPP_HAS_SHARED_PTR) && defined(NROS_CPP_HAS_STD_STRING) &&                        \
    defined(NROS_CPP_HAS_STD_VECTOR) && defined(NROS_CPP_HAS_STD_FUNCTION)

namespace rclcpp {

namespace detail {

#if defined(NROS_SYSTEM_PARAM_SERVICES)
// --- launch-seed adoption (issue 0745) --------------------------------------
//
// The generated entry seeds `nros_cpp_declare_param` into the EXECUTOR's store
// before any user code runs. `rclcpp::Node`'s store is a different object, so
// without reading the seed back a `declare_parameter("period", 0.15)` would
// return 0.15 while launch said 0.03 — a silently dropped configuration, which
// is exactly what RFC-0089's rule forbids compiling.
//
// Compiled only when the bringup declares `param_services`, which is also what
// links `nros_cpp_get_param_*`. Where it is absent there is no executor store,
// so there is no seed to drop.
//
// KNOWN DUPLICATION, flagged rather than hidden: this is a C++14 restatement of
// `ComponentNode::adopt_launch_seed_` (`component_node.hpp:536-565`), which
// spells the same dispatch with C++17 `if constexpr`. There should be ONE
// helper; hoisting it is tracked with phase-417 W2.a (issue 0793, "one
// parameter store").

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

// --- Node --------------------------------------------------------------------
//
// rclcpp users write `auto n = std::make_shared<rclcpp::Node>("name");` and
// then `n->create_publisher<M>(topic, qos)` returning a shared_ptr. nros's
// Node is created via an `Executor` and exposes out-ref `create_*` member
// functions. This matches the rclcpp call shape onto it.
//
// A `rclcpp::Node` does NOT own an executor. It opens its `nros::Node` on the
// GLOBAL one `rclcpp::init()` created (issue 0465 — one session per image), and
// since phase-417 every entity it creates — publisher, subscription, wall
// timer, service, client — is registered THERE. So this class holds no dispatch
// state and runs no loop: it is a call-shape adapter over `nros::Node`, which
// is all RFC-0019 permits a wrapper to be, and it is the property that lets a
// later step merge the two node types safely.
//
// Threading: callbacks fire on whatever thread services a spin verb, mirroring
// the rclcpp default. An `nros::Node` cannot move between executors (RFC-0002,
// one executor per RTOS task), which is why the executor is decided at
// construction and never afterwards.

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
    // these are one-line forwarders, which is all RFC-0089 §"Who implements an
    // adopted name" permits the wrapper to be. An uninitialized node answers
    // `""` / the node's own default-constructed clock, matching what
    // `nros::Node` does — this adds no behaviour of its own.

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
        // every Node its OWN executor and therefore its own RMW session.
        // A non-bridge application has exactly one session; two is the bridge
        // shape. With `ZPICO_MAX_SESSIONS` at its default of 1 the second open
        // exhausted the pool and returned -1, surfacing as
        // `Transport(ConnectionFailed)` — the same text a real connection
        // failure gives, which is why it read as one for two months. Raising
        // the pool would have hidden the design error behind memory spent on
        // every embedded target.
        //
        // `::nros::create_node` targets `Node::global_storage()`, so N
        // `rclcpp::Node`s share the single session, which is also what rclcpp's
        // own process-level Context model implies.
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
    // `nros::Node::create_subscription` makes, so the executor owns the
    // subscriber and dispatches the callback during `spin_once` — whichever
    // spin verb the caller drives. It used to create a POLL-mode subscription
    // and drain it from a node-local `pump()`, which only `rclcpp::spin` /
    // `spin_some` called: a file that spun `nros::spin_once()` instead got zero
    // callbacks and no diagnostic (RFC-0089 §"There is also one mismatch the
    // rename makes strictly worse").
    //
    // Why not `node_.create_subscription(*s, topic, cb, qos)`: that overload is
    // SFINAE-restricted to `void(*)(const M&)` — a plain function pointer with
    // NO context slot — and every ported rclcpp callback captures. The typed
    // ctx-carrying delivery exists one layer down (`Subscription<M>::
    // user_fn_ctx_` + `user_ctx_`, and the `message_trampoline` branch that
    // reads them, `subscription.hpp:563`) but NO `create_*` sets those fields,
    // so the raw register with a type-erased ctx is the reachable shape. A
    // `create_subscription` overload taking `(void(*)(const M&, void*), void*
    // ctx)` would let this call `nros::Node` instead, and is C++-side work.
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
    // `""` so no existing caller moves.
    //
    // OWNERSHIP: the arena stores the cell's address as its dispatch context
    // and there is no unregister, so the cell must outlive the registration.
    // `owned_entities_` holds it for the node's lifetime — the same rule
    // `create_service` below states — and the returned `shared_ptr` is a
    // co-owner via the aliasing constructor, so dropping it is safe.
    //
    // WHAT THE RETURNED POINTER IS: a keep-alive, which is all upstream source
    // does with it (`rclcpp::Subscription<M>::SharedPtr sub_;`). The executor
    // owns the real subscriber, so `sub->take(msg)` on it answers
    // `NotInitialized` — the sample went to your callback.
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

#ifdef NROS_CPP_HAS_STD_CHRONO
    /// `create_wall_timer(period, callback)` — fires `callback()` every
    /// `period`, dispatched by the EXECUTOR during `spin_once`, i.e. under
    /// whichever spin verb the caller drives: `rclcpp::spin`, `spin_some`,
    /// `nros::spin_once`, `nros::spin`, `rclcpp::Rate::sleep`, or an
    /// `nros::Executor` driven directly.
    ///
    /// The `std::chrono::duration` → milliseconds conversion is the only work
    /// this function does beyond delegating; that is ergonomics and permitted,
    /// the schedule is not. See the envelope on `rclcpp::TimerBase`
    /// (`timer.hpp`) for the two things it costs (millisecond resolution, and
    /// catch-up rather than rcl's drop-the-backlog on a missed deadline).
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
#endif // NROS_CPP_HAS_STD_CHRONO

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
    // and it cannot be done from here — the executor store exposes four typed
    // GETTERS across the FFI (`nros_cpp_get_param_*`) and no setter and no
    // existence check, so `set_parameter` and `has_parameter` have nothing to
    // call. That is Rust-side work, not wrapper work.
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
    //     `spin_once`, on the executor arena — as every other entity this node
    //     creates now does (phase-417).
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
    // it, so a node driven by any other spin verb dispatched nothing. Every
    // entity a `rclcpp::Node` creates is now registered on the executor arena,
    // so there is nothing left for a node-local sweep to do and mixing spin
    // spellings is harmless. Do not reintroduce one: a second dispatch path
    // here is the RFC-0019 violation the whole item was about, and it cannot be
    // made visible by a diagnostic.

  private:
    ::nros::Node node_;
    NodeOptions node_options_;
    bool initialized_ = false;
#ifdef NROS_CPP_HAS_STD_CHRONO
    // Wall-timer cells. The executor arena dispatches them and holds each
    // cell's address as its callback context, so the node keeps them alive —
    // see `rclcpp::detail::WallTimer` for the destruction-order rule.
    std::vector<std::shared_ptr<detail::WallTimer>> timers_;
#endif
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
// `nros::spin()` (block until `ok()` goes false) and `rclcpp::spin_some(node)`
// is one 0-timeout `nros::spin_once`. Both used to call a node-local `pump()`
// first, which was the ONLY thing that ran a ported node's timers and
// subscriptions; now those live on the executor, so these two dispatch nothing
// of their own and a file that reaches for `nros::spin_once()` or drives an
// `nros::Executor` gets exactly the same callbacks. That equivalence is the
// structural prerequisite for step B: after it both node types share a name,
// and a mismatch would be invisible.
//
// One behaviour change for an existing caller: `nros::spin()` RETURNS on the
// first failing `spin_once`, where this loop used to discard the result and
// keep going. A dead session now ends the spin instead of looking alive
// forever, which is what `nros::spin()` and `Executor::spin` already promise.
//
// The `node` argument is still checked, and still otherwise unused: there is
// one session per image (issue 0465), so every `rclcpp::Node` is on the global
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

} // namespace rclcpp

#endif // NROS_CPP_HAS_SHARED_PTR && ...

// --- Rate / WallRate (phase-417 W2.d) ----------------------------------------
//
// A FORWARDER onto `nros::spin(remaining_ms, poll_ms)`, which budgets by wall
// clock inside `nros_cpp_spin_for` — Rust-side, once.
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
//     Since phase-417 that includes a `rclcpp::Node`'s OWN timers and
//     subscriptions: they are executor entities now, so `rate.sleep()` runs
//     them.
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
//
// Gated on `<chrono>` alone: the duration constructor and `period()` are spelled
// in `std::chrono`, and everything else it touches is an integer.
#ifdef NROS_CPP_HAS_STD_CHRONO

namespace rclcpp {

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
            // — upstream's `Rate` contract, and the choice the retired
            // node-local `pump()` used to make for wall timers before they
            // became executor timers (which catch up instead; see the envelope
            // on `rclcpp::TimerBase`).
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

} // namespace rclcpp

#endif // NROS_CPP_HAS_STD_CHRONO

#endif // NROS_CPP_HPP
