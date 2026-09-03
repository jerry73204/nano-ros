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
//   - `rclcpp::spin(node)` / `rclcpp::spin_some(node)` (`spin_some` ≈ a single
//     `nros::Executor::spin_once`; `spin` loops until `rclcpp::shutdown()`).
//   - `rclcpp::QoS(depth)`, `rclcpp::SystemDefaultsQoS{}`.
//   - `rclcpp::NodeOptions{}` and `Node(name, options)` constructor shapes
//     used by `rclcpp_components` classes.
//   - Log macros `RCLCPP_INFO/WARN/ERROR/DEBUG/FATAL` (`_THROTTLE` variants
//     degrade to plain log — no real throttle yet).
//   - `rclcpp_action::Server<A>`/`Client<A>` aliases over nros action shapes.
//
// Out of scope (will need source adapt or follow-up shims):
//   - Full `rclcpp::NodeOptions` parameter/remap/allocator semantics. The
//     compatibility type stores common scalar toggles so source compiles, but
//     nano-ros launch/codegen owns runtime projection for now.
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

#include "nros/nros.hpp"
#include "nros/node.hpp"
#include "nros/publisher.hpp"
#include "nros/subscription.hpp"
#include "nros/service.hpp"
#include "nros/client.hpp"
#include "nros/executor.hpp"
#include "nros/qos.hpp"
#include "nros/log.hpp"
#include "nros/action_server.hpp"
#include "nros/action_client.hpp"
#include "nros/result.hpp"

#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <chrono>

namespace rclcpp {

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
    QoS(::size_t depth) : ::nros::QoS() { keep_last(static_cast<int>(depth)); }
    QoS(const ::nros::QoS& other) : ::nros::QoS(other) {}
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

// --- QoS conveniences --------------------------------------------------------
//
// `rclcpp::QoS(10)` is the common spelling; nros::QoS takes a depth argument
// too, so the constructor is already compatible. These named profiles save the
// most-cited spellings from `rclcpp::QoS` factories.

inline QoS SystemDefaultsQoS() {
    return QoS(10);
}
inline QoS ServicesQoS() {
    return QoS(10);
}
inline QoS ParametersQoS() {
    return QoS(10);
}

inline QoS KeepLast(::size_t depth) {
    return QoS(depth);
}

// --- NodeOptions -------------------------------------------------------------
//
// Minimal source-compat holder for upstream composable nodes. The fields are
// intentionally inert today: nano-ros projects launch parameters/remaps during
// codegen/runtime setup, while this type keeps common `Node("name", options)`
// and builder-style option chains compile-compatible.

class NodeOptions {
  public:
    NodeOptions() = default;

    NodeOptions& arguments(const std::vector<std::string>& args) {
        arguments_ = args;
        return *this;
    }
    const std::vector<std::string>& arguments() const { return arguments_; }

    NodeOptions& use_global_arguments(bool value) {
        use_global_arguments_ = value;
        return *this;
    }
    bool use_global_arguments() const { return use_global_arguments_; }

    NodeOptions& enable_rosout(bool value) {
        enable_rosout_ = value;
        return *this;
    }
    bool enable_rosout() const { return enable_rosout_; }

    NodeOptions& start_parameter_services(bool value) {
        start_parameter_services_ = value;
        return *this;
    }
    bool start_parameter_services() const { return start_parameter_services_; }

    NodeOptions& start_parameter_event_publisher(bool value) {
        start_parameter_event_publisher_ = value;
        return *this;
    }
    bool start_parameter_event_publisher() const { return start_parameter_event_publisher_; }

    NodeOptions& allow_undeclared_parameters(bool value) {
        allow_undeclared_parameters_ = value;
        return *this;
    }
    bool allow_undeclared_parameters() const { return allow_undeclared_parameters_; }

    NodeOptions& automatically_declare_parameters_from_overrides(bool value) {
        automatically_declare_parameters_from_overrides_ = value;
        return *this;
    }
    bool automatically_declare_parameters_from_overrides() const {
        return automatically_declare_parameters_from_overrides_;
    }

    NodeOptions& use_intra_process_comms(bool value) {
        use_intra_process_comms_ = value;
        return *this;
    }
    bool use_intra_process_comms() const { return use_intra_process_comms_; }

    NodeOptions& enable_topic_statistics(bool value) {
        enable_topic_statistics_ = value;
        return *this;
    }
    bool enable_topic_statistics() const { return enable_topic_statistics_; }

    NodeOptions& enable_logger_service(bool value) {
        enable_logger_service_ = value;
        return *this;
    }
    bool enable_logger_service() const { return enable_logger_service_; }

  private:
    std::vector<std::string> arguments_;
    bool use_global_arguments_ = true;
    bool enable_rosout_ = false;
    bool start_parameter_services_ = false;
    bool start_parameter_event_publisher_ = false;
    bool allow_undeclared_parameters_ = false;
    bool automatically_declare_parameters_from_overrides_ = false;
    bool use_intra_process_comms_ = false;
    bool enable_topic_statistics_ = false;
    bool enable_logger_service_ = false;
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
// `rclcpp::init(argc, argv)` is a process-level handshake. nano-ros has a
// global `nros::init()` (no argc/argv — args are ignored). `rclcpp::shutdown()`
// → `nros::shutdown()`. `rclcpp::ok()` → `nros::ok()` (nros tracks the
// shutdown flag).

inline void init(int /*argc*/, char const* const* /*argv*/) {
    (void)::nros::init();
}
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
// functions. Wrap that to match the rclcpp call shape. A `Node` shim owns its
// own `nros::Executor` (the typical single-node-per-process pattern). When
// shared across nodes is needed, the caller can construct multiple Nodes —
// each currently gets its own Executor (matches the single-node default).
//
// Threading: `rclcpp::spin(node)` borrows the node's executor for the calling
// thread (same as `nros::Executor::spin`). Callbacks fire on whatever thread
// services `spin*`, mirroring the rclcpp default.

// --- Timer surface ----------------------------------------------------------
//
// rclcpp users typically store a `rclcpp::TimerBase::SharedPtr` and only care
// that it stays alive as long as the timer should fire. The actual dispatch
// happens through `Node::create_wall_timer(period, callback)`. Implementation:
// the compat tracks `WallTimer`s on the Node and fires them from `Node::pump()`
// — same polling model as the subscription pump. Sufficient for typical ROS 2
// periodic callbacks; the wall-clock granularity is whatever `rclcpp::spin*`'s
// caller drives.
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
class WallTimer : public TimerBase {
  public:
    std::chrono::steady_clock::duration period{};
    std::chrono::steady_clock::time_point next_fire{};
    std::function<void()> callback;
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
    // plain fn ptr) — nros's native callback-subscription overload is SFINAE-
    // restricted to `void(*)(const M&)` plain fn ptrs (no capture). Wrap a
    // polling Subscription + a pump callback the node's spin loop invokes per
    // sweep (`Node::pump()`, called by `rclcpp::spin`/`spin_some`). The
    // callable is heap-stored (shared_ptr) so its lifetime matches the
    // subscription; cleanup is automatic when the subscription drops out of
    // scope. (Native callback-arena path is a future optimization — direct
    // FFI hook with std::function as user_data — when per-spin polling
    // overhead matters; for the source-compat target this is fine.)
    template <typename M, typename Cb>
    std::shared_ptr<::nros::Subscription<M>> create_subscription(const std::string& topic,
                                                                 const ::nros::QoS& qos, Cb cb) {
        auto s = std::make_shared<::nros::Subscription<M>>();
        (void)node_.create_subscription(*s, topic.c_str(), qos);
        auto cb_fn = std::make_shared<std::function<void(const M&)>>(std::move(cb));
        pump_callbacks_.push_back([s, cb_fn]() {
            M msg;
            while (s->take(msg).ok()) {
                (*cb_fn)(msg);
            }
        });
        return s;
    }

    template <typename M, typename Cb>
    std::shared_ptr<::nros::Subscription<M>> create_subscription(const std::string& topic,
                                                                 ::size_t depth, Cb cb) {
        return create_subscription<M>(topic, QoS(static_cast<::size_t>(depth)), std::move(cb));
    }

    // create_wall_timer(period, callback) — fires `callback()` every `period`,
    // driven by `Node::pump()` (i.e. each `rclcpp::spin_some` / `spin` sweep).
    template <typename Rep, typename Period, typename Cb>
    std::shared_ptr<TimerBase> create_wall_timer(std::chrono::duration<Rep, Period> period, Cb cb) {
        auto t = std::make_shared<detail::WallTimer>();
        t->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        t->next_fire = std::chrono::steady_clock::now() + t->period;
        t->callback = std::move(cb);
        timers_.push_back(t);
        return std::static_pointer_cast<TimerBase>(t);
    }

    // Pump all polling subscriptions + due wall-timers. Called by
    // rclcpp::spin / spin_some before invoking the underlying nros
    // executor's spin_once.
    void pump() {
        const auto now = std::chrono::steady_clock::now();
        for (auto& t : timers_) {
            if (!t || !t->callback) continue;
            if (now >= t->next_fire) {
                t->next_fire += t->period;
                // If we've fallen badly behind, snap to "now" to avoid burst
                // firing — matches rclcpp's WallTimer behaviour.
                if (now > t->next_fire) {
                    t->next_fire = now + t->period;
                }
                t->callback();
            }
        }
        for (auto& f : pump_callbacks_) {
            if (f) f();
        }
    }

  private:
    ::nros::Node node_;
    NodeOptions node_options_;
    bool initialized_ = false;
    // Heap-stored polling pumps for create_subscription callbacks (one per
    // sub). Captured by std::function so any callable shape (capturing lambda,
    // member-fn bind, std::function) works.
    std::vector<std::function<void()>> pump_callbacks_;
    // Wall-timers driven from `pump()` — see `create_wall_timer`.
    std::vector<std::shared_ptr<detail::WallTimer>> timers_;
};

// --- spin / spin_some --------------------------------------------------------
//
// `rclcpp::spin(node)` loops the node's executor until shutdown.
// `rclcpp::spin_some(node)` makes one progress sweep (≈ a 0-timeout spin_once).

inline void spin(const Node::SharedPtr& node) {
    if (!node || !node->initialized()) {
        return;
    }
    while (::nros::ok()) {
        node->pump(); // polling subscription dispatch (capturing-lambda path)
        (void)::nros::spin_once(10);
    }
}

inline void spin_some(const Node::SharedPtr& node) {
    if (!node || !node->initialized()) {
        return;
    }
    node->pump();
    (void)::nros::spin_once(0);
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
// Same call shape as rclcpp; the logger arg is accepted and ignored (the file/
// line/format are what reach the sink). _THROTTLE variants degrade to plain log
// — they get the message out; a follow-up can add interval gating.

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

#define RCLCPP_INFO_STREAM(logger, args) RCLCPP_INFO(logger, "%s", "")
#define RCLCPP_WARN_STREAM(logger, args) RCLCPP_WARN(logger, "%s", "")
#define RCLCPP_ERROR_STREAM(logger, args) RCLCPP_ERROR(logger, "%s", "")

#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...) RCLCPP_INFO(logger, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, period_ms, ...) RCLCPP_WARN(logger, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...) RCLCPP_ERROR(logger, __VA_ARGS__)
#endif // RCLCPP_INFO

#endif // NROS_RCLCPP_COMPAT_HPP
