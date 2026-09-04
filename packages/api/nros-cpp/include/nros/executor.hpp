// nros-cpp: Executor class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file executor.hpp
 * @ingroup grp_executor
 * @brief `nros::Executor` — drives transport I/O and dispatches callbacks.
 */

#ifndef NROS_CPP_EXECUTOR_HPP
#define NROS_CPP_EXECUTOR_HPP

#include <cstdint>
#include <cstddef>

#include "nros/result.hpp"
#include "nros/nros_cpp_config_generated.h"

#include "nros_cpp_ffi.h"

namespace nros {

// Forward declarations
class Node;
class NodeBuilder;

/// Signature of a shutdown callback: `void callback(void* context)`.
///
/// rclcpp takes `std::function<void()>`, which is a heap allocation per
/// registration and needs the STL. This is freestanding C++ with no allocator,
/// so the callback is a plain function pointer plus an opaque `context` the
/// callee casts back to its own state — the same shape every other callback in
/// this API uses. Capture what you need in a struct and pass its address.
using ShutdownCallback = void (*)(void* context);

/// Handle to a registered shutdown callback — rclcpp's
/// `rclcpp::ShutdownCallbackHandle`. Issue 0790.
///
/// rclcpp's handle owns a `std::shared_ptr` to the callback. There is no
/// allocator here and the callbacks live in a fixed-capacity static table, so
/// this wraps the SLOT INDEX instead: the smallest thing that names a row of a
/// static array, and one `uint32_t` across the C ABI.
class ShutdownCallbackHandle {
  public:
    /// An empty handle. `valid()` is false; passing it to a `remove` does
    /// nothing and reports false.
    ShutdownCallbackHandle() : value_(NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID) {}

    /// Wrap a raw handle from the C FFI. Rarely needed directly — the
    /// `add_*_shutdown_callback` methods hand back a typed handle.
    explicit ShutdownCallbackHandle(nros_cpp_shutdown_callback_handle_t value) : value_(value) {}

    /// True when this handle came from a successful registration.
    bool valid() const { return value_ != NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID; }

    /// The raw FFI value, for passing to the C entry points directly.
    nros_cpp_shutdown_callback_handle_t value() const { return value_; }

  private:
    nros_cpp_shutdown_callback_handle_t value_;
};

/// Handle to a PRE-shutdown callback — rclcpp's
/// `rclcpp::PreShutdownCallbackHandle`.
///
/// A distinct type from [`OnShutdownCallbackHandle`], as in rclcpp, so the
/// compiler refuses `remove_on_shutdown_callback(a_pre_handle)`. The runtime
/// value carries its phase too, so the same mistake made through the C API
/// fails instead of removing the wrong callback — but a compile error is a
/// better place to learn it.
class PreShutdownCallbackHandle : public ShutdownCallbackHandle {
  public:
    using ShutdownCallbackHandle::ShutdownCallbackHandle;
};

/// Handle to an ON-shutdown callback — rclcpp's
/// `rclcpp::OnShutdownCallbackHandle`. See [`PreShutdownCallbackHandle`].
class OnShutdownCallbackHandle : public ShutdownCallbackHandle {
  public:
    using ShutdownCallbackHandle::ShutdownCallbackHandle;
};

/// Explicit executor for managing ROS 2 entities and spinning.
///
/// Mirrors `rclcpp::executors::SingleThreadedExecutor`. Provides an
/// explicit alternative to the global `nros::init()`/`nros::spin_once()`
/// free functions.
///
/// The executor uses inline opaque storage — no heap allocation required.
///
/// Usage:
/// ```cpp
/// nros::Executor executor;
/// NROS_TRY(nros::Executor::create(executor));
///
/// nros::Node node;
/// NROS_TRY(executor.create_node(node, "my_node"));
///
/// // Create publishers, subscriptions, etc. on node...
///
/// while (executor.ok()) {
///     executor.spin_once(10);
/// }
/// executor.shutdown();
/// ```
class Executor {
  public:
    /// Default constructor — creates an uninitialized executor.
    Executor() : storage_(), initialized_(false) {}

    /// Create and initialize an executor.
    ///
    /// Opens a middleware connection. This is the explicit alternative
    /// to `nros::init()`.
    ///
    /// @param out        Receives the initialized executor.
    /// @param locator    Middleware locator (e.g., "tcp/127.0.0.1:7447"), or nullptr.
    /// @param domain_id  ROS domain ID (0-232).
    /// @return Result indicating success or failure.
    static Result create(Executor& out, const char* locator = nullptr, uint8_t domain_id = 0) {
        return create(out, locator, domain_id, "nros_cpp");
    }

    /// Create and initialize an executor with an explicit session name.
    ///
    /// `session_name` flows through to the XRCE-DDS RMW backend as the
    /// per-process key derivation seed. Two processes sharing one
    /// XRCE Agent MUST use distinct names; see `nros::init`'s named
    /// overload for the full discussion.
    static Result create(Executor& out, const char* locator, uint8_t domain_id,
                         const char* session_name) {
        // -3 = NROS_CPP_RET_INVALID_ARGUMENT (generated header).
        if (session_name == nullptr) {
            return Result(-3);
        }
        // Issue 1050 defect (3) — the same baked RMW selector `nros::init()`
        // reads. Both spellings of "open the one session" must apply it, or the
        // bake means one thing on one path and nothing on the other — which is
        // how `BACKENDS` came to read as a declaration that decided nothing.
#ifdef NROS_ENTRY_RMW
        const char* rmw = NROS_ENTRY_RMW;
#else
        const char* rmw = nullptr;
#endif
        nros_cpp_ret_t ret =
            nros_cpp_init_rmw(rmw, locator, domain_id, session_name, nullptr, out.storage_);
        if (ret == 0) {
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Issue 1050 defect (3) — [`create`](Executor::create) against a named RMW
    /// backend.
    ///
    /// `rmw` is the baked rung of RFC-0045's precedence model A: a hosted
    /// `$NROS_RMW` still wins, and `nullptr` / `""` means "name none", which
    /// resolves only when exactly one backend is registered. See
    /// `nros::init_with_rmw` for why naming one is sometimes the only way to
    /// get the backend the image declared.
    static Result create_with_rmw(Executor& out, const char* rmw, const char* locator = nullptr,
                                  uint8_t domain_id = 0, const char* session_name = "nros_cpp") {
        if (session_name == nullptr) {
            return Result(-3);
        }
        nros_cpp_ret_t ret =
            nros_cpp_init_rmw(rmw, locator, domain_id, session_name, nullptr, out.storage_);
        if (ret == 0) {
            out.initialized_ = true;
        }
        return Result(ret);
    }

    /// Create a node on this executor.
    ///
    /// @param out   Receives the initialized node.
    /// @param name  Node name (null-terminated).
    /// @param ns    Node namespace (null-terminated), or nullptr for "/".
    /// @return Result indicating success or failure.
    Result create_node(Node& out, const char* name, const char* ns = nullptr);

    /// Phase 104.C.9 — chainable Node-creation builder.
    ///
    /// Mirrors Rust's `Executor::node_builder(name).rmw(...).locator(...)
    /// .domain_id(...).namespace(...).sched(...).build()`. Use this when
    /// binding a Node to a specific RMW backend, locator, domain, or
    /// SchedContext. Definition follows the full `NodeBuilder` class in
    /// `node.hpp`.
    NodeBuilder node_builder(const char* name);

    /// Drive transport I/O and dispatch callbacks.
    ///
    /// Processes pending subscriptions, timers, services, and guard conditions.
    /// Call this periodically in your main loop.
    ///
    /// @param timeout_ms  Maximum time to block waiting for I/O.
    /// @return Result indicating success or failure.
    Result spin_once(int32_t timeout_ms = 10) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_spin_once(storage_, timeout_ms));
    }

    /// Phase 124.F.3 — session-level connectivity probe.
    ///
    /// Wire-level round-trip ("is the peer / agent / router
    /// reachable?") with `timeout_ms` budget. Returns
    /// `Result::success()` on reply, `ErrorCode::Timeout` on no
    /// reply, `ErrorCode::Unsupported` when the active backend
    /// can't probe. Mirrors micro-ROS's `rmw_uros_ping_agent`.
    ///
    /// Useful for reconnect-on-link-loss patterns — call
    /// periodically and tear down / re-open the executor on
    /// timeout.
    Result ping(int32_t timeout_ms) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_ping(storage_, timeout_ms));
    }

    /// phase-381 W4 — every node on the graph, with its namespace.
    ///
    /// `visit(name, ns, enclave)` is called once per node; `enclave` is
    /// `nullptr` where the backend tracks none, which is what lets one call
    /// answer both `rmw_get_node_names` forms. Return `false` to stop early.
    ///
    /// Takes a plain function pointer plus `ctx` rather than a `std::function`:
    /// this header is compiled `-nostdinc++` against Zephyr's minimal libcpp in
    /// embedded builds, where `<functional>` does not exist (issue 0112). Every
    /// string is BORROWED for the duration of the call.
    ///
    /// **Reports what has been DISCOVERED and never blocks.** The first call
    /// after startup legitimately sees a partial graph — the backend keeps a
    /// standing query fed by the spin loop — so poll rather than calling once
    /// and concluding. An empty enumeration means "nobody seen yet", never
    /// "nobody exists", and `ErrorCode::Unsupported` (a backend with no graph)
    /// stays distinct from it.
    Result get_node_names(nros_cpp_node_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_node_names(storage_, visit, ctx));
    }

    /// phase-381 W4 — every topic on the graph, with the types on it.
    ///
    /// `visit(name, types, types_count)` is called once per distinct TOPIC: a
    /// topic carrying two types is one call with two entries, not two calls.
    /// `types_count` may legitimately be 0 on a partially discovered graph.
    /// Same discovery caveat as [`get_node_names`].
    Result get_topic_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_topic_names_and_types(storage_, visit, ctx));
    }

    /// phase-381 W4 — every service on the graph, with its types. As
    /// [`get_topic_names_and_types`], over servers and clients.
    Result get_service_names_and_types(nros_cpp_names_and_types_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_service_names_and_types(storage_, visit, ctx));
    }

    /// phase-381 W4 — how many publishers are visible on `topic_name`.
    ///
    /// `topic_name` is a ROS name (`"/chatter"`). The count reflects what has
    /// been DISCOVERED, so it can be low right after startup and is never a
    /// proof of absence.
    Result count_publishers(const char* topic_name, size_t* out_count) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_count_publishers(storage_, topic_name, out_count));
    }

    /// phase-381 W4 — how many subscribers are visible on `topic_name`. See
    /// [`count_publishers`] for the caveats.
    Result count_subscribers(const char* topic_name, size_t* out_count) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_count_subscribers(storage_, topic_name, out_count));
    }

    /// phase-381 W4 — what one named node PUBLISHES, with the types.
    Result get_publisher_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                                 nros_cpp_names_and_types_visit_fn visit,
                                                 void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_publisher_names_and_types_by_node(
            storage_, node_name, node_namespace, visit, ctx));
    }

    /// phase-381 W4 — what one named node SUBSCRIBES to, with the types.
    ///
    /// `subscription`, not `subscriber`: the C++ surface takes rclcpp's
    /// vocabulary (`create_subscription`, `Subscription<T>`,
    /// `get_subscriptions_info_by_topic`). rclcpp has no `*_by_node` form for
    /// subscriptions at all, so the WORD comes from its vocabulary rather than
    /// from a method it lacks. The C surface says `subscriber` because rcl
    /// does, and the vtable slot because upstream rmw does.
    Result get_subscription_names_and_types_by_node(const char* node_name,
                                                    const char* node_namespace,
                                                    nros_cpp_names_and_types_visit_fn visit,
                                                    void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_subscription_names_and_types_by_node(
            storage_, node_name, node_namespace, visit, ctx));
    }

    /// phase-381 W4 — what services one named node SERVES, with the types.
    Result get_service_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                               nros_cpp_names_and_types_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_service_names_and_types_by_node(
            storage_, node_name, node_namespace, visit, ctx));
    }

    /// phase-381 W4 — what services one named node CALLS, with the types.
    Result get_client_names_and_types_by_node(const char* node_name, const char* node_namespace,
                                              nros_cpp_names_and_types_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_get_client_names_and_types_by_node(
            storage_, node_name, node_namespace, visit, ctx));
    }

    /// phase-381 W4 — the publishers on `topic_name`, one visit each.
    ///
    /// The endpoint carries no QoS: the GRANTED profile is what would answer
    /// "why is nothing arriving", no backend can read one back yet, and
    /// reporting the remote's DECLARED profile would be a confident wrong
    /// answer.
    Result get_publishers_info_by_topic(const char* topic_name,
                                        nros_cpp_endpoint_info_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(
            nros_cpp_executor_get_publishers_info_by_topic(storage_, topic_name, visit, ctx));
    }

    /// phase-381 W4 — the subscriptions on `topic_name`, one visit each.
    Result get_subscriptions_info_by_topic(const char* topic_name,
                                           nros_cpp_endpoint_info_visit_fn visit, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(
            nros_cpp_executor_get_subscriptions_info_by_topic(storage_, topic_name, visit, ctx));
    }

    /// Spin until this executor is shut down (blocking) — `rclcpp::Executor::spin`.
    ///
    /// Issue 0338 — this verb used to mean the OPPOSITE here: `spin` was the
    /// BOUNDED form and there was no way to say "spin forever" on an executor,
    /// while `spin` blocks until shutdown in rclcpp, in the C API
    /// (`nros_executor_spin`) and in Rust. A user porting rclcpp code wrote
    /// `exec.spin()` and it did not compile; reaching for `spin(ms)` instead
    /// silently returned early. The bounded form is now [`spin_for`].
    ///
    /// Exit condition: [`cancel`] on THIS executor — typically from a signal
    /// handler or another thread, which is rclcpp's clean-shutdown idiom.
    /// Returns the first non-success `spin_once` result, or success after a
    /// clean cancel.
    ///
    /// phase-417 W4.c — the loop MOVED. It used to be right here, spelled
    /// `while (initialized_)`, which made [`shutdown`] the only way out; and
    /// `shutdown()` calls `nros_cpp_fini`, so a C++ node could not stop spinning
    /// without tearing down the middleware and paying a full discovery round to
    /// come back. Two things were wrong with that: the missing verb, and a
    /// polling loop living in the wrapper (RFC-0020 violation class 2, the same
    /// defect issue 0329 fixed for the BOUNDED form). Both are gone —
    /// `nros_cpp_spin` is the loop, Rust-side, and this is a forwarder.
    ///
    /// A concurrent `shutdown()` is no longer the supported way to end a spin.
    /// It never really was: it frees the executor the loop is polling. Say
    /// `cancel()`, then `shutdown()` once `spin()` has returned.
    ///
    /// @param poll_ms  Individual spin_once timeout (default: 10ms).
    Result spin(int32_t poll_ms = 10) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_spin(storage_, poll_ms));
    }

    /// Ask a running [`spin`] to stop, WITHOUT tearing down the session —
    /// `rclcpp::Executor::cancel`. phase-417 W4.c.
    ///
    /// ADOPT-BOUNDED (RFC-0087): `cancel` sets a flag the spin loop observes at
    /// the NEXT POLL BOUNDARY, so it returns BEFORE spinning has actually
    /// stopped; [`is_spinning`] is the observable that tells you when it has.
    /// The boundary is one `poll_ms` wide (the `spin()` argument), so the delay
    /// is a duration you chose rather than an unknown. rclcpp's `cancel()` makes
    /// the same "returns immediately" promise; naming the width is the part we
    /// add.
    ///
    /// This is the verb a signal handler wants. [`shutdown`] is the other one
    /// and still tears everything down: after `cancel()` the executor is still
    /// `ok()`, still owns its publishers, subscriptions, services and clients,
    /// and can be spun again with no rediscovery. Collapsing the two — which is
    /// what a C++ node had to do before this existed — costs a full discovery
    /// round on every mode change.
    ///
    /// Safe to call from another thread or a signal handler.
    Result cancel() {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_executor_cancel(storage_));
    }

    /// Is a spin loop running on this executor right now? —
    /// `rclcpp::Executor::is_spinning`. phase-417 W4.c.
    ///
    /// A DIFFERENT question from [`ok`], which answers "is this executor
    /// INITIALISED" — an executor that has never spun is `ok()` and not
    /// spinning, and one that has been cancelled mid-poll is both. Before this
    /// existed, `ok()` was the closest thing on offer and it answered neither
    /// half of the cancel envelope.
    ///
    /// This is the second half of [`cancel`]'s contract: poll it to learn when
    /// the cancel has been ACTED on.
    ///
    /// `const` because it is a pure read, and a predicate a caller cannot ask of
    /// a `const Executor&` is a predicate with an arbitrary restriction. The FFI
    /// slot takes `void*` because every OTHER `nros_cpp_executor_*` entry point
    /// mutates through it (see [`handle`]); this one does not, and the
    /// `const_cast` is confined to that mismatch.
    bool is_spinning() const {
        return nros_cpp_executor_is_spinning(const_cast<uint8_t*>(storage_));
    }

    /// Spin for a bounded duration (blocking) — rclcpp's
    /// `spin_some(max_duration)`.
    ///
    /// Repeatedly calls `spin_once()` until `duration_ms` has elapsed.
    ///
    /// @param duration_ms  Total time to spin, in milliseconds.
    /// @param poll_ms      Individual spin_once timeout (default: 10ms).
    /// @return Result from the last spin_once call.
    Result spin_for(uint32_t duration_ms, int32_t poll_ms = 10) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        // Issue 0329 — the wall-clock budgeted loop (Phase 118.C: iteration-count
        // budgeting collapses when `spin_once` returns early on a signaled
        // condvar) now lives once, Rust-side, in `nros_cpp_spin_for`; forward to
        // it so this and `nros::spin()` share one implementation.
        return Result(nros_cpp_spin_for(storage_, duration_ms, poll_ms));
    }

    /// Deprecated alias for [`spin_for`] — issue 0338.
    ///
    /// Kept for one release so existing code compiles. The two-argument form is
    /// unambiguous (the new `spin()` takes at most one argument), so this
    /// overload only ever matches a call that meant the BOUNDED verb.
    [[deprecated("bounded spin is now `spin_for(duration_ms, poll_ms)`; "
                 "`spin()` blocks until shutdown, matching rclcpp (issue 0338)")]] Result
    spin(uint32_t duration_ms, int32_t poll_ms) {
        return spin_for(duration_ms, poll_ms);
    }

    /// Check if the executor is initialized.
    bool ok() const { return initialized_; }

    /// Get the raw executor storage (for advanced use).
    ///
    /// Non-const: downstream FFI mutates executor state through this
    /// pointer (e.g. `spin_once`), so exposing it as `const` would be a
    /// lie. Callers that only need to observe the handle should do so
    /// through methods on `Executor` directly.
    void* handle() { return storage_; }

    /// Register a callback to run BEFORE this executor's entities are torn
    /// down — rclcpp's `Context::add_pre_shutdown_callback`. Issue 0790.
    ///
    /// This is the load-bearing half and the one with no workaround. The
    /// callback runs while every publisher, subscription, service and client
    /// still works, so a node can publish a final state, answer a last request,
    /// park an actuator or release a bus. After teardown it cannot: on a device
    /// there is no OS to reclaim a claimed SPI bus, an armed DMA channel or an
    /// actuator holding its last commanded position.
    ///
    /// The callbacks run from [`shutdown`] (and from the destructor, which
    /// shuts down if still active). They are a CLEAN-STOP facility: a watchdog
    /// reset, a hard fault or an abort does not come through here, and nothing
    /// backed by a static table could promise otherwise.
    ///
    /// @param callback  Function to invoke. Must not be null.
    /// @param context   Opaque pointer handed back to `callback`. Must stay
    ///                  valid until the callback runs or is removed.
    /// @param out       Receives the handle `remove_pre_shutdown_callback`
    ///                  takes. Optional.
    /// @return Success, `ErrorCode::Full` when the fixed table is exhausted
    ///         (`NROS_EXECUTOR_MAX_SHUTDOWN_CBS`, default 2), or
    ///         `ErrorCode::NotInitialized`.
    Result add_pre_shutdown_callback(ShutdownCallback callback, void* context = nullptr,
                                     PreShutdownCallbackHandle* out = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        nros_cpp_shutdown_callback_handle_t raw = NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID;
        nros_cpp_ret_t ret = nros_cpp_add_pre_shutdown_callback(storage_, callback, context, &raw);
        if (out != nullptr) *out = PreShutdownCallbackHandle(raw);
        return Result(ret);
    }

    /// Register a callback to run AFTER this executor's entities are torn down
    /// — rclcpp's `Context::add_on_shutdown_callback` / `rclcpp::on_shutdown`.
    ///
    /// The entities are gone by the time it runs, so anything that needs the
    /// wire belongs in [`add_pre_shutdown_callback`] instead. Use this for what
    /// survives the middleware: releasing a GPIO, stopping a peripheral clock,
    /// writing a log line.
    ///
    /// @see add_pre_shutdown_callback for the parameter and error contract.
    Result add_on_shutdown_callback(ShutdownCallback callback, void* context = nullptr,
                                    OnShutdownCallbackHandle* out = nullptr) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        nros_cpp_shutdown_callback_handle_t raw = NROS_CPP_SHUTDOWN_CALLBACK_HANDLE_INVALID;
        nros_cpp_ret_t ret = nros_cpp_add_on_shutdown_callback(storage_, callback, context, &raw);
        if (out != nullptr) *out = OnShutdownCallbackHandle(raw);
        return Result(ret);
    }

    /// Remove a registered pre-shutdown callback — rclcpp's
    /// `Context::remove_pre_shutdown_callback`.
    ///
    /// `bool` for rclcpp's reason: "it was not there" is an ordinary answer
    /// (the callback may already have run), not an error.
    ///
    /// @return true when `handle` named a live callback.
    bool remove_pre_shutdown_callback(PreShutdownCallbackHandle handle) {
        if (!initialized_) return false;
        return nros_cpp_remove_pre_shutdown_callback(storage_, handle.value()) == 0;
    }

    /// Remove a registered on-shutdown callback — rclcpp's
    /// `Context::remove_on_shutdown_callback`.
    /// @see remove_pre_shutdown_callback
    bool remove_on_shutdown_callback(OnShutdownCallbackHandle handle) {
        if (!initialized_) return false;
        return nros_cpp_remove_on_shutdown_callback(storage_, handle.value()) == 0;
    }

    /// Shut down the executor and close the middleware connection.
    Result shutdown() {
        if (!initialized_) return Result::success();
        nros_cpp_ret_t ret = nros_cpp_fini(storage_);
        initialized_ = false;
        return Result(ret);
    }

    /// Destructor — shuts down if still active.
    ~Executor() {
        if (initialized_) {
            nros_cpp_fini(storage_);
            initialized_ = false;
        }
    }

    // Move semantics (non-copyable)
    Executor(Executor&& other) : initialized_(other.initialized_) {
        for (unsigned i = 0; i < sizeof(storage_); ++i) {
            storage_[i] = other.storage_[i];
            other.storage_[i] = 0;
        }
        other.initialized_ = false;
    }

    Executor& operator=(Executor&& other) {
        if (this != &other) {
            if (initialized_) {
                nros_cpp_fini(storage_);
            }
            for (unsigned i = 0; i < sizeof(storage_); ++i) {
                storage_[i] = other.storage_[i];
                other.storage_[i] = 0;
            }
            initialized_ = other.initialized_;
            other.initialized_ = false;
        }
        return *this;
    }

  private:
    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;

    alignas(8) uint8_t storage_[NROS_CPP_EXECUTOR_STORAGE_SIZE];
    bool initialized_;
};

} // namespace nros

#endif // NROS_CPP_EXECUTOR_HPP
