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

#endif // NROS_CPP_HPP
