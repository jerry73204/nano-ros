// nros-cpp: Timer class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file timer.hpp
 * @ingroup grp_executor
 * @brief `nros::Timer` — periodic callback driven by the executor.
 */

#ifndef NROS_CPP_TIMER_HPP
#define NROS_CPP_TIMER_HPP

#include <cstdint>
#include <cstddef>

#include "nros/result.hpp"

#ifdef NROS_CPP_STD
#include <functional>
#include <memory>
#endif

// phase-417 W1.a — `<memory>` for the nested pointer aliases. Rationale (and
// why the test is `__has_include` rather than `__STDC_HOSTED__`, issue 0112)
// lives in `publisher.hpp`.
#if defined(NROS_CPP_STD)
#include <memory>
#define NROS_CPP_HAS_SHARED_PTR 1
#elif defined(__has_include)
#if __has_include(<memory>)
#include <memory>
#define NROS_CPP_HAS_SHARED_PTR 1
#endif
#endif

#include "nros_cpp_ffi.h"

namespace nros {

/// Repeating or one-shot timer registered with the executor.
///
/// Timers fire during `spin_once()` when their period has elapsed.
/// The callback is a C function pointer with a user context.
///
/// Usage:
/// ```cpp
/// void on_timer(void* ctx) { /* periodic work */ }
///
/// nros::Timer timer;
/// NROS_TRY(node.create_wall_timer(timer, 1000, on_timer));  // 1000ms period
/// // timer fires during nros::spin_once()
/// timer.cancel();
/// timer.reset();  // restart from zero
/// ```
class Timer {
  public:
#ifdef NROS_CPP_HAS_SHARED_PTR
    /// `Timer::SharedPtr` — phase-417 W1.a; the analog of
    /// `rclcpp::TimerBase::SharedPtr`, which is how ported source declares a
    /// timer member (`rclcpp::TimerBase::SharedPtr timer_;`). The shim's own
    /// `rclcpp::TimerBase` carries the same three aliases.
    ///
    /// Ergonomics only (RFC-0087 §"Who implements an adopted name"): a
    /// spelling for `std::shared_ptr<Timer>`, no second code path.
    ///
    /// Present only where `<memory>` is — a freestanding target has no
    /// `std::shared_ptr` to alias.
    using SharedPtr = std::shared_ptr<Timer>;
    /// `rclcpp::TimerBase::ConstSharedPtr` — see `SharedPtr`.
    using ConstSharedPtr = std::shared_ptr<const Timer>;
    /// `rclcpp::TimerBase::UniquePtr` — see `SharedPtr`.
    using UniquePtr = std::unique_ptr<Timer>;
#endif

    /// Cancel the timer. It stops firing but remains in the executor.
    /// Use `reset()` to restart it.
    Result cancel() {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_timer_cancel(executor_, handle_id_));
    }

    /// Reset the timer (restart from zero elapsed time).
    /// If cancelled, this also un-cancels it.
    Result reset() {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        return Result(nros_cpp_timer_reset(executor_, handle_id_));
    }

    /// Check if the timer is cancelled.
    bool is_canceled() const {
        if (!initialized_) return true;
        return nros_cpp_timer_is_canceled(executor_, handle_id_);
    }

    /// Check if the timer is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Destructor — cancels the timer.
    ~Timer() {
        if (initialized_) {
            nros_cpp_timer_cancel(executor_, handle_id_);
            initialized_ = false;
        }
        // closure_ (if any) destructs here; the runtime no longer
        // holds a raw pointer to it because we cancelled above.
    }

    // Move semantics (non-copyable)
    Timer(Timer&& other)
        : executor_(other.executor_), handle_id_(other.handle_id_), initialized_(other.initialized_)
#ifdef NROS_CPP_STD
          ,
          closure_(std::move(other.closure_))
#endif
    {
        other.executor_ = nullptr;
        other.initialized_ = false;
    }

    Timer& operator=(Timer&& other) {
        if (this != &other) {
            if (initialized_) {
                nros_cpp_timer_cancel(executor_, handle_id_);
            }
            executor_ = other.executor_;
            handle_id_ = other.handle_id_;
            initialized_ = other.initialized_;
#ifdef NROS_CPP_STD
            closure_ = std::move(other.closure_);
#endif
            other.executor_ = nullptr;
            other.initialized_ = false;
        }
        return *this;
    }

    /// Default constructor — creates an uninitialized timer.
    /// Use `Node::create_wall_timer()` to initialize.
    Timer() : executor_(nullptr), handle_id_(0), initialized_(false) {}

#ifdef NROS_CPP_STD
    /// @internal Attach a heap-allocated std::function closure to this
    /// timer. Called by the `NROS_CPP_STD` convenience wrappers in
    /// `std_compat.hpp` *after* the runtime registered a raw callback
    /// pointing into the same closure. The unique_ptr keeps the closure
    /// alive for the lifetime of the Timer, freeing it automatically on
    /// destruction. Not intended for user code.
    void attach_std_closure(std::unique_ptr<std::function<void()>> closure) {
        closure_ = std::move(closure);
    }
#endif

  private:
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    friend class Node;

    void* executor_;
    size_t handle_id_;
    bool initialized_;

#ifdef NROS_CPP_STD
    /// Owns the heap-allocated `std::function<void()>` closure (if any).
    ///
    /// Only populated when the Timer was created through the
    /// `NROS_CPP_STD` convenience wrapper. Freed automatically when the
    /// Timer is destroyed or moved-from.
    std::unique_ptr<std::function<void()>> closure_;
#endif
};

} // namespace nros

// ============================================================================
// rclcpp::TimerBase (RFC-0087 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`. rclcpp users typically store a
// `rclcpp::TimerBase::SharedPtr` and only care that it stays alive as long as
// the timer should fire. The dispatch happens through
// `rclcpp::Node::create_wall_timer(period, callback)` (`nros/nros.hpp`).
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
// structural fix RFC-0087 makes a prerequisite for the stage-6 rename. Do not
// reintroduce a second dispatch loop.
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
//     whole missed periods and re-phases onto the grid, firing once. Closing
//     the gap means a missed-deadline POLICY on the executor's timer —
//     Rust-side work, not a loop re-added here (issue 1041).

// `<functional>` for the type-erased callback cell. Gated for the same reason
// `<memory>` is above — issue 0112, rationale in `publisher.hpp`.
#if defined(NROS_CPP_STD)
#include <functional>
#define NROS_CPP_HAS_STD_FUNCTION 1
#elif defined(__has_include)
#if __has_include(<functional>)
#include <functional>
#define NROS_CPP_HAS_STD_FUNCTION 1
#endif
#endif

#ifdef NROS_CPP_HAS_SHARED_PTR
namespace rclcpp {

/// `rclcpp::TimerBase` — what `rclcpp::TimerBase::SharedPtr timer_;` names.
///
/// Present only where `<memory>` is: the three nested aliases ARE the reason
/// upstream source declares this type, and a freestanding target has no
/// `std::shared_ptr` to alias.
class TimerBase {
  public:
    /// phase-417 W1.a — `rclcpp::TimerBase::SharedPtr timer_;` is how upstream
    /// source declares a timer member.
    using SharedPtr = std::shared_ptr<TimerBase>;
    using ConstSharedPtr = std::shared_ptr<const TimerBase>;
    using UniquePtr = std::unique_ptr<TimerBase>;

    virtual ~TimerBase() = default;
};

#ifdef NROS_CPP_HAS_STD_FUNCTION
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
/// unregisters it, so the cell has to outlive the registration.
/// `rclcpp::Node::timers_` holds a `shared_ptr` for the node's lifetime, and
/// the MEMBER ORDER below is load-bearing — members destruct in reverse
/// declaration order, so `timer` goes first and `~nros::Timer` cancels the
/// arena slot (`timer.hpp:70`) before `callback` is destroyed. Declared the
/// other way round, a tick landing between the two destructions would run a
/// destroyed `std::function`.
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

} // namespace detail
#endif // NROS_CPP_HAS_STD_FUNCTION

} // namespace rclcpp
#endif // NROS_CPP_HAS_SHARED_PTR

#endif // NROS_CPP_TIMER_HPP
