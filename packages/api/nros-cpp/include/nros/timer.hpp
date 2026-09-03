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

#endif // NROS_CPP_TIMER_HPP
