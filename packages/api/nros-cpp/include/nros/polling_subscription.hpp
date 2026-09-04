// nros-cpp: latest-value polling subscription
// Freestanding C++ — no exceptions, no STL required

/**
 * @file polling_subscription.hpp
 * @ingroup grp_pubsub
 * @brief `nros::PollingSubscription<M>` — pull-based, latest-value subscriber.
 *
 * Issue 0278 — the nano-ros analog of ROS 2
 * `autoware_utils::InterProcessPollingSubscriber`: a poll-mode
 * `Subscription<M>` plus a RETAINED last value. Where the bare
 * `Subscription<M>` is consuming (each `take` yields a sample once, then
 * `TryAgain`), this wrapper keeps the newest value so a caller can read it
 * repeatably at a chosen point — replacing the hand-rolled "callback caches
 * latest + has_ flag" pattern (see `examples/templates/topic-state-monitor-port`).
 *
 * Every accessor first DRAINS to the newest pending sample (a burst collapses
 * to its last element, matching upstream `takeData`), then answers from the
 * cache. No executor, no callback — the caller owns polling.
 */

#ifndef NROS_CPP_POLLING_SUBSCRIPTION_HPP
#define NROS_CPP_POLLING_SUBSCRIPTION_HPP

#include <cstddef>

#include "nros/node.hpp"
#include "nros/result.hpp"
#include "nros/subscription.hpp"

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

namespace nros {

/// Latest-value polling subscription.
///
/// Usage:
/// ```cpp
/// nros::PollingSubscription<std_msgs::msg::Int32> sub;
/// NROS_TRY(node.create_polling_subscription(sub, "/count"));
/// // ... later, in a timer tick or the main loop:
/// nros::spin_once(executor, 0);           // pump the transport
/// if (const auto* v = sub.take_data()) {  // newest known value, or nullptr
///     use(v->data);
/// }
/// ```
template <typename M> class PollingSubscription {
  public:
#ifdef NROS_CPP_HAS_SHARED_PTR
    /// `PollingSubscription<M>::SharedPtr` — phase-417 W1.a.
    ///
    /// The nested-pointer spelling rclcpp uses for every entity type, carried
    /// here so a member declaration reads the same as its
    /// `Subscription<M>::SharedPtr` sibling. There is no upstream
    /// `PollingSubscription`; the analog is `autoware_utils::
    /// InterProcessPollingSubscriber` (issue 0278).
    ///
    /// Ergonomics only (RFC-0089 §"Who implements an adopted name"): a
    /// spelling for `std::shared_ptr<PollingSubscription<M>>`, no second code
    /// path.
    ///
    /// Present only where `<memory>` is — a freestanding target has no
    /// `std::shared_ptr` to alias.
    using SharedPtr = std::shared_ptr<PollingSubscription<M>>;
    /// `PollingSubscription<M>::ConstSharedPtr` — see `SharedPtr`.
    using ConstSharedPtr = std::shared_ptr<const PollingSubscription<M>>;
    /// `PollingSubscription<M>::UniquePtr` — see `SharedPtr`.
    using UniquePtr = std::unique_ptr<PollingSubscription<M>>;
#endif

    PollingSubscription() : sub_(), latest_(), has_ever_(false) {}

    PollingSubscription(const PollingSubscription&) = delete;
    PollingSubscription& operator=(const PollingSubscription&) = delete;

    /// True once the underlying subscription is created.
    bool is_valid() const { return sub_.is_valid(); }

    /// True once at least one sample has ever been received.
    bool has_data() const { return has_ever_; }

    /// Drain to the newest pending sample, then return a pointer to the
    /// retained latest value — repeatably, whether or not a new sample arrived
    /// this call. Returns `nullptr` only if NOTHING has ever been received
    /// (mirrors `InterProcessPollingSubscriber::takeData`).
    const M* take_data() {
        drain();
        return has_ever_ ? &latest_ : nullptr;
    }

    /// Drain to the newest pending sample; return a pointer to it ONLY if a new
    /// sample actually arrived this call, else `nullptr` (mirrors
    /// `takeNewData` — use when "did it change?" matters).
    const M* take_new_data() { return drain() ? &latest_ : nullptr; }

    // phase-379 W6 — `take(M&)` was REMOVED here, deliberately, and the name is
    // now reserved for rclcpp's meaning.
    //
    // It drained to the newest sample and returned `true` if a value had EVER
    // been received, cached or new. `rclcpp::Subscription::take` is CONSUMING:
    // "true if data was taken and is valid". So the idiomatic drain loop
    //
    //     while (sub.take(msg)) { process(msg); }
    //
    // terminated under rclcpp and spun forever on one stale sample here --
    // same name, same signature, opposite contract, no compile error.
    //
    // It was a convenience duplicating `take_data()`, with zero real callers.
    // `take_data()` (retained latest) and `take_new_data()` (only if new) are
    // the faithful `autoware_utils` mirrors (issue 0278) and are unaffected;
    // use `take_data()` for what this did.

    /// Direct read of the cached latest without draining (no transport poll).
    /// `nullptr` until the first value arrives. Pair with an explicit
    /// `take_data()`/`take_new_data()` elsewhere if you only want to poll once.
    const M* peek() const { return has_ever_ ? &latest_ : nullptr; }

  private:
    friend class Node;

    /// Consume every pending sample, keeping the newest in `latest_`. Returns
    /// `true` iff at least one new sample was taken this call. `take`
    /// writes `latest_` only on success, so a trailing `TryAgain` leaves the
    /// previous latest intact.
    bool drain() {
        bool got = false;
        while (sub_.take(latest_).ok()) {
            has_ever_ = true;
            got = true;
        }
        return got;
    }

    Subscription<M> sub_;
    M latest_;
    bool has_ever_;
};

} // namespace nros

#include "nros/node.hpp"

namespace nros {

template <typename M>
Result Node::create_polling_subscription(PollingSubscription<M>& out, const char* topic,
                                         const QoS& qos) {
    // Reuse the existing poll-mode subscription factory to own storage/init/
    // destroy; the wrapper only adds the retained-latest cache (issue 0278).
    return create_subscription(out.sub_, topic, qos);
}

} // namespace nros

#endif // NROS_CPP_POLLING_SUBSCRIPTION_HPP
