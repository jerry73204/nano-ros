// nros-cpp: QoS (Quality of Service) profiles
// Freestanding C++ — no STL required

/**
 * @file qos.hpp
 * @ingroup grp_qos
 * @brief `nros::QoS` — full DDS-shaped QoS settings (Phase 108.B.7).
 */

#ifndef NROS_CPP_QOS_HPP
#define NROS_CPP_QOS_HPP

#include <stdint.h>

// Phase 379 W5 — the deadline / lifespan / lease accessors take and return
// `nros::Duration`, so this header names it. `nros.hpp` includes `qos.hpp`
// before `duration.hpp`, so the dependency is spelled HERE rather than left to
// the umbrella's ordering: a header that names a type must be able to be
// included first.
#include "nros/duration.hpp"
// phase-417 stage 6 step A — `rclcpp::detail::refuse` and the
// `NROS_RCLCPP_REFUSE_*` diagnostics, used by `SystemDefaultsQoS` below.
// `log.hpp` includes nothing of ours, so this adds no cycle.
#include "nros/log.hpp"
#include <type_traits> // rclcpp::detail::is_qos_arg

// FFI struct definition — mirrors `nros_cpp_qos_t` in
// nros_cpp_ffi.h. Phase 118.D: guarded by `NROS_CPP_FFI_H`. If
// `nros_cpp_ffi.h` was included earlier (it sets that guard), the
// canonical types are already in scope; otherwise emit local
// definitions so this header stays self-contained for callers that
// don't pull the cbindgen header directly.
//
// The FIELD names below are ABI. `deadline_ms` / `lifespan_ms` /
// `liveliness_lease_ms` are `uint32_t` milliseconds and are mirrored again in
// `nros/component.h` (issue 0160, gated by `check-ffi-struct-mirrors`). The
// `_ms` suffix on a `QoS` METHOD was renamed in phase-379 W5; the suffix on a
// STRUCT FIELD was not, and must not be — the C ABI carries milliseconds.
#ifndef NROS_CPP_FFI_H
extern "C" {
enum nros_cpp_qos_reliability_t {
    NROS_CPP_QOS_RELIABLE = 0,
    NROS_CPP_QOS_BEST_EFFORT = 1,
};
enum nros_cpp_qos_durability_t {
    NROS_CPP_QOS_VOLATILE = 0,
    NROS_CPP_QOS_TRANSIENT_LOCAL = 1,
};
enum nros_cpp_qos_history_t {
    NROS_CPP_QOS_KEEP_LAST = 0,
    NROS_CPP_QOS_KEEP_ALL = 1,
};
enum nros_cpp_qos_liveliness_t {
    NROS_CPP_QOS_LIVELINESS_NONE = 0,
    NROS_CPP_QOS_LIVELINESS_AUTOMATIC = 1,
    NROS_CPP_QOS_LIVELINESS_MANUAL_BY_TOPIC = 2,
    NROS_CPP_QOS_LIVELINESS_MANUAL_BY_NODE = 3,
};
struct nros_cpp_qos_t {
    enum nros_cpp_qos_reliability_t reliability;
    enum nros_cpp_qos_durability_t durability;
    enum nros_cpp_qos_history_t history;
    enum nros_cpp_qos_liveliness_t liveliness_kind;
    int depth;
    uint32_t deadline_ms;
    uint32_t lifespan_ms;
    uint32_t liveliness_lease_ms;
    uint8_t avoid_ros_namespace_conventions;
    uint8_t tx_express;
};
}
#endif // NROS_CPP_FFI_H

namespace nros {

// -- Policy enums (phase 379 W5) ------------------------------------------
//
// At NAMESPACE scope and public, under rclcpp's names, because that is where a
// ported node looks for them (`rclcpp::ReliabilityPolicy`, `qos.hpp:34-58`).
// Three of these were PRIVATE members of `QoS` until phase-379 W5, which is why
// the getters had to hand back `int` — a public getter cannot name a private
// type. Ledger: `cpp:ReliabilityPolicy` and its three siblings.
//
// Two deliberate differences from rclcpp survive the rename, both ledgered:
//   * no `SystemDefault` — it means "defer to the middleware" and there is
//     none to defer to; the backend is linked at build time (RFC-0036).
//   * no `Unknown` — it is a discovery artefact for a policy read off a remote
//     endpoint, and we do no dynamic discovery.
//
// These are UNSCOPED enums, not rclcpp's `enum class`. Both spellings work as a
// result: `nros::Reliable` (ours, historical) and `nros::ReliabilityPolicy::
// Reliable` (rclcpp's). Switching to `enum class` would break the first and is
// its own decision, which no ledger row makes — see the qos.json rows.

/// Reliability policy. Matches DDS `RELIABILITY_QOS_POLICY`.
enum ReliabilityPolicy {
    Reliable = 0,
    BestEffort = 1,
};

/// Durability policy. Matches DDS `DURABILITY_QOS_POLICY`.
enum DurabilityPolicy {
    Volatile = 0,
    TransientLocal = 1,
};

/// History policy. Matches DDS `HISTORY_QOS_POLICY`.
enum HistoryPolicy {
    KeepLast = 0,
    KeepAll = 1,
};

/// Liveliness policy kind. Matches DDS `LIVELINESS_QOS_POLICY`.
///
/// The enumerators keep their `Liveliness` prefix because these are UNSCOPED
/// enums at namespace scope: a bare `Automatic` / `None` in `nros::` would be
/// far worse names than the redundancy costs. `nros::LivelinessPolicy::
/// LivelinessAutomatic` also works, so a qualified rclcpp-shaped spelling
/// compiles. `LivelinessManualByNode` is ours-only — rmw deprecated it, so
/// rclcpp's `LivelinessPolicy` does not list it.
enum LivelinessPolicy {
    LivelinessNone = 0,
    LivelinessAutomatic = 1,
    LivelinessManualByTopic = 2,
    LivelinessManualByNode = 3,
};

class QoS;

namespace detail {

/// Milliseconds one QoS window is worth, at the `Duration` → C-ABI boundary.
///
/// The C ABI is `uint32_t` milliseconds (`nros_cpp_qos_t.deadline_ms` and
/// siblings) and `0` there means INFINITE, so truncation is not a rounding
/// question — a 500 µs deadline truncated to `0` would silently mean "no
/// deadline at all". So:
///
///   * `<= 0` → `0`. A non-positive window is the "unset" spelling
///     (`Duration()` / `Duration::zero()`), and `0` is the ABI's infinity.
///   * otherwise ROUND UP to the next whole millisecond. A sub-millisecond
///     window becomes 1 ms rather than infinity, and rounding up is the
///     lenient direction for all three policies (a longer deadline, lifespan
///     or lease never turns a working profile into a violated one).
///   * above `UINT32_MAX` ms (~49.7 days) → `UINT32_MAX`, saturating rather
///     than wrapping into a short window.
constexpr uint32_t qos_window_ms(const Duration& d) {
    return d.nanoseconds() <= 0
               ? 0u
               : (d.nanoseconds() > static_cast<int64_t>(UINT32_MAX) * 1000000
                      ? UINT32_MAX
                      : static_cast<uint32_t>((d.nanoseconds() + 999999) / 1000000));
}

/// The inverse: the C ABI's milliseconds as a `Duration`. Exact, and exact in
/// both directions — `qos_window_ms(qos_window_duration(ms)) == ms`.
constexpr Duration qos_window_duration(uint32_t ms) {
    return Duration::from_nanoseconds(static_cast<int64_t>(ms) * 1000000);
}

} // namespace detail

/// QoS profile for publishers and subscriptions.
///
/// Mirrors rclcpp::QoS with chainable setters and predefined profiles.
/// All methods are constexpr — profiles can be computed at compile time.
///
/// Phase 108.B.7 — extended with full DDS QoS surface (deadline,
/// lifespan, liveliness kind + lease, avoid_ros_namespace_conventions).
/// Backends advertise per-policy support; entities created with a
/// profile the active backend can't honour return
/// `NROS_CPP_RET_INCOMPATIBLE_QOS` synchronously at create time.
///
/// Phase 379 W5 — the policy enums moved to namespace scope, the getters
/// return them instead of `int`, and the three time windows take and return
/// `nros::Duration` instead of carrying an `_ms` suffix. Every old spelling
/// still compiles and is `[[deprecated]]`.
class QoS {
  public:
    /// Deprecated spelling of `nros::LivelinessPolicy`.
    using Liveliness [[deprecated("QoS::Liveliness is deprecated; use nros::LivelinessPolicy")]] =
        LivelinessPolicy;

    // The four liveliness enumerators were reachable as `QoS::Liveliness*`
    // while the enum was a member. They still are, deprecated, so no source
    // that named one stops compiling. The enumerator SPELLING did not change —
    // only its scope — so `nros::LivelinessAutomatic` is the live name.
    [[deprecated("QoS::LivelinessNone is deprecated; use "
                 "nros::LivelinessNone")]] static constexpr LivelinessPolicy LivelinessNone =
        ::nros::LivelinessNone;
    [[deprecated(
        "QoS::LivelinessAutomatic is deprecated; use "
        "nros::LivelinessAutomatic")]] static constexpr LivelinessPolicy LivelinessAutomatic =
        ::nros::LivelinessAutomatic;
    [[deprecated("QoS::LivelinessManualByTopic is deprecated; use "
                 "nros::LivelinessManualByTopic")]] static constexpr LivelinessPolicy
        LivelinessManualByTopic = ::nros::LivelinessManualByTopic;
    [[deprecated(
        "QoS::LivelinessManualByNode is deprecated; use "
        "nros::LivelinessManualByNode")]] static constexpr LivelinessPolicy LivelinessManualByNode =
        ::nros::LivelinessManualByNode;

    /// Default QoS: reliable, volatile, keep-last(10), automatic
    /// liveliness, no deadline / lifespan / lease.
    constexpr QoS()
        : reliability_(Reliable), durability_(Volatile), history_(KeepLast),
          liveliness_(::nros::LivelinessAutomatic), depth_(10), deadline_ms_(0), lifespan_ms_(0),
          liveliness_lease_ms_(0), avoid_ros_namespace_conventions_(0), tx_express_(0) {}

    /// rclcpp-shape depth ctor: `QoS(1)` == default profile with
    /// keep-last(1). Lets ported rclcpp code (`rclcpp::QoS{1}.transient_local()`)
    /// keep its spelling on the native surface.
    explicit constexpr QoS(int depth) : QoS() { depth_ = depth; }

    // -- Chainable setters (match rclcpp fluent API) --

    /// Set reliability to `RELIABLE` (acked transport, retransmits on loss).
    constexpr QoS& reliable() {
        reliability_ = Reliable;
        return *this;
    }

    /// Set reliability to `BEST_EFFORT` (fire-and-forget; default for sensors).
    constexpr QoS& best_effort() {
        reliability_ = BestEffort;
        return *this;
    }

    /// Set durability to `TRANSIENT_LOCAL` — late joiners get the last value.
    constexpr QoS& transient_local() {
        durability_ = TransientLocal;
        return *this;
    }

    /// Set durability to `VOLATILE` — late joiners get nothing (default).
    constexpr QoS& durability_volatile() {
        durability_ = Volatile;
        return *this;
    }

    /// Use `KEEP_LAST` history with the given depth.
    /// @param depth maximum number of messages buffered per entity.
    constexpr QoS& keep_last(int depth) {
        history_ = KeepLast;
        depth_ = depth;
        return *this;
    }

    /// Use `KEEP_ALL` history (bounded by transport).
    constexpr QoS& keep_all() {
        history_ = KeepAll;
        return *this;
    }

    /// Subscriber max-inter-arrival / publisher offered-rate.
    /// A zero or negative duration = infinite (no deadline check).
    ///
    /// The C ABI underneath is `uint32_t` milliseconds, so a sub-millisecond
    /// window ROUNDS UP to 1 ms — it never truncates to `0`, which the ABI
    /// reads as infinity. Above ~49.7 days it saturates. See
    /// `detail::qos_window_ms`.
    constexpr QoS& deadline(const Duration& d) {
        deadline_ms_ = detail::qos_window_ms(d);
        return *this;
    }

    /// Sample expiry. A zero or negative duration = infinite.
    /// Same millisecond boundary as `deadline()`.
    constexpr QoS& lifespan(const Duration& d) {
        lifespan_ms_ = detail::qos_window_ms(d);
        return *this;
    }

    /// Liveliness kind. Pair with `liveliness_lease_duration()` for
    /// `MANUAL_BY_TOPIC` / `MANUAL_BY_NODE`. AUTOMATIC is the default.
    constexpr QoS& liveliness(LivelinessPolicy kind) {
        liveliness_ = kind;
        return *this;
    }

    /// Liveliness lease duration. A zero or negative duration = infinite.
    /// Same millisecond boundary as `deadline()`.
    constexpr QoS& liveliness_lease_duration(const Duration& d) {
        liveliness_lease_ms_ = detail::qos_window_ms(d);
        return *this;
    }

    /// Skip the ROS `/rt/` topic-name prefix. Off by default; enable
    /// when interoperating with non-ROS DDS endpoints.
    constexpr QoS& avoid_ros_namespace_conventions(bool on) {
        avoid_ros_namespace_conventions_ = on ? 1 : 0;
        return *this;
    }

    /// Phase 282 (#145) — mark this publisher "express": its samples bypass
    /// transport tx batching (sent immediately even when the batching knob
    /// is on). A transport hint for control-tier / latency-sensitive topics;
    /// ignored on subscriptions and by backends without batching.
    constexpr QoS& tx_express(bool on) {
        tx_express_ = on ? 1 : 0;
        return *this;
    }

    // -- Deprecated millisecond setters (phase 379 W5) --

    /// @deprecated Use `deadline(nros::Duration)`.
    [[deprecated("QoS::deadline_ms(uint32_t) is deprecated; use "
                 "QoS::deadline(nros::Duration)")]] constexpr QoS&
    deadline_ms(uint32_t ms) {
        deadline_ms_ = ms;
        return *this;
    }

    /// @deprecated Use `lifespan(nros::Duration)`.
    [[deprecated("QoS::lifespan_ms(uint32_t) is deprecated; use "
                 "QoS::lifespan(nros::Duration)")]] constexpr QoS&
    lifespan_ms(uint32_t ms) {
        lifespan_ms_ = ms;
        return *this;
    }

    /// @deprecated Use `liveliness_lease_duration(nros::Duration)`.
    [[deprecated("QoS::liveliness_lease_ms(uint32_t) is deprecated; use "
                 "QoS::liveliness_lease_duration(nros::Duration)")]] constexpr QoS&
    liveliness_lease_ms(uint32_t ms) {
        liveliness_lease_ms_ = ms;
        return *this;
    }

    // -- Predefined profiles (match rclcpp named constructors) --

    /// Default profile: `RELIABLE` + `VOLATILE` + `KEEP_LAST(10)`.
    static constexpr QoS default_profile() { return QoS(); }

    /// Sensor-data profile: `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST(5)`.
    static constexpr QoS sensor_data() { return QoS().best_effort().keep_last(5); }

    /// Services profile: `RELIABLE` + `VOLATILE` + `KEEP_LAST(10)`.
    static constexpr QoS services() { return QoS().reliable(); }

    // -- Accessors --

    /// The reliability policy.
    constexpr ReliabilityPolicy reliability() const { return reliability_; }
    /// The durability policy.
    constexpr DurabilityPolicy durability() const { return durability_; }
    /// The history policy.
    constexpr HistoryPolicy history() const { return history_; }
    /// The liveliness policy kind.
    constexpr LivelinessPolicy liveliness() const { return liveliness_; }
    /// Configured queue depth (only meaningful for `KEEP_LAST`).
    constexpr int depth() const { return depth_; }
    /// The deadline window. `Duration()` (zero) = infinite.
    constexpr Duration deadline() const { return detail::qos_window_duration(deadline_ms_); }
    /// The lifespan window. `Duration()` (zero) = infinite.
    constexpr Duration lifespan() const { return detail::qos_window_duration(lifespan_ms_); }
    /// The liveliness lease window. `Duration()` (zero) = infinite.
    constexpr Duration liveliness_lease_duration() const {
        return detail::qos_window_duration(liveliness_lease_ms_);
    }
    /// Whether to skip the `/rt/` ROS topic-name prefix.
    constexpr bool avoid_ros_namespace_conventions() const {
        return avoid_ros_namespace_conventions_ != 0;
    }
    /// Whether this publisher's samples bypass transport tx batching.
    constexpr bool tx_express() const { return tx_express_ != 0; }

    // -- Deprecated accessors (phase 379 W5) --
    //
    // The `_raw()` four existed only because their enums were private; the
    // `_ms()` three only because C++ had no `Duration`. Both reasons are gone.

    /// @deprecated Use `reliability()`, which returns `ReliabilityPolicy`.
    [[deprecated("QoS::reliability_raw() is deprecated; use QoS::reliability()")]] constexpr int
    reliability_raw() const {
        return static_cast<int>(reliability_);
    }
    /// @deprecated Use `durability()`, which returns `DurabilityPolicy`.
    [[deprecated("QoS::durability_raw() is deprecated; use QoS::durability()")]] constexpr int
    durability_raw() const {
        return static_cast<int>(durability_);
    }
    /// @deprecated Use `history()`, which returns `HistoryPolicy`.
    [[deprecated("QoS::history_raw() is deprecated; use QoS::history()")]] constexpr int
    history_raw() const {
        return static_cast<int>(history_);
    }
    /// @deprecated Use `liveliness()`, which returns `LivelinessPolicy`.
    [[deprecated("QoS::liveliness_raw() is deprecated; use QoS::liveliness()")]] constexpr int
    liveliness_raw() const {
        return static_cast<int>(liveliness_);
    }
    /// @deprecated Use `deadline()`, which returns `nros::Duration`.
    [[deprecated("QoS::deadline_ms() is deprecated; use QoS::deadline()")]] constexpr uint32_t
    deadline_ms() const {
        return deadline_ms_;
    }
    /// @deprecated Use `lifespan()`, which returns `nros::Duration`.
    [[deprecated("QoS::lifespan_ms() is deprecated; use QoS::lifespan()")]] constexpr uint32_t
    lifespan_ms() const {
        return lifespan_ms_;
    }
    /// @deprecated Use `liveliness_lease_duration()`, which returns `nros::Duration`.
    [[deprecated("QoS::liveliness_lease_ms() is deprecated; use "
                 "QoS::liveliness_lease_duration()")]] constexpr uint32_t
    liveliness_lease_ms() const {
        return liveliness_lease_ms_;
    }

  private:
    // The private members keep their `_ms_` spelling: they hold exactly what
    // the C ABI carries (`uint32_t` milliseconds), and renaming them would say
    // the storage changed when only the accessors did.
    ReliabilityPolicy reliability_;
    DurabilityPolicy durability_;
    HistoryPolicy history_;
    LivelinessPolicy liveliness_;
    int depth_;
    uint32_t deadline_ms_;
    uint32_t lifespan_ms_;
    uint32_t liveliness_lease_ms_;
    uint8_t avoid_ros_namespace_conventions_;
    uint8_t tx_express_;
};

namespace detail {

/// `QoS` → the by-value C ABI record, in ONE place.
///
/// Ten call sites across `publisher.hpp` / `subscription.hpp` / `service.hpp` /
/// `client.hpp` / `action_{server,client}.hpp` / `component.hpp` open-coded
/// this ten-field copy. A field appended to `nros_cpp_qos_t` then has ten
/// places to be missed, which is the hand-mirror class issue 0160 records;
/// phase-379 W5 folded them into this one function while renaming the
/// accessors they called.
/// `constexpr` so a fixed profile's marshalled form is checkable at compile
/// time (`qos_policy_accessors.cpp` static_asserts the millisecond fields);
/// value-initialised because C++14 forbids an uninitialised local in a
/// `constexpr` function, and zeroing ten fields we then overwrite costs
/// nothing.
constexpr nros_cpp_qos_t qos_to_ffi(const QoS& qos) {
    nros_cpp_qos_t f{};
    f.reliability = static_cast<nros_cpp_qos_reliability_t>(qos.reliability());
    f.durability = static_cast<nros_cpp_qos_durability_t>(qos.durability());
    f.history = static_cast<nros_cpp_qos_history_t>(qos.history());
    f.liveliness_kind = static_cast<nros_cpp_qos_liveliness_t>(qos.liveliness());
    f.depth = qos.depth();
    // The FFI FIELDS are milliseconds and keep their `_ms` names; only the
    // accessors changed. `qos_window_ms` is the exact inverse of the
    // `Duration` the getter built, so this round-trip loses nothing.
    f.deadline_ms = qos_window_ms(qos.deadline());
    f.lifespan_ms = qos_window_ms(qos.lifespan());
    f.liveliness_lease_ms = qos_window_ms(qos.liveliness_lease_duration());
    f.avoid_ros_namespace_conventions = qos.avoid_ros_namespace_conventions() ? 1 : 0;
    f.tx_express = qos.tx_express() ? 1 : 0;
    return f;
}

} // namespace detail

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling of the QoS surface (RFC-0089 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer exists as a
// separate surface: RFC-0089 §"Naming: replace, with alias as the migration
// step" makes the ROS 2 spelling a first-class name declared by the API header
// that owns the concept. `nros::QoS` is unchanged and still the name every
// in-tree caller writes; step B deprecates it.
//
// Everything below is freestanding-safe — `constexpr` classes over `nros::QoS`
// and one `<type_traits>` predicate — so a `no_std` C++ build gets the ROS 2
// QoS vocabulary too, which the hosted-STL shim could never offer it.

namespace rclcpp {

// rclcpp::QoS subclasses nros::QoS to add the `QoS(depth)` integer ctor every
// ported source uses; the chainable setters (`reliable()`, `best_effort()`,
// `keep_last(n)`, …) are inherited. Implicit-converts to `nros::QoS` (used in
// the create_publisher/subscription overloads on `rclcpp::Node`).
class QoS : public ::nros::QoS {
  public:
    constexpr QoS() = default;
    // NOLINTNEXTLINE(google-explicit-constructor)
    constexpr QoS(::size_t depth) : ::nros::QoS() { keep_last(static_cast<int>(depth)); }
    // NOLINTNEXTLINE(google-explicit-constructor)
    constexpr QoS(const ::nros::QoS& other) : ::nros::QoS(other) {}
};

namespace detail {

/// Is this argument a QoS profile rather than a callback?
///
/// The refusing `create_service(name, F, qos)` overload on `rclcpp::Node` must
/// not swallow `create_service<S>("name", rclcpp::ServicesQoS())`. Deducing `F`
/// is an exact match while binding `const nros::QoS&` needs a derived-to-base
/// conversion, so without this guard the callback template WINS and a perfectly
/// good poll-style call fails with the shared_ptr-callback diagnostic — a
/// refusal firing on something it does not describe, which is worse than no
/// refusal. Caught by the positive probe on its first compile.
template <typename F>
struct is_qos_arg : ::std::is_base_of<::nros::QoS, typename ::std::decay<F>::type> {};

} // namespace detail

// --- Named QoS profiles (RFC-0089 W3.f) --------------------------------------
//
// TRANSCRIBED from upstream field by field, not approximated. Two of these
// shipped WRONG, which is the evidence for why the transcription is pinned by a
// test rather than by review:
//
//   * `ParametersQoS()` returned `QoS(10)` where `rmw_qos_profile_parameters`
//     is KEEP_LAST/**1000** — a hundredfold history difference under a name
//     that claims to be the ROS 2 profile, costing samples under load with
//     nothing to read.
//   * `SystemDefaultsQoS()` returned `QoS(10)`, which is
//     `rmw_qos_profile_default` — a DIFFERENT upstream profile. It is now
//     REFUSE-LOUD; see the class below for why no value can be right.
//
// Sources, read 2026-09-04 against ROS 2 Humble as installed:
//   /opt/ros/humble/include/rmw/rmw/qos_profiles.h    :25,38,51,64,77,90
//   /opt/ros/humble/include/rcl/rcl/logging_rosout.h  :37   (rosout)
//   /opt/ros/humble/include/rclcpp/rclcpp/qos.hpp     :351-489
//
// These are CLASSES, as upstream's are, so `rclcpp::SensorDataQoS{}` — the
// brace form — compiles alongside `rclcpp::SensorDataQoS()`. They are
// `constexpr`, so the table-driven check lives in `static_assert`s
// (`tests/compile/ros2_api_adoption_stage2.cpp`) rather than in a runtime test
// no embedded lane runs.
//
// Three fields are the same in EVERY upstream profile above and are therefore
// not spelled per profile:
//   * `avoid_ros_namespace_conventions` is `false` upstream, `0` in a default
//     `nros::QoS`.
//   * liveliness is `RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT` upstream;
//     `nros::QoS` defaults to `LivelinessAutomatic`, which is what every
//     reference RMW folds that sentinel to.
//   * deadline and the liveliness lease are `RMW_QOS_*_DEFAULT` (infinite)
//     upstream and `0` here, which `nros::detail::qos_window_ms` documents as
//     infinite.

/// `rmw_qos_profile_sensor_data` — KEEP_LAST(5), BEST_EFFORT, VOLATILE. ADOPT.
class SensorDataQoS : public QoS {
  public:
    constexpr SensorDataQoS() : QoS(::nros::QoS().best_effort().keep_last(5)) {}
};

/// `rmw_qos_profile_services_default` — KEEP_LAST(10), RELIABLE, VOLATILE.
/// ADOPT. (Verified rather than assumed: this one was already right, but it
/// was right by way of `QoS(10)`, which states the depth and leaves the
/// reliability to the default. It now states both.)
class ServicesQoS : public QoS {
  public:
    constexpr ServicesQoS() : QoS(::nros::QoS().reliable().keep_last(10)) {}
};

/// `rmw_qos_profile_parameters` — KEEP_LAST(**1000**), RELIABLE, VOLATILE.
/// ADOPT, at the corrected depth. This is one of RFC-0089's two live
/// inversions: it returned `QoS(10)`.
class ParametersQoS : public QoS {
  public:
    constexpr ParametersQoS() : QoS(::nros::QoS().reliable().keep_last(1000)) {}
};

/// `rmw_qos_profile_parameter_events` — KEEP_LAST(1000), RELIABLE, VOLATILE.
/// ADOPT. A ported node that publishes parameter events names it.
class ParameterEventsQoS : public QoS {
  public:
    constexpr ParameterEventsQoS() : QoS(::nros::QoS().reliable().keep_last(1000)) {}
};

/// `rcl_qos_profile_rosout_default` — KEEP_LAST(1000), RELIABLE,
/// TRANSIENT_LOCAL, lifespan 10 s. ADOPT.
///
/// ADOPT-BOUNDED on one point, and it is about the TOPIC, not the profile: the
/// profile's four policies are transcribed exactly, but nano-ros publishes no
/// `/rosout` topic (logging goes to `nros_log`'s per-platform sink), so this
/// names a profile you can apply to a topic of your own rather than one the
/// runtime is already using.
class RosoutQoS : public QoS {
  public:
    constexpr RosoutQoS()
        : QoS(::nros::QoS().reliable().transient_local().keep_last(1000).lifespan(
              ::nros::Duration::from_nanoseconds(10000000000LL))) {}
};

/// `rclcpp::ClockQoS` — KEEP_LAST(1), BEST_EFFORT, VOLATILE. ADOPT.
///
/// Upstream builds it from `rmw_qos_profile_sensor_data` with `KeepLast(1)`
/// (`rclcpp/qos.hpp:351-357`), which is why the depth differs from
/// `SensorDataQoS`'s 5 and the reliability does not.
class ClockQoS : public QoS {
  public:
    constexpr ClockQoS() : QoS(::nros::QoS().best_effort().keep_last(1)) {}
};

/// `rclcpp::SystemDefaultsQoS` — **REFUSE-LOUD**.
///
/// Every field of `rmw_qos_profile_system_default` is a sentinel meaning "let
/// the RMW decide", and issue 0829 measured the two reference RMWs resolving
/// the depth sentinel to different numbers (Cyclone 1, zenoh 42). `nros::QoS`
/// has no sentinel, deliberately — the backend is linked at build time, so
/// there is nothing to defer to. Any concrete value here would be a different
/// profile wearing this name, which is precisely what the old `QoS(10)` was.
class SystemDefaultsQoS : public QoS {
  public:
    template <typename T = void> SystemDefaultsQoS() {
        static_assert(detail::refuse<T>::value, NROS_RCLCPP_REFUSE_SYSTEM_DEFAULTS_QOS);
    }
};

/// `rclcpp::KeepLast(depth)`.
///
/// ADOPT-BOUNDED: upstream returns a `QoSInitialization` — a history/depth pair
/// a `QoS` is then built from — and this returns a whole `QoS` carrying the
/// default reliability and durability. The porting spellings that matter,
/// `rclcpp::QoS(rclcpp::KeepLast(10))` and passing it straight to
/// `create_publisher`, both resolve to the same profile either way. What does
/// NOT carry over is using it as an initialiser for a profile whose other
/// policies you meant to keep — `rclcpp::SensorDataQoS(rclcpp::KeepLast(1))`
/// has no equivalent here; write `nros::QoS().best_effort().keep_last(1)`.
constexpr QoS KeepLast(::size_t depth) {
    return QoS(depth);
}

/// `rclcpp::KeepAll()`. Same ADOPT-BOUNDED note as `KeepLast`.
constexpr QoS KeepAll() {
    return QoS(::nros::QoS().keep_all());
}

} // namespace rclcpp

#endif // NROS_CPP_QOS_HPP
