// nros-cpp: ask a message type how big it can get (freestanding C++14)
//
// phase-408 W1/W4, issue 0896 — one spelling, `nros::rx_size_bound<M>::value`,
// for "how many bytes must a receive buffer for M hold". Every C++ subscribe
// path fills `rx_buffer_hint` from it, so the number reaching the executor
// arena and the zenoh payload class is the type's OWN derived bound and not a
// figure the caller typed.
//
// issue 0964 — a SECOND spelling, `nros::rx_buffer_capacity<M>::value`, for the
// ~13 places inside these headers that stack an actual `uint8_t buf[N]` on the
// stack and receive into it. It answers the same question wherever a derived
// bound exists, and falls back to the legacy `SERIALIZED_SIZE_MAX` ESTIMATE
// where one does not, because turning those sites into compile errors for an
// unbounded type is a behaviour change with a blast radius across every C++
// consumer. Both traits select their arm with ONE predicate, `detail::shape_of`
// below — there is no second way to ask whether a type states a derived bound.

/**
 * @file size_bound.hpp
 * @ingroup grp_support
 * @brief `nros::rx_size_bound<M>` / `nros::tx_size_bound<M>` — a message
 *        type's derived serialized-size bound, or a compile error saying why
 *        it has none; `nros::rx_buffer_capacity<M>` /
 *        `nros::tx_buffer_capacity<M>` — the same number where it exists, the
 *        legacy estimate where it does not.
 */

#ifndef NROS_CPP_SIZE_BOUND_HPP
#define NROS_CPP_SIZE_BOUND_HPP

#include <cstddef>

namespace nros {
namespace detail {

/// A `false` that DEPENDS on a template parameter.
///
/// `static_assert(false, ...)` inside a template fires when the template is
/// PARSED; `static_assert(size_bound_dependent_false<T>::value, ...)` fires
/// when it is INSTANTIATED. Generated message headers use the second form for
/// the poisoned `tx_size_bound`/`rx_size_bound` of an unbounded type, so
/// including such a header stays fine and only ASKING it for a number is the
/// error.
template <class...> struct size_bound_dependent_false {
    static constexpr bool value = false;
};

template <class...> struct make_void {
    using type = void;
};
/// `std::void_t` is C++17. nano-ros DECLARES C++17 since issue 1118, but
/// this header is still written in the freestanding C++14 subset that
/// `just check cpp` parses it at, so it carries its own.
template <class... Ts> using void_t = typename make_void<Ts...>::type;

/// The three shapes a message type can have, and the ONE question every trait
/// in this header asks. Both `strict_bounds` and `buffer_bounds` below
/// specialize on this value, so they cannot disagree about which arm a type is
/// in — the sizes-header mirror class (0088 → 0114 → 0122 → 0123 → 0245 →
/// 0268) is exactly what two independent detections would reintroduce.
enum class bound_shape {
    /// No `nros_derived_size_bounds` member: a type from a codegen with no
    /// derivation at all, or a hand-written message-shaped struct (the
    /// `tests/compile/*.cpp` stubs are these). All it has is the older
    /// `SERIALIZED_SIZE_MAX` ESTIMATE.
    legacy,
    /// Marked, and states `TX_MAX_SERIALIZED_SIZE` / `RX_MAX_SERIALIZED_SIZE`.
    derived,
    /// Marked, and states NEITHER constant: the type has no bound, so
    /// `packs/cpp/message.hpp.jinja` emitted the poison templates instead.
    unbounded,
};

template <class M, class = void> struct has_bound_marker {
    static constexpr bool value = false;
};
template <class M> struct has_bound_marker<M, void_t<typename M::nros_derived_size_bounds>> {
    static constexpr bool value = true;
};

/// The two constants and the two `*_size_bound` templates come out of ONE
/// `{% if tx_max_serialized_size %}` arm in `packs/cpp/message.hpp.jinja`, so
/// "states the constants" and "states a real bound rather than the poison" are
/// the same fact, read here from the half that can be probed without
/// instantiating anything. A `static_assert` failure is a hard error, never
/// SFINAE, so the poison templates themselves cannot be the probe.
template <class M, class = void> struct states_bound_constants {
    static constexpr bool value = false;
};
template <class M> struct states_bound_constants<M, void_t<decltype(M::RX_MAX_SERIALIZED_SIZE)>> {
    static constexpr bool value = true;
};

template <class M> constexpr bound_shape shape_of() {
    return !has_bound_marker<M>::value
               ? bound_shape::legacy
               : (states_bound_constants<M>::value ? bound_shape::derived : bound_shape::unbounded);
}

/// STRICT: the DERIVED bound, or a compile error naming the member that costs
/// this type its bound. Backs `nros::tx_size_bound` / `nros::rx_size_bound`.
template <class M, bound_shape = shape_of<M>()> struct strict_bounds;

template <class M> struct strict_bounds<M, bound_shape::legacy> {
    static constexpr size_t tx = M::SERIALIZED_SIZE_MAX;
    static constexpr size_t rx = M::SERIALIZED_SIZE_MAX;
};
template <class M> struct strict_bounds<M, bound_shape::derived> {
    static constexpr size_t tx = M::TX_MAX_SERIALIZED_SIZE;
    static constexpr size_t rx = M::RX_MAX_SERIALIZED_SIZE;
};
template <class M> struct strict_bounds<M, bound_shape::unbounded> {
    // The poison. Instantiating THIS arm is the deliberate error.
    static constexpr size_t tx = M::template tx_size_bound<>::value;
    static constexpr size_t rx = M::template rx_size_bound<>::value;
};

/// CAPACITY: the number these headers actually stack a `uint8_t buf[N]` on.
/// Same answer as `strict_bounds` for a bounded type, and — since issue 0964
/// was decided — the same poison for a MARKED type with no bound. It still
/// differs from `strict_bounds` in one arm: a `legacy` type, which predates the
/// bound marker entirely and never had a derivation to disagree with, keeps the
/// estimate so an existing consumer keeps compiling.
///
/// The `unbounded` arm WAS issue 0964's open half. Decided 2026-09-05: it
/// poisons. The reasoning is on the arm itself; in short, a buffer sized from an
/// invented number cannot accept a message the user legitimately sends, and
/// that is a fact about the build rather than something a run can recover from.
/// The migration is a `cap` in `nros-codegen.toml` (or a bound in the `.msg`),
/// which fixes every site for that type, or the `_sized` form at one site.
template <class M, bound_shape = shape_of<M>()> struct buffer_bounds;

template <class M> struct buffer_bounds<M, bound_shape::legacy> {
    static constexpr size_t tx = M::SERIALIZED_SIZE_MAX;
    static constexpr size_t rx = M::SERIALIZED_SIZE_MAX;
};
template <class M> struct buffer_bounds<M, bound_shape::derived> {
    static constexpr size_t tx = M::TX_MAX_SERIALIZED_SIZE;
    static constexpr size_t rx = M::RX_MAX_SERIALIZED_SIZE;
};
template <class M> struct buffer_bounds<M, bound_shape::unbounded> {
    // Issue 0964, DECIDED 2026-09-05: this arm poisons.
    //
    // The estimate is not a smaller-than-ideal bound, it is a WRONG one: this
    // type's derivation ran and found no bound at all, so any fixed number
    // here is invented, and `compute_serialized_size_max` is not even trying to
    // be an upper bound (a flat 512 per nested message, a default capacity per
    // string). It over-stated 38 of 39 bounded types and under-stated 1.
    //
    // Under-stating is the case that decides it. A buffer smaller than the
    // message a user legitimately sends cannot accept that message, and the
    // program cannot be made correct at run time — the number is baked into a
    // stack array. That is a fact about the build, so it is a build error.
    //
    // Two ways out, both stating a number somebody chose: bound the type
    // (`string<=64` in the `.msg`, or a `cap` in `nros-codegen.toml`), which
    // fixes every site at once; or call the `_sized` form at the one site that
    // needs it. `heap` / `view` caps do NOT bound (RFC-0033).
    static constexpr size_t tx = strict_bounds<M>::tx;
    static constexpr size_t rx = strict_bounds<M>::rx;
};

} // namespace detail

/// Does `M` state a DERIVED serialized-size bound?
///
/// True only for the `derived` arm — a type whose codegen computed a bound and
/// found one. False both for a legacy type (no derivation) and for a marked
/// type with no bound; those are different facts, and
/// `detail::shape_of<M>()` is where they are told apart.
template <class M> struct has_derived_size_bound {
    static constexpr bool value = detail::shape_of<M>() == detail::bound_shape::derived;
};

/// Bytes a TRANSMIT buffer for `M` must hold — XCDR1, the only encoding this
/// stack writes.
template <class M> struct tx_size_bound {
    static constexpr size_t value = detail::strict_bounds<M>::tx;
};

/// Bytes a RECEIVE buffer for `M` must hold — `max(XCDR1, XCDR2)`, because
/// `data_representation` is negotiable and a non-default peer can send either
/// (RFC-0055). Under-sizing this drops samples at the transport with no
/// diagnostic, which is the failure issue 0896 exists to remove; over-sizing
/// only wastes bytes.
template <class M> struct rx_size_bound {
    static constexpr size_t value = detail::strict_bounds<M>::rx;
};

/// Bytes the in-tree transmit buffers stack for `M`: `tx_size_bound<M>` where
/// a derived bound exists, the legacy `SERIALIZED_SIZE_MAX` estimate where it
/// does not. Prefer `tx_size_bound<M>` in new code — this exists so a type
/// with no bound keeps compiling (issue 0964).
template <class M> struct tx_buffer_capacity {
    static constexpr size_t value = detail::buffer_bounds<M>::tx;
};

/// Bytes the in-tree RECEIVE buffers stack for `M`: `rx_size_bound<M>` where a
/// derived bound exists, the legacy `SERIALIZED_SIZE_MAX` estimate where it
/// does not.
///
/// For the 39 of 120 stock Humble types that HAVE a bound, this is strictly
/// better than the estimate it replaces: the estimate matched the derived bound
/// zero times — 38 over, 1 under — and the one UNDER-estimate truncates a
/// received sample. For the other 81 the behaviour is unchanged, and the
/// escape hatch is the `_sized` form beside every method that uses this
/// (`Subscription<M>::try_recv_sized<N>`, `Stream<T>::try_next_sized<N>`, …),
/// mirroring how `bind_subscription_sized` relates to `bind_subscription`.
template <class M> struct rx_buffer_capacity {
    static constexpr size_t value = detail::buffer_bounds<M>::rx;
};

} // namespace nros

#endif // NROS_CPP_SIZE_BOUND_HPP
