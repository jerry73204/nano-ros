// phase-417 W4.c — the executor lifecycle verbs must agree across our own three
// languages, and C++ was the one that could not stop a spin at all.
//
// Measured before this landed: C had `nros_executor_stop`; Rust had
// `Executor::halt` / `is_halted`; C++ had NEITHER — `spin()` looped on
// `while (initialized_)` and only `shutdown()` cleared that, and `shutdown()`
// calls `nros_cpp_fini`. So the standard rclcpp clean-shutdown idiom
// (`cancel()` from a signal handler) cost a full middleware teardown and a fresh
// discovery round on the way back. `ok()` answered "am I initialised", not "am I
// spinning", so neither half of the cancel contract had a reader.
//
// A COMPILE probe rather than a runtime one for the same reason as
// `spin_verbs.cpp`: the defect was the SHAPE of the API — which names exist,
// what they return, and whether a `const Executor&` can be asked. Terminating a
// real spin is asserted Rust-side in
// `nros-node/src/executor/spin.rs::cancel_tests`, where the flag and the loop
// actually live (RFC-0019: the Rust API is the implementation source of truth,
// C and C++ are shims that delegate).

#include "nros/executor.hpp"
#include <type_traits>

namespace {

// `cancel()` exists, takes no argument, and reports like every other verb.
static_assert(std::is_same<decltype(std::declval<nros::Executor&>().cancel()), nros::Result>::value,
              "Executor::cancel() must exist with no argument and return Result "
              "(rclcpp::Executor::cancel shape)");

// `is_spinning()` exists and is a predicate, not a Result.
static_assert(
    std::is_same<decltype(std::declval<nros::Executor&>().is_spinning()), bool>::value,
    "Executor::is_spinning() must exist and return bool (rclcpp::Executor::is_spinning shape)");

// ...and it is askable of a CONST executor. A predicate that needs a mutable
// reference cannot be polled by a supervisor holding `const Executor&`, which is
// the caller this observable exists for.
static_assert(
    std::is_same<decltype(std::declval<const nros::Executor&>().is_spinning()), bool>::value,
    "Executor::is_spinning() must be const-callable");

// `ok()` survives and stays a DIFFERENT question: "am I initialised". An
// executor that has never spun is `ok()` and not spinning. If someone ever
// collapses the two, this file is where the intent is written down.
static_assert(std::is_same<decltype(std::declval<const nros::Executor&>().ok()), bool>::value,
              "Executor::ok() must remain, answering initialisation not spinning");

// `shutdown()` survives and still MEANS teardown. cancel and shutdown are
// different verbs; collapsing them is the bug this work item fixed.
static_assert(
    std::is_same<decltype(std::declval<nros::Executor&>().shutdown()), nros::Result>::value,
    "Executor::shutdown() must remain the teardown verb, distinct from cancel()");

// The FFI slots the two forward to are DECLARED, not merely defined. cbindgen
// does not expand macros, so a `#[no_mangle]` that never reached the header is
// uncallable from C++ while the Rust side stays green (the phase-381 W4 class).
// Taking function POINTERS makes a missing declaration a build failure here.
void* const kEntryPoints[] = {
    reinterpret_cast<void*>(&nros_cpp_executor_cancel),
    reinterpret_cast<void*>(&nros_cpp_executor_is_spinning),
    reinterpret_cast<void*>(&nros_cpp_spin),
};

} // namespace

int main() {
    return kEntryPoints[0] != nullptr ? 0 : 1;
}
