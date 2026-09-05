// issue 1050 defect (3) — a module must be able to SAY which backend it means.
//
// `nros::init()` opens whichever backend registered FIRST. On a hosted target
// that order is decided before `main` by `.init_array` constructors, so an
// image linking more than one backend hands the session to whichever the linker
// happened to order first. Measured on PX4: a uORB-only module linked against a
// zenoh-carrying `libnros_cpp.a` was given zenoh, dialled
// `tcp/127.0.0.1:7447`, found no router, and failed at `init()` — a module
// broken by a backend it never declared, winning a race it should not have been
// in.
//
// The capability existed twice already (`Executor::open_with_rmw`,
// `NodeBuilder().rmw(...)`) and the one-liner the examples use could not reach
// it. This probe is the surface, not the behaviour: it asserts the entry point
// EXISTS with a shape a PX4 module can call, including the defaulted arguments
// that make the one-liner a one-liner.
//
// Compile-only and freestanding (`-fno-exceptions -fno-rtti -std=c++14`,
// issue 0112) because there is no backend registered in this TU to open. The
// runtime half — a named backend that is not registered must FAIL rather than
// fall back — is the Rust `open_with_rmw_in` contract and is exercised where a
// backend exists.

#include <nros/node.hpp>
#include <stdint.h>

namespace {

// The PX4 shape: one argument, the backend the module actually declared.
nros::Result (*one_liner)(const char*, const char*, uint8_t, const char*) = &nros::init_with_rmw;

// Every argument spelled out, so a signature change here is a compile error
// rather than a silently different overload being selected.
void fully_specified() {
    (void)nros::init_with_rmw("uorb", nullptr, 0, "node");
    (void)nros::init_with_rmw("zenoh", "tcp/127.0.0.1:7447", 7, "px4_demo");
}

// The defaults are the point: a module that wants uORB and nothing else says so
// in one argument, and keeps `init()`'s locator/domain ladder untouched.
void defaults_are_usable() {
    (void)nros::init_with_rmw("uorb");
    (void)nros::init_with_rmw("cyclonedds", "auto");
}

// And the historical spelling still resolves — `init()` is now
// `init_with_rmw(nullptr, ...)`, so this asserts the delegation did not change
// the public surface.
void unchanged_entry_points() {
    (void)nros::init();
    (void)nros::init("tcp/127.0.0.1:7447");
    (void)nros::init(nullptr, nros::kDomainIdExplicitZero);
    (void)nros::init(nullptr, 0, "talker");
}

} // namespace

int main() {
    (void)one_liner;
    fully_specified();
    defaults_are_usable();
    unchanged_entry_points();
    return 0;
}
