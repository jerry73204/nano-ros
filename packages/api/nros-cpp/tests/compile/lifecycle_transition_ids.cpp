// Issue 1099 — `nros::LifecycleNode::trigger_transition(uint8_t)` has rclcpp's
// exact signature, so the id space it accepts MUST be rclcpp's too. It was not:
// four of eight ids disagreed with `lifecycle_msgs/msg/Transition`, and a ported
// `trigger_transition(2)` on an Inactive node ACTIVATED it and returned
// `Result::ok` where ROS 2 cleans it up. Same name, same arity, same type,
// opposite effect — nothing a compiler or a reviewer would flag.
//
// This TU pins three things at compile time:
//
//   1. the C++ enum carries the UPSTREAM numbers (transcribed below from
//      `/opt/ros/humble/share/lifecycle_msgs/msg/Transition.msg`);
//   2. the C++ enum and the C `NROS_LIFECYCLE_TRANSITION_*` macros agree, so a
//      mixed C/C++ image cannot hold two id spaces — the failure mode a
//      translate-at-the-C++-boundary fix would have created;
//   3. `shutdown()` resolves its transition from the CURRENT state, exhaustively.
//
// (1) and (2) fail to compile against the pre-fix header. Built by
// `just check cpp`.

#include <cstdint>

#include "nros/lifecycle.hpp"
#include "nros/nros_generated.h" // NROS_LIFECYCLE_TRANSITION_*

namespace {

// ---------------------------------------------------------------------------
// 1. The enum IS lifecycle_msgs/msg/Transition.
// ---------------------------------------------------------------------------
// Literals, deliberately: an assertion written against the constants it is
// checking proves nothing. These are the numbers a ported rclcpp file means.

constexpr uint8_t id_of(nros::LifecycleTransition t) {
    return static_cast<uint8_t>(t);
}

static_assert(id_of(nros::LifecycleTransition::Configure) == 1, "TRANSITION_CONFIGURE = 1");
static_assert(id_of(nros::LifecycleTransition::Cleanup) == 2, "TRANSITION_CLEANUP = 2");
static_assert(id_of(nros::LifecycleTransition::Activate) == 3, "TRANSITION_ACTIVATE = 3");
static_assert(id_of(nros::LifecycleTransition::Deactivate) == 4, "TRANSITION_DEACTIVATE = 4");
static_assert(id_of(nros::LifecycleTransition::ShutdownUnconfigured) == 5,
              "TRANSITION_UNCONFIGURED_SHUTDOWN = 5");
static_assert(id_of(nros::LifecycleTransition::ShutdownInactive) == 6,
              "TRANSITION_INACTIVE_SHUTDOWN = 6");
static_assert(id_of(nros::LifecycleTransition::ShutdownActive) == 7,
              "TRANSITION_ACTIVE_SHUTDOWN = 7");
// Upstream 8 is TRANSITION_DESTROY, which nano-ros does not implement; upstream
// models error recovery as the implicit TRANSITION_ON_ERROR_SUCCESS = 60.
static_assert(id_of(nros::LifecycleTransition::ErrorRecovery) == 60,
              "TRANSITION_ON_ERROR_SUCCESS = 60");

// The four primary states carry their `lifecycle_msgs/msg/State` ids too.
// `ErrorProcessing` deliberately does not (upstream 15) — but 5 is UNASSIGNED
// upstream, so unlike the transition ids it can never name a different state.
static_assert(static_cast<uint8_t>(nros::LifecycleState::Unconfigured) == 1, "");
static_assert(static_cast<uint8_t>(nros::LifecycleState::Inactive) == 2, "");
static_assert(static_cast<uint8_t>(nros::LifecycleState::Active) == 3, "");
static_assert(static_cast<uint8_t>(nros::LifecycleState::Finalized) == 4, "");

// ---------------------------------------------------------------------------
// 2. C and C++ share ONE id space.
// ---------------------------------------------------------------------------
// This is the assertion that rules out the other candidate fix. Translating
// only at the C++ boundary would have left `NROS_LIFECYCLE_TRANSITION_ACTIVATE`
// (our 2) and `trigger_transition(2)` (upstream CLEANUP) meaning different
// things inside a single mixed-language image — a worse trap than the one being
// fixed, because both spellings look correct.

static_assert(id_of(nros::LifecycleTransition::Configure) == NROS_LIFECYCLE_TRANSITION_CONFIGURE,
              "");
static_assert(id_of(nros::LifecycleTransition::Cleanup) == NROS_LIFECYCLE_TRANSITION_CLEANUP, "");
static_assert(id_of(nros::LifecycleTransition::Activate) == NROS_LIFECYCLE_TRANSITION_ACTIVATE, "");
static_assert(id_of(nros::LifecycleTransition::Deactivate) == NROS_LIFECYCLE_TRANSITION_DEACTIVATE,
              "");
static_assert(id_of(nros::LifecycleTransition::ShutdownUnconfigured) ==
                  NROS_LIFECYCLE_TRANSITION_SHUTDOWN_UNCONFIGURED,
              "");
static_assert(id_of(nros::LifecycleTransition::ShutdownInactive) ==
                  NROS_LIFECYCLE_TRANSITION_SHUTDOWN_INACTIVE,
              "");
static_assert(id_of(nros::LifecycleTransition::ShutdownActive) ==
                  NROS_LIFECYCLE_TRANSITION_SHUTDOWN_ACTIVE,
              "");
static_assert(id_of(nros::LifecycleTransition::ErrorRecovery) ==
                  NROS_LIFECYCLE_TRANSITION_ERROR_RECOVERY,
              "");

// No two transitions may share an id — a renumbering that collapsed two
// variants would otherwise satisfy every row above that it did not touch.
static_assert(id_of(nros::LifecycleTransition::Configure) !=
                  id_of(nros::LifecycleTransition::Cleanup),
              "");
static_assert(id_of(nros::LifecycleTransition::Activate) !=
                  id_of(nros::LifecycleTransition::Deactivate),
              "");
static_assert(id_of(nros::LifecycleTransition::Cleanup) !=
                  id_of(nros::LifecycleTransition::Activate),
              "");

// ---------------------------------------------------------------------------
// 3. shutdown() resolves from the current state.
// ---------------------------------------------------------------------------
// `LifecycleNode::shutdown()` used to hardcode 5 (`ShutdownUnconfigured`),
// whose only legal source state is `Unconfigured` — so it could not shut down
// an Inactive or Active node, i.e. any node that had done work. Every arm,
// including the three states with no legal shutdown, which fall back to
// `ShutdownUnconfigured` so the state machine (not this header) reports the
// error.

static_assert(nros::shutdown_transition_for(nros::LifecycleState::Unconfigured) ==
                  nros::LifecycleTransition::ShutdownUnconfigured,
              "");
static_assert(nros::shutdown_transition_for(nros::LifecycleState::Inactive) ==
                  nros::LifecycleTransition::ShutdownInactive,
              "shutdown() from Inactive must send INACTIVE_SHUTDOWN, not 5");
static_assert(nros::shutdown_transition_for(nros::LifecycleState::Active) ==
                  nros::LifecycleTransition::ShutdownActive,
              "shutdown() from Active must send ACTIVE_SHUTDOWN, not 5");
static_assert(nros::shutdown_transition_for(nros::LifecycleState::Unknown) ==
                  nros::LifecycleTransition::ShutdownUnconfigured,
              "");
static_assert(nros::shutdown_transition_for(nros::LifecycleState::Finalized) ==
                  nros::LifecycleTransition::ShutdownUnconfigured,
              "");
static_assert(nros::shutdown_transition_for(nros::LifecycleState::ErrorProcessing) ==
                  nros::LifecycleTransition::ShutdownUnconfigured,
              "");

// The typed overload must exist and be reachable — it is what keeps a caller
// from writing the literal that caused this issue.
using TypedOverload = nros::Result (nros::LifecycleNode::*)(nros::LifecycleTransition);
using RawOverload = nros::Result (nros::LifecycleNode::*)(uint8_t);
constexpr TypedOverload typed_ = &nros::LifecycleNode::trigger_transition;
constexpr RawOverload raw_ = &nros::LifecycleNode::trigger_transition;
static_assert(typed_ != nullptr, "");
static_assert(raw_ != nullptr, "");

} // namespace
