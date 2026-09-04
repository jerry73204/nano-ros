// phase-417 W3.b — `rclcpp::init(argc, argv)` is ADOPT-BOUNDED, not compile-refused.
//
// Whether the process was given `--ros-args` is a VALUE, not a type, so the
// compiler cannot see it at the DECLARATION. A `static_assert` on the overload
// would reject every caller -- including the common embedded one that forwards
// `main`'s argv and has no ROS arguments -- and would make upstream's own tutorial
// `main` unportable for a reason unrelated to what the program does.
//
// RFC-0089's rule is that configuration must never be dropped SILENTLY. Compile
// time is the earliest point loudness is available, not the only one; when only
// the value carries the defect, the call is that point. So the call compiles and
// aborts at runtime if `--ros-args` is actually present.
//
// The predicate that decides is `constexpr`, which is what makes the decision
// checkable HERE rather than in a process that would have to die to be observed.
// An abort inlined into `init` is a check nothing ever runs.
#include "nros/nros.hpp"

using rclcpp::detail::argv_has_ros_args;

namespace {
constexpr char const* kPlain[] = {"node", "--flag", "value"};
constexpr char const* kRosArgs[] = {"node", "--ros-args", "-r", "chatter:=/other"};
constexpr char const* kTrailing[] = {"node", "-r", "--ros-args"};
constexpr char const* kWithNull[] = {"node", nullptr, "--ros-args"};
constexpr char const* kPrefix[] = {"node", "--ros-args-extra"};
} // namespace

// absent -> identical to the zero-argument form; nothing is dropped
static_assert(!argv_has_ros_args(3, kPlain), "plain argv must not trip the refusal");
// present -> the call aborts rather than proceeding with an unapplied remap
static_assert(argv_has_ros_args(4, kRosArgs), "--ros-args must trip the refusal");
// LAST position: an off-by-one in the bound is how this silently stops refusing,
// and it would pass every other case here
static_assert(argv_has_ros_args(3, kTrailing), "--ros-args in the last slot must trip");
// a null entry must not end the scan, or the refusal is skippable by argv shape
static_assert(argv_has_ros_args(3, kWithNull), "a null argv entry must not end the scan");
// argc bounds the scan, not the array
static_assert(!argv_has_ros_args(1, kRosArgs), "argc must bound the scan");
// a PREFIX is not the flag -- matching on prefix would refuse unrelated arguments
static_assert(!argv_has_ros_args(2, kPrefix), "--ros-args-extra is a different flag");

// and the upstream tutorial `main` spelling still compiles
void phase417_w3b_call_shape(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::init();
}
