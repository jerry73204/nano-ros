// POSITIVE compile probe for phase-417 W2.b — the branch that only exists when
// the bringup declares `param_services`.
//
// `rclcpp::Node::declare_parameter<T>` forwards to a NODE-LOCAL
// `nros::ParameterServer`. Where the EXECUTOR's parameter store also exists
// (`NROS_SYSTEM_PARAM_SERVICES`, which is what links `nros_cpp_get_param_*`),
// the generated entry has already seeded launch parameters into that store
// before any user code runs — so `declare_parameter` must adopt the seeded
// value over the code default, exactly as `nros::ComponentNode` does for issue
// 0745. Without it a launch parameter would be dead weight, which is the
// "silently drops configuration" RFC-0087 requires to fail to compile.
//
// This TU exists because that branch is compiled by NO other probe: the rest of
// `just check cpp` never defines the macro, so the adoption helper would ship
// unparsed. The define is IN THE FILE rather than on the command line, so the
// lane needs no special flags for it.
//
// `-std=c++17`, not the `-std=c++14` of its neighbours: turning the macro on
// also pulls in `ComponentNode::adopt_launch_seed_`, which is written with
// `if constexpr`. C++17 is what the component lane actually compiles with, so
// this is the standard that branch really sees.
//
// (The two helpers are the same dispatch written twice — `component_node.hpp`
// in C++17, `rclcpp::detail::adopt_executor_param_seed` in `nros.hpp` in C++14,
// because that half must parse at C++14. There should be ONE, in a header both
// reach; that is phase-417 W2.a / issue 0793 and it means editing
// `component_node.hpp`.)

#define NROS_SYSTEM_PARAM_SERVICES 1

#include <nros/nros.hpp>

#include <string>

namespace nros_cpp_ros2_param_launch_seed_test {

// Every parameter type this path can carry end-to-end. The list is short, and
// the reason is `nros::ParameterServer`, not this shim: its `declare_impl` /
// `get_impl` overload sets cover `bool`, `int`, `int64_t`, `double` and (for
// declare/set only) `const char*`, plus `std::string` behind `NROS_CPP_STD`.
//
// Measured while writing this probe, and worth recording because it is the
// next stage-2 gap rather than a property of the forwarders:
//
//   * `declare_parameter<float>` does NOT compile -- there is no
//     `get_impl(const char*, float&)`, so the read-back has nothing to bind.
//     `float` is the type a ported control node most often uses for a gain.
//   * `declare_parameter<std::string>` compiles only where `NROS_CPP_STD` is
//     defined (`parameter.hpp:459-465`). This shim is hosted-STL by
//     construction and still does not define that macro, because doing so
//     would change what every other nano-ros header does in the same TU --
//     the flag-gated-struct-field hazard of issue 0135 -- which is not a
//     decision this file can make on its own.
//
// Both are compile ERRORS, so they are loud rather than silent and the
// compile-or-conform rule is satisfied. They are still porting friction, and
// they live in `parameter.hpp`.
inline void declare_every_seedable_type(rclcpp::Node& node) {
    const bool verbose = node.declare_parameter<bool>("verbose", false);
    const int64_t depth = node.declare_parameter<int64_t>("queue_depth", 10);
    const int narrow = node.declare_parameter<int>("narrow", 1);
    const double period = node.declare_parameter<double>("ctrl_period", 0.15);

    (void)verbose;
    (void)depth;
    (void)narrow;
    (void)period;
}

// The seed adoption is what makes a launch parameter reach the node: each of
// these types has a matching `nros_cpp_get_param_*` on the executor store, and
// `declare_parameter` must prefer the seeded value over the code default.
// Reading them back through the two-argument form pins the same path.
inline void read_back(rclcpp::Node& node) {
    bool b = false;
    int64_t i = 0;
    double d = 0.0;
    (void)node.get_parameter<bool>("verbose", b);
    (void)node.get_parameter<int64_t>("queue_depth", i);
    (void)node.get_parameter<double>("ctrl_period", d);
    (void)node.set_parameter<double>("ctrl_period", 0.05).ok();
    (void)node.has_parameter("ctrl_period");
}

// `std::string`-KEYED (not std::string-valued) -- how rclcpp itself keys
// parameters, and the reason the shim carries a second set of overloads.
inline void declare_string_keyed(rclcpp::Node& node) {
    const std::string prefix("ctrl.");
    (void)node.declare_parameter<double>(prefix + "gain", 1.0);
    (void)node.get_parameter<double>(prefix + "gain");
    (void)node.has_parameter(prefix + "gain");
    (void)node.set_parameter<double>(prefix + "gain", 2.0).ok();
}

} // namespace nros_cpp_ros2_param_launch_seed_test
