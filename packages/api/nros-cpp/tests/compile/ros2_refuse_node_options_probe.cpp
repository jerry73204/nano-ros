/*
 * NEGATIVE probe — phase-417 W3.f, one of RFC-0087's two live inversions.
 *
 * `rclcpp::NodeOptions`' ten option setters stored their argument in a private
 * field that NOTHING read, and returned `*this`, so the idiomatic chained call
 * compiled and configured nothing. RFC-0087's rule: a contract that silently
 * drops configuration must fail to COMPILE.
 *
 * `just check cpp` requires this TU to FAIL, and requires the failure text to
 * name the concept and the alternative. A refusal that merely says
 * "use of deleted function" would pass an exit-code check while teaching
 * nothing, which is why the lane greps the message too.
 *
 * The POSITIVE half is `ros2_api_adoption_stage2.cpp`, which proves the same
 * header compiles and that `rclcpp::NodeOptions{}` plus `Node(name, options)`
 * — the shapes a composable node actually needs — still work. Compile that one
 * FIRST: an expected-failure compile cannot tell "the refusal fired" from
 * "the file is not there".
 */

#include <nros/rclcpp_compat.hpp>

#include <string>
#include <vector>

int ros2_refuse_node_options_probe();
int ros2_refuse_node_options_probe() {
    // The chain RFC-0087 quotes: compiles today, configures nothing.
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

    // ...and the read-back, which reports a policy that does not exist.
    (void)options.use_intra_process_comms();

    // The other nine, so a partial fix cannot pass this probe.
    (void)rclcpp::NodeOptions().arguments(std::vector<std::string>{"--ros-args"});
    (void)rclcpp::NodeOptions().use_global_arguments(false);
    (void)rclcpp::NodeOptions().enable_rosout(true);
    (void)rclcpp::NodeOptions().start_parameter_services(true);
    (void)rclcpp::NodeOptions().start_parameter_event_publisher(true);
    (void)rclcpp::NodeOptions().allow_undeclared_parameters(true);
    (void)rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    (void)rclcpp::NodeOptions().enable_topic_statistics(true);
    (void)rclcpp::NodeOptions().enable_logger_service(true);
    return 0;
}
