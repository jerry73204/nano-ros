// SPDX-License-Identifier: Apache-2.0
//
// minimal_publisher.cpp — the canonical ROS 2 "minimal publisher" tutorial
// node, vendored UNMODIFIED. (Source pattern from the ROS 2 tutorials —
// `Writing a simple publisher and subscriber (C++)`.)
//
// Phase 209.G iter 2: proves an unmodified ROS-2-generic C++ source compiles
// + links + runs against nano-ros through the 209.A–D compat surface, with
// only the build-script glue prepended (one `include(NrosRclcppCompat.cmake)`
// line). No source edits.
//
// What this exercises:
//   * Subclass `rclcpp::Node` with the `(name)` constructor.
//   * `create_publisher<M>(topic, qos)` returning `shared_ptr<Publisher<M>>`.
//   * `create_wall_timer(period, callback)` with a capturing-lambda callback.
//   * `std_msgs::msg::String` message (heap-allocated string field on
//     upstream; nano-ros codegen uses FixedString — adapted in the timer).
//   * `RCLCPP_INFO` log macro.
//   * `rclcpp::init(argc, argv)` + `rclcpp::spin(node)` + `rclcpp::shutdown()`.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/std_msgs.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
        : rclcpp::Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, [this]() { this->timer_callback(); });
    }

private:
    void timer_callback() {
        std_msgs::msg::String message;
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
