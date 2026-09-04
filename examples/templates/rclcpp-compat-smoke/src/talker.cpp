// SPDX-License-Identifier: Apache-2.0
//
// rclcpp-compat smoke — Phase 209 MVP-quartet integration test.
//
// Verifies that A (`nros/rclcpp_compat.hpp`) + B (`NrosRclcppCompat.cmake`) +
// C (`rclcpp_components_compat.hpp`) + D (`diagnostic_updater::Updater`) compose:
// a ROS-2-idiom source (no `#include <nros/...>` except the codegen umbrella for
// the message type) compiles + links + publishes through nano-ros.
//
// What is intentionally rclcpp-idiom:
//   * `rclcpp::init(argc, argv)` / `shutdown()` / `ok()`.
//   * Subclass `rclcpp::Node` with `std::make_shared<MySmokeNode>("name")`.
//   * `this->create_publisher<M>(topic, qos)` returning `shared_ptr<Publisher>`.
//   * `rclcpp::spin_some(node)` in the loop.
//   * `diagnostic_updater::Updater(shared_from_this(), period)` + `add(...)` +
//     `update()`. (Updater publishes a `diagnostic_msgs/DiagnosticArray` on
//     `/diagnostics` rate-limited by the period arg.)
//   * `RCLCPP_INFO(get_logger(), ...)` log macro.
//
// What is nano-ros-specific (one line; documented):
//   * The std_msgs / diagnostic_msgs umbrella headers are nano-ros's generated
//     layout (`<pkg>/<pkg>.hpp`), not the upstream `<pkg>/msg/<name>.hpp` form.
//     Closing that gap is 209.E (codegen emits the upstream layout too).

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>                           // → nros/nros.hpp
#include <diagnostic_updater/diagnostic_updater.hpp>   // → 209.D shim

// nano-ros codegen umbrella for std_msgs (Int32 lives here).
#include "std_msgs/std_msgs.hpp"

class SmokeNode : public rclcpp::Node {
public:
    SmokeNode() : rclcpp::Node("rclcpp_compat_smoke") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("smoke_topic", 10);
        RCLCPP_INFO(this->get_logger(), "%s",
                    "rclcpp_compat_smoke up; publishing std_msgs/Int32 on smoke_topic");
    }

    // Two-phase init: rclcpp idiom is `auto n = std::make_shared<MyNode>();
    // n->post_init();` because `shared_from_this()` cannot be called from the
    // ctor. (Upstream rclcpp has the same restriction.)
    void post_init() {
        updater_ = std::make_shared<diagnostic_updater::Updater>(
            shared_from_this(), /*period_seconds=*/1.0);
        updater_->setHardwareID("smoke");
        updater_->add("publish_count",
                      [this](diagnostic_updater::DiagnosticStatusWrapper& w) {
                          w.summary(diagnostic_updater::OK, "alive");
                          w.add("count", static_cast<int>(count_));
                      });
    }

    void tick() {
        std_msgs::msg::Int32 msg;
        msg.data = count_++;
        publisher_->publish(msg);
        if (updater_) {
            updater_->update();
        }
    }

private:
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher_;
    std::shared_ptr<diagnostic_updater::Updater> updater_;
    int32_t count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SmokeNode>();
    node->post_init();

    auto logger = rclcpp::get_logger("rclcpp_compat_smoke");
    RCLCPP_INFO(logger, "spinning; ctrl-c to stop");

    while (rclcpp::ok()) {
        node->tick();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
