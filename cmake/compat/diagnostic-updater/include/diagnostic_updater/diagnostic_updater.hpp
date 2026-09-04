// SPDX-License-Identifier: Apache-2.0
//
// diagnostic_updater.hpp — Phase 209.D
//
// `diagnostic_updater::Updater` source-compat shim — lets a ROS 2 node that
// publishes `diagnostic_msgs/DiagnosticArray` through the upstream Updater
// compile + run against nano-ros without source edits.
//
// Surface mirrored from upstream `diagnostic_updater::Updater`:
//   * Constructor `Updater(rclcpp::Node::SharedPtr node, double period = 1.0)`
//     and `Updater(rclcpp::Node::SharedPtr node, double, double freq_hz)`.
//   * `add(name, std::function<void(DiagnosticStatusWrapper&)>)`.
//   * `add(name, T* obj, void (T::*method)(DiagnosticStatusWrapper&))`.
//   * `setHardwareID(id)` / `getHardwareID()`.
//   * `force_update()` — publish immediately.
//   * `update()` — publish if `period_` has elapsed since last publish (no-op
//     otherwise); intended to be called from the node's executor loop. The
//     application is responsible for invoking it (typically from a periodic
//     timer or from `rclcpp::spin_some`'s caller); nano-ros has no
//     auto-firing timer registration through the rclcpp_compat Node shim yet.
//   * `broadcast(level, message)` — publishes a single status with the given
//     summary, no add(...)s.
//
// Out of scope (deferred): `DiagnosticTask` class (the function-call form
// covers most uses), `CompositeDiagnosticTask`, frequency-based stats helpers
// (`FrequencyStatus`/`TimeStampStatus` from `update_functions.hpp`). Add as
// new ports surface them.

#ifndef NROS_DIAGNOSTIC_UPDATER_DIAGNOSTIC_UPDATER_HPP
#define NROS_DIAGNOSTIC_UPDATER_DIAGNOSTIC_UPDATER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nros/nros.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

namespace diagnostic_updater {

class Updater {
public:
    using TaskCallback = std::function<void(DiagnosticStatusWrapper&)>;

    Updater(std::shared_ptr<rclcpp::Node> node, double period_seconds = 1.0)
        : node_(std::move(node)),
          period_seconds_(period_seconds > 0.0 ? period_seconds : 1.0),
          last_publish_(std::chrono::steady_clock::now()) {
        if (node_) {
            publisher_ = node_->create_publisher<::diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", rclcpp::QoS(10));
        }
    }

    // Legacy upstream form: (node, period_seconds, frequency_hz). Frequency
    // arg is ignored — `period_seconds` wins.
    Updater(std::shared_ptr<rclcpp::Node> node, double period_seconds, double /*freq_hz*/)
        : Updater(std::move(node), period_seconds) {}

    Updater(const Updater&) = delete;
    Updater& operator=(const Updater&) = delete;

    // --- hardware id --------------------------------------------------------
    void setHardwareID(const std::string& id) { hardware_id_ = id; }
    const std::string& getHardwareID() const { return hardware_id_; }
    // Snake-case alias the older Autoware code uses.
    void set_hardware_id(const std::string& id) { setHardwareID(id); }

    // --- task registration --------------------------------------------------
    void add(const std::string& name, TaskCallback callback) {
        tasks_.emplace_back(name, std::move(callback));
    }

    template <typename T>
    void add(const std::string& name, T* obj,
             void (T::*method)(DiagnosticStatusWrapper&)) {
        add(name, [obj, method](DiagnosticStatusWrapper& w) { (obj->*method)(w); });
    }

    void removeByName(const std::string& name) {
        for (auto it = tasks_.begin(); it != tasks_.end();) {
            if (it->first == name) {
                it = tasks_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // --- publish ------------------------------------------------------------
    void force_update() { publish_now(); }

    void update() {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration<double>(now - last_publish_).count();
        if (elapsed >= period_seconds_) {
            publish_now();
        }
    }

    void broadcast(uint8_t level, const std::string& message) {
        if (!publisher_) {
            return;
        }
        ::diagnostic_msgs::msg::DiagnosticArray array;
        ::diagnostic_msgs::msg::DiagnosticStatus status;
        status.level       = level;
        status.name        = (const char*) "";
        status.message     = message.c_str();
        status.hardware_id = hardware_id_.c_str();
        array.status.push_back(std::move(status));
        publisher_->publish(array);
        last_publish_ = std::chrono::steady_clock::now();
    }

    void setPeriod(double period_seconds) {
        period_seconds_ = period_seconds > 0.0 ? period_seconds : period_seconds_;
    }

private:
    using Task = std::pair<std::string, TaskCallback>;

    void publish_now() {
        if (!publisher_) {
            return;
        }
        ::diagnostic_msgs::msg::DiagnosticArray array;
        for (const auto& task : tasks_) {
            DiagnosticStatusWrapper w;
            task.second(w);
            w.name        = task.first.c_str();
            w.hardware_id = hardware_id_.c_str();
            array.status.push_back(static_cast<::diagnostic_msgs::msg::DiagnosticStatus>(w));
        }
        publisher_->publish(array);
        last_publish_ = std::chrono::steady_clock::now();
    }

    std::shared_ptr<rclcpp::Node> node_;
    double period_seconds_;
    std::chrono::steady_clock::time_point last_publish_;
    std::string hardware_id_;
    std::vector<Task> tasks_;
    std::shared_ptr<::nros::Publisher<::diagnostic_msgs::msg::DiagnosticArray>> publisher_;
};

}  // namespace diagnostic_updater

#endif  // NROS_DIAGNOSTIC_UPDATER_DIAGNOSTIC_UPDATER_HPP
