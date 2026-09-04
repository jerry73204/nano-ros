// Aux.cpp — ws-realtime-cpp-mps2 mid-tier auxiliary node.
//
// Publishes a monotonic Int32 counter on /aux every 50 ms via the typed
// Publisher<std_msgs::msg::Int32> (generated serialization). Runs on the
// mid-priority FreeRTOS tier task (priority 3) via FreertosBoard::run_tiers
// (RFC-0015 Model 1 embedded). The mid tier is spawned by a spawned tier
// (boot→mid→low) — a `[aux] tick` proves the #144 chained spawn serialized the
// declares so this tier's publisher write filter opened.

#include "aux_pkg/Aux.hpp"

#include <cstdio>

namespace aux_pkg {

void Aux::on_tick() {
    std_msgs::msg::Int32 msg;
    msg.data = count_;
    if (pub_.publish(msg).ok()) {
        std::printf("[aux] tick=%d\n", count_);
    }
    count_++;
}

::rclcpp::Result Aux::configure(::nros::Node& node) {
    // Line-buffer stdout so each tick flushes immediately when piped.
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    ::rclcpp::Result r = node.create_publisher(pub_, "/aux");
    if (!r.ok()) return r;
    return ::nros::bind_timer<Aux, &Aux::on_tick>(node, timer_, 50, this);
}

} // namespace aux_pkg
