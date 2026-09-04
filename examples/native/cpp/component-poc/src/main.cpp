/// @file main.cpp
/// @brief Phase 240.1 (RFC-0043) proof — stateful component objects bind real
/// member callbacks (timer publish + raw zero-copy subscription) and run through
/// the real executor. No declarative string descriptors, no synthesizing
/// interpreter, no callback names.
///
/// Run both roles in one process (default — relies on the RMW's local delivery)
/// or split: `component_poc talker` / `component_poc listener`.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nros/component.hpp>
#include <nros/main.hpp> // Phase 240.2 — LinuxBoard::run_components (real executor)
#include <nros/nros.hpp>

#include "std_msgs.hpp"

using Int32 = std_msgs::msg::Int32;

// ---- Talker: a timer member callback publishes a real counter --------------
class Talker {
    rclcpp::Publisher<Int32> pub_;
    nros::Timer timer_;
    int count_ = 0;

    void on_tick() { // real body, bound by identity (no name)
        Int32 m;
        m.data = count_++;
        if (pub_.publish(m).ok()) {
            std::printf("Published: %d\n", m.data);
        }
    }

  public:
    rclcpp::Result configure(nros::Node& node) {
        rclcpp::Result r = node.create_publisher(pub_, "/chatter");
        if (!r.ok()) return r;
        return nros::bind_timer<Talker, &Talker::on_tick>(node, timer_, 500, this);
    }
};

// ---- Listener: a raw (zero-copy) sub member callback counts receipts --------
class Listener {
    int recv_ = 0;

    void on_raw(const uint8_t* data, size_t len) { // real body, bound by identity
        int32_t v = 0;
        if (len >= 8) {
            v = static_cast<int32_t>(
                static_cast<uint32_t>(data[4]) | (static_cast<uint32_t>(data[5]) << 8) |
                (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[7]) << 24));
        }
        std::printf("Received: %d\n", v);
        ++recv_;
    }

  public:
    rclcpp::Result configure(nros::Node& node) {
        // NB: the wire keyexpr uses the DDS-mangled type name the typed
        // `Publisher<Int32>` registers (`std_msgs::msg::dds_::Int32_`), not the
        // ROS slash form — so the raw sub must pass `Int32::TYPE_NAME` to match.
        // (Raw-vs-typed type-name-form unification is a separate concern.)
        return nros::bind_subscription_raw<Listener, &Listener::on_raw>(node, "/chatter",
                                                                        Int32::TYPE_NAME, this);
    }
};

int main(int argc, char** argv) {
    const char* role = (argc > 1) ? argv[1] : "both";
    std::printf("component-poc role=%s\n", role);

    // Components + node are stack-owned in main, so they outlive the executor
    // spin loop (which runs inside `run_components` and holds `&member` as the
    // dispatch context). The codegen Entry (240.2b) will own them in static
    // storage; the lifetime contract is the same.
    nros::Node node;
    Talker talker;
    Listener listener;

    // Phase 240.2 — the board owns init → setup → spin_once loop → shutdown,
    // driving the REAL executor (no synthesizing interpreter). `setup`
    // constructs the topology + binds the real member callbacks.
    return ::nros::board::LinuxBoard::run_components([&]() -> int32_t {
        rclcpp::Result r = nros::create_node(node, "component_poc");
        if (!r.ok()) return static_cast<int32_t>(r.raw());
        if (std::strcmp(role, "listener") != 0) {
            r = talker.configure(node);
            if (!r.ok()) return static_cast<int32_t>(r.raw());
        }
        if (std::strcmp(role, "talker") != 0) {
            r = listener.configure(node);
            if (!r.ok()) return static_cast<int32_t>(r.raw());
            std::printf("Waiting for messages\n");
        }
        return 0;
    });
}
