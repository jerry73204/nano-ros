/// @file main.cpp
/// @brief Phase 240.7 (RFC-0043, = phase-236 236.D.5) — the **transform** node:
/// a stateful component whose subscription callback PUBLISHES (sub → callback →
/// pub), the real ROS pattern beyond a counter. Three roles, one per process:
///
///   transform-poc source   — publishes an Int32 counter on `/in`
///   transform-poc relay     — subscribes `/in`, doubles, publishes `/out`  ← transform
///   transform-poc sink      — subscribes `/out`, prints `Doubled: N`
///
/// All on the real executor via component objects bound by identity — no
/// declarative string layer, no synthesizing interpreter.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nros/component.hpp>
#include <nros/main.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

using Int32 = std_msgs::msg::Int32;

static int32_t read_i32_le(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

// ---- Source: timer publishes a counter on /in -----------------------------
class Source {
    rclcpp::Publisher<Int32> pub_;
    nros::Timer timer_;
    int count_ = 0;

    void on_tick() {
        Int32 m;
        m.data = count_++;
        if (pub_.publish(m).ok()) {
            std::printf("Source published: %d\n", m.data);
        }
    }

  public:
    rclcpp::Result configure(nros::Node& node) {
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        rclcpp::Result r = node.create_publisher(pub_, "/in");
        if (!r.ok()) return r;
        return nros::bind_timer<Source, &Source::on_tick>(node, timer_, 500, this);
    }
};

// ---- Relay: the TRANSFORM — sub /in → callback doubles → pub /out ----------
class Relay {
    rclcpp::Publisher<Int32> out_;

    void on_in(const uint8_t* data, size_t len) {
        int32_t v = (len >= 8) ? read_i32_le(data + 4) : 0;
        Int32 m;
        m.data = v * 2;
        if (out_.publish(m).ok()) {
            std::printf("Relay: %d -> %d\n", v, m.data);
        }
    }

  public:
    rclcpp::Result configure(nros::Node& node) {
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        rclcpp::Result r = node.create_publisher(out_, "/out");
        if (!r.ok()) return r;
        return nros::bind_subscription_raw<Relay, &Relay::on_in>(node, "/in", Int32::TYPE_NAME,
                                                                 this);
    }
};

// ---- Sink: sub /out → print -----------------------------------------------
class Sink {
    void on_out(const uint8_t* data, size_t len) {
        int32_t v = (len >= 8) ? read_i32_le(data + 4) : 0;
        std::printf("Doubled: %d\n", v);
    }

  public:
    rclcpp::Result configure(nros::Node& node) {
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        rclcpp::Result r =
            nros::bind_subscription_raw<Sink, &Sink::on_out>(node, "/out", Int32::TYPE_NAME, this);
        if (r.ok()) std::printf("Waiting for messages\n");
        return r;
    }
};

int main(int argc, char** argv) {
    const char* role = (argc > 1) ? argv[1] : "relay";
    std::printf("transform-poc role=%s\n", role);

    nros::Node node;
    Source source;
    Relay relay;
    Sink sink;

    return ::nros::board::LinuxBoard::run_components([&]() -> int32_t {
        rclcpp::Result r = nros::create_node(node, "transform_poc");
        if (!r.ok()) return static_cast<int32_t>(r.raw());
        if (std::strcmp(role, "source") == 0) {
            r = source.configure(node);
        } else if (std::strcmp(role, "sink") == 0) {
            r = sink.configure(node);
        } else {
            r = relay.configure(node); // default: the transform node
        }
        return static_cast<int32_t>(r.raw());
    });
}
