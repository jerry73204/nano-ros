/// @file main.cpp
/// @brief Phase 242.1 + 242.2 (RFC-0044) proof — the rclcpp-faithful component
/// model. Each node **IS-A** `nros::ComponentNode`; its **constructor** receives
/// the executor-bound node handle and creates its entities (a publisher, a typed
/// member-callback subscription, a typed member timer) as member calls. No
/// `configure(Node&)`, no callback names, no raw bytes at the authoring surface.
///
/// Compare `examples/native/cpp/component-poc/` (the RFC-0043 default-construct +
/// `configure(Node&)` + raw-bytes shape this supersedes for rclcpp-faithful nodes).
///
/// Run both roles in one process (default) or split:
/// `component_node_poc talker` / `component_node_poc listener`.

#include <cstdio>
#include <cstring>
#include <new>

#include <nros/component_node.hpp>
#include <nros/main.hpp> // LinuxBoard::run_components (real executor)
#include <nros/nros.hpp>

#include "std_msgs.hpp"

using Int32 = std_msgs::msg::Int32;

// ---- Talker: IS-A node; ctor creates a publisher + a typed member timer ----
class Talker : public nros::ComponentNode {
    rclcpp::Publisher<Int32> pub_;
    int count_ = 0;

  public:
    // Ctor receives the executor-bound handle and wires entities (rclcpp-style).
    // Creation failure aborts (boot-fatal) — no Result threading here.
    explicit Talker(nros::NodeHandle h) : nros::ComponentNode(h, "cn_talker") {
        pub_ = create_publisher<Int32>("/chatter");
        create_wall_timer<Talker, &Talker::on_tick>(500);
    }

    void on_tick() { // real body, bound by identity (no name)
        Int32 m;
        m.data = count_++;
        if (pub_.publish(m).ok()) {
            std::printf("Published: %d\n", m.data);
        }
    }
};

// ---- Listener: IS-A node; ctor creates a typed member-callback subscription -
class Listener : public nros::ComponentNode {
    int recv_ = 0;

  public:
    explicit Listener(nros::NodeHandle h) : nros::ComponentNode(h, "cn_listener") {
        // Typed member-callback subscription (242.2). The macro derives Self from
        // `this`; it registers M::TYPE_NAME (DDS-mangled) + deserializes each
        // sample into a typed Int32 before dispatching to on_msg.
        NROS_SUBSCRIBE(Int32, on_msg, "/chatter");
    }

    void on_msg(const Int32& m) { // typed member callback — real body
        std::printf("Received: %d\n", m.data);
        ++recv_;
    }
};

// Static storage so the components outlive the spin loop (the executor holds
// `&member`/`this` as the dispatch context). No heap. This mirrors the codegen
// Entry's arena placement-new (Phase 242.4) — the lifetime contract is the same.
alignas(Talker) static unsigned char g_talker_buf[sizeof(Talker)];
alignas(Listener) static unsigned char g_listener_buf[sizeof(Listener)];

int main(int argc, char** argv) {
    const char* role = (argc > 1) ? argv[1] : "both";
    std::printf("component-node-poc role=%s\n", role);

    // The board owns init → setup → spin_once loop → shutdown, driving the REAL
    // executor. The components are CONSTRUCTED INSIDE setup (after init), because
    // a ComponentNode ctor creates its node against the now-valid executor handle.
    return ::nros::board::LinuxBoard::run_components([&]() -> int32_t {
        nros::NodeHandle handle(nros::global_handle());
        if (!handle.valid()) {
            return static_cast<int32_t>(nros::ErrorCode::NotInitialized);
        }
        if (std::strcmp(role, "listener") != 0) {
            new (g_talker_buf) Talker(handle); // ctor wires the topology
        }
        if (std::strcmp(role, "talker") != 0) {
            new (g_listener_buf) Listener(handle);
            std::printf("Waiting for messages\n");
        }
        return 0;
    });
}
