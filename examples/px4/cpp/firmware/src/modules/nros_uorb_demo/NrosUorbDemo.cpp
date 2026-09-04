// nano-ros uORB interop demo — the zero-serialization path (phase-325 W2).
//
// WHAT THIS SHOWS, and why uORB is the odd backend out:
//
//   every other backend                    uORB
//   -------------------                    ----
//   wire bytes = CDR encoding              wire bytes = the PX4 C struct, verbatim
//   identity   = type name + type hash     identity   = ORB_ID(<topic>)
//   cost       = encode + decode / sample  cost       = none, the payload IS the struct
//   readers    = nano-ros / ROS 2 peers    readers    = ANY stock PX4 module
//
// So everywhere else nano-ros interoperates by speaking a wire protocol; here it
// interoperates by SHARING PX4's in-memory type. `nros_rmw_uorb::publisher_create`
// ignores type_name/type_hash/qos/domain entirely and resolves the topic through
// nros_rmw_uorb_register_topic() to a `const struct orb_metadata *`;
// `publisher_publish_raw` checks `len >= meta->o_size` and hands the bytes
// straight to orb_publish(). There is no serialization step to skip — there was
// never one.
//
// PROVING IT: publish through nano-ros, then read it with a STOCK PX4 consumer:
//
//     nros_uorb_demo start
//     listener debug_key_value
//
// `listener` is PX4's own command and knows nothing about nano-ros. That is the
// point. A demo where a nano-ros subscriber reads a nano-ros publisher would pass
// identically with a correct and a broken struct layout — both ends share the
// bug — which is why phase-325 requires a foreign peer (issue 0351's thesis).
//
// Written to PX4 convention: tab indent, Kconfig beside it, ModuleBase<T> +
// ScheduledWorkItem because this daemonizes, PRINT_MODULE_* usage strings.
// nano-ros files carry no per-file copyright block; PX4's BSD 3-clause header
// names the PX4 Development Team and is a licensing practice, not a style rule.

#include <nros/nros.hpp>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/vehicle_status.h>

#include "nros_rmw_uorb.h"
#include "nros_rmw_uorb_registry.h"

#include <cstring>

using namespace time_literals;

namespace {

// Type tags, not message definitions.
//
// nros::Node::create_publisher<M> needs M::TYPE_NAME and M::TYPE_HASH; the typed
// publish() path would additionally need M::ffi_publish, and we never call it.
// On uORB both strings are IGNORED — topic identity is the orb_metadata pointer
// registered below — so the ROS type name is carried for documentation and for
// the day the same topic is bridged to a networked backend (phase-325 W3), where
// it becomes load-bearing.
//
// The PAYLOAD type is PX4's own `debug_key_value_s` from <uORB/topics/...>. There
// is deliberately no generated nano-ros binding: a generated one would describe a
// CDR layout, and CDR is exactly what does not happen here.
struct DebugKeyValueTag {
    static constexpr const char* TYPE_NAME = "px4_msgs::msg::DebugKeyValue";
    static constexpr const char* TYPE_HASH = "";
};

struct VehicleStatusTag {
    static constexpr const char* TYPE_NAME = "px4_msgs::msg::VehicleStatus";
    static constexpr const char* TYPE_HASH = "";
};

constexpr const char* kDebugTopic = "/fmu/out/debug_key_value";
constexpr const char* kStatusTopic = "/fmu/out/vehicle_status";

} // namespace

class NrosUorbDemo : public ModuleBase<NrosUorbDemo>, public px4::ScheduledWorkItem {
  public:
    NrosUorbDemo();
    ~NrosUorbDemo() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char* argv[]);
    static int custom_command(int argc, char* argv[]);
    static int print_usage(const char* reason = nullptr);

    int print_status() override;

    bool init();

  private:
    void Run() override;

    nros::Node _node{};
    rclcpp::Publisher<DebugKeyValueTag> _debug_pub{};
    rclcpp::Subscription<VehicleStatusTag> _status_sub{};

    uint32_t _published{0};
    uint32_t _received{0};
    float _counter{0.0F};
};

NrosUorbDemo::NrosUorbDemo() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

bool NrosUorbDemo::init() {
    // 1. Teach the backend which orb_metadata each ROS-style name maps to. uORB
    //    has no name-keyed metadata lookup of its own, so without this a
    //    create_publisher gets NROS_RMW_RET_TOPIC_NAME_INVALID.
    if (nros_rmw_uorb_register_topic(kDebugTopic, DebugKeyValueTag::TYPE_NAME,
                                     ORB_ID(debug_key_value)) != NROS_RMW_RET_OK) {
        PX4_ERR("register_topic(%s) failed", kDebugTopic);
        return false;
    }

    if (nros_rmw_uorb_register_topic(kStatusTopic, VehicleStatusTag::TYPE_NAME,
                                     ORB_ID(vehicle_status)) != NROS_RMW_RET_OK) {
        PX4_ERR("register_topic(%s) failed", kStatusTopic);
        return false;
    }

    // 2. Ordinary nano-ros bring-up. Nothing here is uORB-specific — the same
    //    three calls appear in every nano-ros example on every backend.
    if (!nros::init().ok()) {
        PX4_ERR("nros::init() failed");
        return false;
    }

    if (!nros::create_node(_node, "nros_uorb_demo").ok()) {
        PX4_ERR("create_node failed");
        return false;
    }

    if (!_node.create_publisher(_debug_pub, kDebugTopic).ok()) {
        PX4_ERR("create_publisher(%s) failed", kDebugTopic);
        return false;
    }

    if (!_node.create_subscription(_status_sub, kStatusTopic).ok()) {
        PX4_ERR("create_subscription(%s) failed", kStatusTopic);
        return false;
    }

    ScheduleOnInterval(1_s);
    return true;
}

void NrosUorbDemo::Run() {
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // --- publish: a PX4 struct, byte for byte -------------------------------
    //
    // No encode step, and none skipped: `msg` IS what lands in the uORB queue.
    // A stock `listener debug_key_value` reads it because it is the same memory
    // layout PX4's own publishers use.
    debug_key_value_s msg{};
    msg.timestamp = hrt_absolute_time();
    std::strncpy(msg.key, "nros", sizeof(msg.key) - 1);
    msg.value = _counter;

    if (_debug_pub.publish_raw(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg)).ok()) {
        _published++;
        _counter += 1.0F;

    } else {
        PX4_ERR("publish_raw failed");
    }

    // --- subscribe: read what a stock PX4 module published -------------------
    //
    // The other direction of the same property. `vehicle_status` is published by
    // PX4's commander, which has never heard of nano-ros; the bytes arrive as
    // `vehicle_status_s` and are used directly.
    uint8_t buf[sizeof(vehicle_status_s)];
    size_t len = 0;

    if (_status_sub.take_serialized(buf, sizeof(buf), len).ok() &&
        len >= sizeof(vehicle_status_s)) {
        const auto* status = reinterpret_cast<const vehicle_status_s*>(buf);
        _received++;

        // Print rarely — this runs at 1 Hz for as long as the module is up.
        if (_received == 1 || (_received % 10) == 0) {
            PX4_INFO("recv vehicle_status: nav_state=%u arming_state=%u (%u samples)",
                     static_cast<unsigned>(status->nav_state),
                     static_cast<unsigned>(status->arming_state), static_cast<unsigned>(_received));
        }
    }

    if (_published == 1 || (_published % 10) == 0) {
        PX4_INFO("published debug_key_value key=nros value=%.1f (%u samples)",
                 static_cast<double>(msg.value), static_cast<unsigned>(_published));
    }
}

int NrosUorbDemo::print_status() {
    PX4_INFO("published: %u  received: %u", static_cast<unsigned>(_published),
             static_cast<unsigned>(_received));
    return 0;
}

int NrosUorbDemo::task_spawn(int argc, char* argv[]) {
    NrosUorbDemo* instance = new NrosUorbDemo();

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (!instance->init()) {
        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    return PX4_OK;
}

int NrosUorbDemo::custom_command(int argc, char* argv[]) {
    return print_usage("unknown command");
}

int NrosUorbDemo::print_usage(const char* reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
nano-ros talking uORB, with no serialization in either direction.

Publishes `debug_key_value` and subscribes `vehicle_status` through the nano-ros
API. On the uORB backend a payload is the PX4 struct itself, so a stock PX4
module reads what nano-ros publishes and vice versa — no CDR, no type hash, no
translation.

Verify from the PX4 side, which is the only verification that proves anything:

$ nros_uorb_demo start
$ listener debug_key_value

`listener` is PX4's own command and knows nothing about nano-ros.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("nros_uorb_demo", "examples");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int nros_uorb_demo_main(int argc, char* argv[]) {
    return NrosUorbDemo::main(argc, argv);
}
