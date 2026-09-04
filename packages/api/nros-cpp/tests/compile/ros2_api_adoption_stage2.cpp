// POSITIVE compile probe for phase-417 stage 2 (RFC-0087) — the node surface a
// real ported rclcpp node reaches once it stops being a tutorial, plus the
// named QoS profiles the loudness pass corrected.
//
// Everything in this TU must COMPILE. Its negative siblings — one per
// REFUSE-LOUD concept — are the `ros2_refuse_*_probe.cpp` files, each of which
// must FAIL. The split matters: an expected-failure compile cannot tell "the
// refusal fired" from "the file is not there", so the positive TU is what
// proves the include paths and the surface are real before any refusal is
// believed.
//
// Compiled HOSTED (no `-ffreestanding`) for the same reason
// `ros2_api_adoption.cpp` is: `rclcpp_compat.hpp` pulls <memory>/<string>/
// <sstream> unconditionally and hands out std::shared_ptr in public
// signatures.
//
// W2.b  parameters on the node — declare / get / set / has, both `const char*`
//       and `std::string` keyed.
// W2.c  create_service / create_client, poll-style and callback-style.
// W2.d  rclcpp::Rate / WallRate as forwarders onto nros::spin.
// W3.f  the named QoS profiles, transcribed from upstream. The static_asserts
//       below ARE the table-driven test RFC-0087 W3.f asks for: the profiles
//       are constexpr, so the check is a compile-time comparison against the
//       values read out of /opt/ros/humble rather than a runtime test no
//       embedded lane runs.
// W3.a  RCLCPP_*_STREAM carries its message.

#include <nros/rclcpp_compat.hpp>

// Not reachable from the `nros.hpp` umbrella (a separate stage-2 gap: 15 of 46
// headers are not), so name it directly.
#include <nros/fixed_string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <type_traits>

namespace nros_cpp_ros2_api_adoption_stage2_test {

// --- W3.f: the named QoS profiles, checked against upstream -----------------
//
// Each row is transcribed from the file named beside it. A wrong value here is
// the defect RFC-0087 was written about: `ParametersQoS()` shipped at depth 10
// where upstream is 1000, under a name that claims to be the ROS 2 profile.
//
//   rmw_qos_profile_sensor_data       qos_profiles.h:25   KEEP_LAST 5    BEST_EFFORT VOLATILE
//   rmw_qos_profile_parameters        qos_profiles.h:38   KEEP_LAST 1000 RELIABLE    VOLATILE
//   rmw_qos_profile_default           qos_profiles.h:51   KEEP_LAST 10   RELIABLE    VOLATILE
//   rmw_qos_profile_services_default  qos_profiles.h:64   KEEP_LAST 10   RELIABLE    VOLATILE
//   rmw_qos_profile_parameter_events  qos_profiles.h:77   KEEP_LAST 1000 RELIABLE    VOLATILE
//   rcl_qos_profile_rosout_default    logging_rosout.h:37 KEEP_LAST 1000 RELIABLE TRANSIENT_LOCAL,
//   lifespan {10,0} rclcpp::ClockQoS                  rclcpp/qos.hpp:351  KEEP_LAST 1 BEST_EFFORT
//   VOLATILE

static_assert(rclcpp::SensorDataQoS().depth() == 5, "SensorDataQoS depth is 5 upstream");
static_assert(rclcpp::SensorDataQoS().history() == ::nros::KeepLast, "SensorDataQoS is KEEP_LAST");
static_assert(rclcpp::SensorDataQoS().reliability() == ::nros::BestEffort,
              "SensorDataQoS is BEST_EFFORT upstream");
static_assert(rclcpp::SensorDataQoS().durability() == ::nros::Volatile,
              "SensorDataQoS is VOLATILE upstream");

static_assert(rclcpp::ServicesQoS().depth() == 10, "ServicesQoS depth is 10 upstream");
static_assert(rclcpp::ServicesQoS().reliability() == ::nros::Reliable,
              "ServicesQoS is RELIABLE upstream");
static_assert(rclcpp::ServicesQoS().durability() == ::nros::Volatile,
              "ServicesQoS is VOLATILE upstream");

// The inversion RFC-0087 names first. 10 was the shipped value; 1000 is
// upstream's, and a hundredfold history difference costs samples under load
// with nothing to read.
static_assert(rclcpp::ParametersQoS().depth() == 1000,
              "ParametersQoS depth is 1000 upstream (rmw_qos_profile_parameters), not 10");
static_assert(rclcpp::ParametersQoS().reliability() == ::nros::Reliable,
              "ParametersQoS is RELIABLE upstream");
static_assert(rclcpp::ParametersQoS().durability() == ::nros::Volatile,
              "ParametersQoS is VOLATILE upstream");

static_assert(rclcpp::ParameterEventsQoS().depth() == 1000,
              "ParameterEventsQoS depth is 1000 upstream");
static_assert(rclcpp::ParameterEventsQoS().reliability() == ::nros::Reliable,
              "ParameterEventsQoS is RELIABLE upstream");

static_assert(rclcpp::RosoutQoS().depth() == 1000, "RosoutQoS depth is 1000 upstream");
static_assert(rclcpp::RosoutQoS().reliability() == ::nros::Reliable,
              "RosoutQoS is RELIABLE upstream");
static_assert(rclcpp::RosoutQoS().durability() == ::nros::TransientLocal,
              "RosoutQoS is TRANSIENT_LOCAL upstream -- this is the policy a late joiner needs");
static_assert(rclcpp::RosoutQoS().lifespan().nanoseconds() == 10000000000LL,
              "RosoutQoS lifespan is {10, 0} == 10 s upstream");

static_assert(rclcpp::ClockQoS().depth() == 1, "ClockQoS depth is 1 upstream");
static_assert(rclcpp::ClockQoS().reliability() == ::nros::BestEffort,
              "ClockQoS is BEST_EFFORT upstream");

// `rclcpp::QoS(10)` is `rmw_qos_profile_default`, and nothing about the
// correction above may move it.
static_assert(rclcpp::QoS(10).depth() == 10, "rclcpp::QoS(depth) keeps its depth");
static_assert(rclcpp::QoS(10).reliability() == ::nros::Reliable,
              "rmw_qos_profile_default is RELIABLE");
static_assert(rclcpp::KeepLast(7).depth() == 7, "KeepLast(n) carries n");
static_assert(rclcpp::KeepAll().history() == ::nros::KeepAll, "KeepAll() is KEEP_ALL");

// Both upstream spellings construct: these are classes now, as upstream's are.
inline void qos_spellings() {
    rclcpp::SensorDataQoS braced{};
    rclcpp::SensorDataQoS called = rclcpp::SensorDataQoS();
    ::nros::QoS as_base = rclcpp::ServicesQoS();
    rclcpp::QoS from_keep_last(rclcpp::KeepLast(10));
    (void)braced;
    (void)called;
    (void)as_base;
    (void)from_keep_last;
}

// --- Message / service stubs, the shapes codegen emits ----------------------

struct StringMsg {
    ::nros::FixedString<64> data;
    static const size_t SERIALIZED_SIZE_MAX = 128;
    static constexpr const char* TYPE_NAME = "std_msgs::msg::dds_::String_";
    static constexpr const char* TYPE_HASH = "RIHS01_string_stub";
    static int ffi_publish(void*, const void*) { return 0; }
    static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
        if (out) *out = 0;
        return 0;
    }
};

struct AddTwoInts {
    struct Request {
        int64_t a = 0;
        int64_t b = 0;
        static const size_t SERIALIZED_SIZE_MAX = 16;
        static constexpr const char* TYPE_NAME =
            "example_interfaces::srv::dds_::AddTwoInts_Request_";
        static constexpr const char* TYPE_HASH = "RIHS01_add_two_ints_request_stub";
        static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
            if (out) *out = 0;
            return 0;
        }
        static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    };
    struct Response {
        int64_t sum = 0;
        static const size_t SERIALIZED_SIZE_MAX = 8;
        static constexpr const char* TYPE_NAME =
            "example_interfaces::srv::dds_::AddTwoInts_Response_";
        static constexpr const char* TYPE_HASH = "RIHS01_add_two_ints_response_stub";
        static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
            if (out) *out = 0;
            return 0;
        }
        static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    };
    static constexpr const char* TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_";
    static constexpr const char* TYPE_HASH = "RIHS01_srv_stub";
};

// The nano-ros handler shapes — a plain function pointer, which is what the
// SFINAE guard on the callback-style overloads admits.
inline void add_two_ints(const AddTwoInts::Request& req, AddTwoInts::Response& res) {
    res.sum = req.a + req.b;
}
inline void on_add_two_ints_response(const AddTwoInts::Response& res) {
    (void)res;
}

// --- W2.b / W2.c / W2.d: the ported node ------------------------------------

class PortedServiceNode : public rclcpp::Node {
  public:
    PortedServiceNode() : rclcpp::Node("ported_service_node") {
        // W2.b — parameters, `const char*` keyed. The value-returning shape is
        // `nros::ComponentNode`'s, so the two facades in this package agree.
        const double period = this->declare_parameter<double>("ctrl_period", 0.15);
        const int64_t depth = this->declare_parameter<int64_t>("queue_depth", 10);
        const bool verbose = this->declare_parameter<bool>("verbose", false);
        (void)period;
        (void)depth;
        (void)verbose;

        // ...and `std::string` keyed, which is how rclcpp itself keys them.
        const std::string prefix("ctrl.");
        const double gain = this->declare_parameter<double>(prefix + "gain", 1.0);
        (void)gain;

        // Upstream's two-argument read, plus the value-returning one.
        double read_back = 0.0;
        const bool found = this->get_parameter<double>("ctrl_period", read_back);
        const double by_value = this->get_parameter<double>("ctrl_period");
        const bool present = this->has_parameter("ctrl_period");
        const bool present_str = this->has_parameter(std::string("ctrl_period"));
        (void)found;
        (void)by_value;
        (void)present;
        (void)present_str;

        // Set forwards to nros::ParameterServer::set_parameter.
        ::nros::Result set_result = this->set_parameter<double>("ctrl_period", 0.05);
        (void)set_result.ok();
        (void)this->set_parameter<double>(std::string("ctrl_period"), 0.05);

        // The C-API escape hatch, same one ComponentNode offers.
        nros_parameter_server_t* raw = this->parameters().raw();
        (void)raw;

        // W2.c — a service and a client, both call shapes.
        poll_service_ = this->create_service<AddTwoInts>("add_two_ints");
        cb_service_ = this->create_service<AddTwoInts>("add_two_ints_cb", &add_two_ints);
        cb_service_lambda_ = this->create_service<AddTwoInts>(
            "add_two_ints_lambda",
            [](const AddTwoInts::Request& req, AddTwoInts::Response& res) { res.sum = req.a; });

        future_client_ = this->create_client<AddTwoInts>("add_two_ints");
        cb_client_ = this->create_client<AddTwoInts>("add_two_ints", &on_add_two_ints_response);

        // With an explicit QoS, including the corrected named profiles.
        auto with_qos = this->create_service<AddTwoInts>("add_qos", rclcpp::ServicesQoS());
        (void)with_qos;
        publisher_ = this->create_publisher<StringMsg>("chatter", rclcpp::SensorDataQoS());

        // W3.a — the stream macros carry their message now.
        RCLCPP_INFO_STREAM(this->get_logger(), "period=" << period << " depth=" << depth);
        RCLCPP_WARN_STREAM(this->get_logger(), "late by " << 3 << "ms");
        RCLCPP_ERROR_STREAM(this->get_logger(), "failed: " << std::string("reason"));
        RCLCPP_DEBUG_STREAM(this->get_logger(), "dbg " << 1);
        RCLCPP_FATAL_STREAM(this->get_logger(), "fatal " << 1);
        RCLCPP_INFO(this->get_logger(), "plain %d", 1);
    }

  private:
    rclcpp::Publisher<StringMsg>::SharedPtr publisher_;
    rclcpp::Service<AddTwoInts>::SharedPtr poll_service_;
    rclcpp::Service<AddTwoInts>::SharedPtr cb_service_;
    rclcpp::Service<AddTwoInts>::SharedPtr cb_service_lambda_;
    rclcpp::Client<AddTwoInts>::SharedPtr future_client_;
    rclcpp::Client<AddTwoInts>::SharedPtr cb_client_;
};

// The returned pointers are the nested-alias types the entity carries, so a
// ported member declaration binds without a rewrite.
static_assert(std::is_same<decltype(std::declval<rclcpp::Node&>().create_service<AddTwoInts>("s")),
                           rclcpp::Service<AddTwoInts>::SharedPtr>::value,
              "create_service<S>(name) must return rclcpp::Service<S>::SharedPtr");
static_assert(std::is_same<decltype(std::declval<rclcpp::Node&>().create_client<AddTwoInts>("s")),
                           rclcpp::Client<AddTwoInts>::SharedPtr>::value,
              "create_client<S>(name) must return rclcpp::Client<S>::SharedPtr");

// --- W2.d: the rate loop, written the upstream way --------------------------

inline void rate_loop() {
    rclcpp::WallRate rate(10.0); // 10 Hz
    rclcpp::Rate from_period(std::chrono::milliseconds(100));
    rate.reset();
    const std::chrono::nanoseconds period = rate.period();
    (void)period;
    while (rclcpp::ok()) {
        const bool on_time = rate.sleep();
        if (!on_time) break;
    }
    (void)from_period.sleep();
}

static_assert(std::is_same<rclcpp::WallRate, rclcpp::Rate>::value,
              "WallRate is Rate here -- both read the monotonic clock (documented envelope)");

// --- NodeOptions still CONSTRUCTS; only its options refuse ------------------
//
// This is the half that must keep compiling: `Node(name, options)` is the
// load-bearing shape for a composable node, and an empty options object claims
// nothing.

class OptionComponent : public rclcpp::Node {
  public:
    explicit OptionComponent(const rclcpp::NodeOptions& options)
        : rclcpp::Node("option_component", options) {}
};

inline void node_options_construct() {
    rclcpp::NodeOptions braced{};
    rclcpp::NodeOptions called = rclcpp::NodeOptions();
    OptionComponent c(rclcpp::NodeOptions{});
    const rclcpp::NodeOptions& back = c.get_node_options();
    (void)braced;
    (void)called;
    (void)back;
}

// --- The zero-argument init still works -------------------------------------

inline void lifecycle() {
    rclcpp::init();
    (void)rclcpp::ok();
    (void)rclcpp::shutdown();
}

inline void instantiate() {
    qos_spellings();
    rate_loop();
    node_options_construct();
    lifecycle();
}

} // namespace nros_cpp_ros2_api_adoption_stage2_test

// phase-417 W3.b — the upstream tutorial `main` spelling COMPILES. It is
// ADOPT-BOUNDED, not refused: `--ros-args` is a runtime value, so the refusal
// fires at the call (see `ros2_init_argv_runtime_refusal.cpp` for the predicate).
// Syntax-only here deliberately — naming `init` in a TU that links drags in the
// session runtime.
static void phase417_w3b_init_argv_shape_compiles(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::init();
}
