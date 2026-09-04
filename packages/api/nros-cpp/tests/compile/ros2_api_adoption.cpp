// Compile regression for phase-417 stage 1 (RFC-0089) — the three "cheap
// unblockers" that the first lines of nearly every ported rclcpp file need.
//
// W1.a  nested `SharedPtr` / `ConstSharedPtr` / `UniquePtr` on the entity
//       types and on `rclcpp::TimerBase`, so
//       `rclcpp::Publisher<T>::SharedPtr member_;` — close to universal in
//       real rclcpp source — declares.
// W1.c  `std::string` interop on `nros::FixedString<N>` / `nros::HeapString`,
//       so `message.data = "Hello, world! " + std::to_string(n);` compiles
//       against a codegen'd string field.
// W1.d  `now()` / `get_clock()` / `get_name()` / `get_namespace()` on the shim
//       `rclcpp::Node`, plus the `rclcpp::Time` / `Duration` / `Clock`
//       aliases.
//
// None of this is behaviour — RFC-0089 §"Who implements an adopted name"
// allows aliases, forwarders and copying conversions in the wrapper and
// nothing else. The header `-fsyntax-only` loop in `just check cpp` only
// PARSES templates, so a nested alias inside a class template is never
// checked there; this TU instantiates them.
//
// phase-417 stage 6 step A — this reaches the surface through `<nros/nros.hpp>`
// and no longer through `nros/rclcpp_compat.hpp`, which now declares nothing.
// The include is the POINT of the probe as much as the body is: the `rclcpp::`
// names are declared by the API headers themselves, so a probe that still went
// through the shim would pass whether or not the move had happened.
//
// Compiled HOSTED (no `-ffreestanding`): `std::shared_ptr` is in the public
// signature of every `rclcpp::Node::create_*`. The freestanding half of the
// contract — that those names VANISH rather than break the build where
// `<memory>` is absent — is covered by the header loop itself, which parses
// every header including `nros.hpp` with `-ffreestanding`.

#include <nros/nros.hpp>

// The string containers codegen emits for a message field. Neither is reachable
// from the `nros.hpp` umbrella today (a separate stage-2 gap: 15 of 46 headers
// are not), so name them directly.
#include <nros/fixed_string.hpp>
#include <nros/heap_string.hpp>

#include <memory>
#include <string>
#include <type_traits>

namespace nros_cpp_ros2_api_adoption_compile_test {

// Mirror of a codegen'd message with a FIXED-capacity string field
// (`mode = "fixed"`, the default) — cf. std_msgs/msg/String.
struct StringMsg {
    ::nros::FixedString<256> data;
    static const size_t SERIALIZED_SIZE_MAX = 512;
    static constexpr const char* TYPE_NAME = "std_msgs::msg::dds_::String_";
    static constexpr const char* TYPE_HASH = "RIHS01_string_stub";
    static int ffi_publish(void*, const void*) { return 0; }
    static int ffi_deserialize(const uint8_t*, size_t, void*) { return 0; }
    static int ffi_serialize(const void*, uint8_t*, size_t, size_t* out) {
        if (out) *out = 0;
        return 0;
    }
};

// The same message with a HEAP string field (`mode = "heap"`, RFC-0033).
struct HeapStringMsg {
    ::nros::HeapString data;
    static const size_t SERIALIZED_SIZE_MAX = 512;
    static constexpr const char* TYPE_NAME = "std_msgs::msg::dds_::String_";
    static constexpr const char* TYPE_HASH = "RIHS01_string_stub";
    static int ffi_publish(void*, const void*) { return 0; }
};

// --- W1.a + W1.d: the upstream node shape, written the upstream way ---------
//
// Every member declaration here is the spelling the ROS 2 tutorial uses. The
// point of the class is that NONE of them needs a `std::shared_ptr<…>`
// rewrite.
class PortedNode : public rclcpp::Node {
  public:
    PortedNode() : rclcpp::Node("ported_node") {
        publisher_ = this->create_publisher<StringMsg>("topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { tick(); });

        // W1.d — identity and clock, forwarded to `nros::Node`.
        const char* name = this->get_name();
        const char* ns = this->get_namespace();
        rclcpp::Time stamp = this->now();
        rclcpp::Clock* clock = this->get_clock();
        rclcpp::Time via_clock = clock->now();
        (void)name;
        (void)ns;
        (void)stamp;
        (void)via_clock;
    }

  private:
    void tick() {
        StringMsg message;
        // W1.c — the upstream line, verbatim. Before this stage the only
        // assignment `FixedString<N>` had was from `const char*`, so a ported
        // file had to insert a `.c_str()`.
        message.data = "Hello, world! " + std::to_string(count_++);

        // …and back out again, both spellings.
        const std::string round_trip = message.data;
        const std::string explicit_round_trip = message.data.to_string();
        bool eq = message.data == round_trip;
        bool ne = message.data != std::string("something else");
        bool eq_cstr = message.data == "Hello, world! 0";
        (void)round_trip;
        (void)explicit_round_trip;
        (void)eq;
        (void)ne;
        (void)eq_cstr;

        // Ungated `std::string`-shaped queries.
        size_t n = message.data.size();
        bool is_empty = message.data.empty();
        (void)n;
        (void)is_empty;

        publisher_->publish(message);
    }

    // W1.a — the nested-pointer spellings, as members.
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<StringMsg>::SharedPtr publisher_;
    size_t count_ = 0;
};

// --- W1.a: every entity type carries the three aliases ----------------------

struct StubService {
    struct Request {
        static const size_t SERIALIZED_SIZE_MAX = 16;
    };
    struct Response {
        static const size_t SERIALIZED_SIZE_MAX = 16;
    };
    static constexpr const char* TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_";
    static constexpr const char* TYPE_HASH = "RIHS01_srv_stub";
};

static_assert(std::is_same<::nros::Publisher<StringMsg>::SharedPtr,
                           std::shared_ptr<::nros::Publisher<StringMsg>>>::value,
              "Publisher<M>::SharedPtr must be std::shared_ptr<Publisher<M>>");
static_assert(std::is_same<::nros::Publisher<StringMsg>::ConstSharedPtr,
                           std::shared_ptr<const ::nros::Publisher<StringMsg>>>::value,
              "Publisher<M>::ConstSharedPtr must be std::shared_ptr<const Publisher<M>>");
static_assert(std::is_same<::nros::Publisher<StringMsg>::UniquePtr,
                           std::unique_ptr<::nros::Publisher<StringMsg>>>::value,
              "Publisher<M>::UniquePtr must be std::unique_ptr<Publisher<M>>");
static_assert(std::is_same<::nros::Subscription<StringMsg>::SharedPtr,
                           std::shared_ptr<::nros::Subscription<StringMsg>>>::value,
              "Subscription<M>::SharedPtr must be std::shared_ptr<Subscription<M>>");
static_assert(std::is_same<::nros::PollingSubscription<StringMsg>::SharedPtr,
                           std::shared_ptr<::nros::PollingSubscription<StringMsg>>>::value,
              "PollingSubscription<M>::SharedPtr must be std::shared_ptr<PollingSubscription<M>>");
static_assert(std::is_same<::nros::Service<StubService>::SharedPtr,
                           std::shared_ptr<::nros::Service<StubService>>>::value,
              "Service<S>::SharedPtr must be std::shared_ptr<Service<S>>");
static_assert(std::is_same<::nros::Client<StubService>::SharedPtr,
                           std::shared_ptr<::nros::Client<StubService>>>::value,
              "Client<S>::SharedPtr must be std::shared_ptr<Client<S>>");
static_assert(std::is_same<::nros::Timer::SharedPtr, std::shared_ptr<::nros::Timer>>::value,
              "Timer::SharedPtr must be std::shared_ptr<Timer>");
static_assert(std::is_same<rclcpp::TimerBase::SharedPtr, std::shared_ptr<rclcpp::TimerBase>>::value,
              "TimerBase::SharedPtr must be std::shared_ptr<TimerBase>");
static_assert(std::is_same<rclcpp::TimerBase::UniquePtr, std::unique_ptr<rclcpp::TimerBase>>::value,
              "TimerBase::UniquePtr must be std::unique_ptr<TimerBase>");

// The rclcpp alias templates hand the SAME nested names through.
static_assert(std::is_same<rclcpp::Publisher<StringMsg>::SharedPtr,
                           ::nros::Publisher<StringMsg>::SharedPtr>::value,
              "rclcpp::Publisher<M>::SharedPtr must resolve through the nros:: alias");
static_assert(std::is_same<rclcpp::Subscription<StringMsg>::SharedPtr,
                           ::nros::Subscription<StringMsg>::SharedPtr>::value,
              "rclcpp::Subscription<M>::SharedPtr must resolve through the nros:: alias");
static_assert(std::is_same<rclcpp::Service<StubService>::SharedPtr,
                           ::nros::Service<StubService>::SharedPtr>::value,
              "rclcpp::Service<S>::SharedPtr must resolve through the nros:: alias");
static_assert(std::is_same<rclcpp::Client<StubService>::SharedPtr,
                           ::nros::Client<StubService>::SharedPtr>::value,
              "rclcpp::Client<S>::SharedPtr must resolve through the nros:: alias");

// --- W1.d: the clock vocabulary is the nano-ros type, not a wrapper ---------

static_assert(std::is_same<rclcpp::Time, ::nros::Time>::value,
              "rclcpp::Time must BE nros::Time — a second type is a second contract");
static_assert(std::is_same<rclcpp::Duration, ::nros::Duration>::value,
              "rclcpp::Duration must BE nros::Duration");
static_assert(std::is_same<rclcpp::Clock, ::nros::Clock>::value,
              "rclcpp::Clock must BE nros::Clock");

// The shim's accessors have the shapes `nros::Node` has, since they forward.
static_assert(
    std::is_same<decltype(std::declval<const rclcpp::Node&>().get_name()), const char*>::value,
    "rclcpp::Node::get_name() must return const char*, as upstream does");
static_assert(
    std::is_same<decltype(std::declval<const rclcpp::Node&>().get_namespace()), const char*>::value,
    "rclcpp::Node::get_namespace() must return const char*, as upstream does");
static_assert(
    std::is_same<decltype(std::declval<const rclcpp::Node&>().now()), ::nros::Time>::value,
    "rclcpp::Node::now() must return nros::Time");
static_assert(
    std::is_same<decltype(std::declval<rclcpp::Node&>().get_clock()), ::nros::Clock*>::value,
    "rclcpp::Node::get_clock() must return a borrowed nros::Clock*");

// --- W1.c: HeapString carries the same conversions as FixedString -----------

inline void heap_string_round_trip(HeapStringMsg& msg) {
    msg.data = std::string("Hello, world! ") + std::to_string(1);
    msg.data = "plain C string";
    const std::string out = msg.data;
    const std::string explicit_out = msg.data.to_string();
    bool eq = msg.data == out;
    bool eq_cstr = msg.data == "plain C string";
    bool ne = msg.data != std::string("other");
    (void)out;
    (void)explicit_out;
    (void)eq;
    (void)eq_cstr;
    (void)ne;
}

// --- The layout the FFI depends on is unchanged (RFC-0033 / heap_string.hpp) -
//
// W1.c adds member FUNCTIONS only. A data member would silently break every
// generated message struct that embeds one of these.
static_assert(sizeof(::nros::FixedString<256>) == 256,
              "FixedString<N> must stay layout-identical to char[N]");
static_assert(sizeof(::nros::FixedString<8>) == 8,
              "FixedString<N> must stay layout-identical to char[N]");
static_assert(sizeof(::nros::HeapString) == sizeof(char*) + 2 * sizeof(size_t),
              "HeapString must stay { char* data; size_t size; size_t capacity; }");

// Instantiate the ported node's members so the bodies above are type-checked.
inline void instantiate() {
    ::nros::Publisher<StringMsg>::SharedPtr pub = std::make_shared<::nros::Publisher<StringMsg>>();
    rclcpp::TimerBase::SharedPtr timer;
    (void)pub;
    (void)timer;
}

} // namespace nros_cpp_ros2_api_adoption_compile_test
