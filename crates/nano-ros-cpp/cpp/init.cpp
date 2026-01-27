/// @file init.cpp
/// @brief Implementation of nano-ros C++ API
///
/// This file implements the C++ wrapper classes that use the cxx-generated
/// FFI bridge to call into the Rust nano-ros implementation.

#include "nano_ros/nano_ros.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

// Include the cxx runtime header first (provides rust::Error, rust::Box, etc.)
#include "rust/cxx.h"

// Include the cxx-generated bridge header
#include "nano-ros-cpp-bridge/src/lib.rs.h"

namespace nano_ros {

// ============================================================================
// Version functions
// ============================================================================

Version get_version() {
    auto v = ffi::get_version();
    return Version{v.major, v.minor, v.patch};
}

bool has_zenoh_support() {
    return ffi::has_zenoh_support();
}

// ============================================================================
// Duration implementation
// ============================================================================

Duration::Duration() = default;

Duration::Duration(int32_t seconds, uint32_t nanoseconds) : sec_(seconds), nanosec_(nanoseconds) {}

Duration::Duration(std::chrono::nanoseconds ns) {
    auto d = ffi::duration_from_nanoseconds(ns.count());
    sec_ = d.sec;
    nanosec_ = d.nanosec;
}

Duration Duration::from_seconds(double seconds) {
    auto d = ffi::duration_from_seconds(seconds);
    return Duration(d.sec, d.nanosec);
}

Duration Duration::from_nanoseconds(int64_t nanoseconds) {
    auto d = ffi::duration_from_nanoseconds(nanoseconds);
    return Duration(d.sec, d.nanosec);
}

Duration Duration::from_milliseconds(int64_t milliseconds) {
    auto d = ffi::duration_from_milliseconds(milliseconds);
    return Duration(d.sec, d.nanosec);
}

Duration Duration::zero() {
    return Duration(0, 0);
}

int64_t Duration::nanoseconds() const {
    ffi::Duration d{sec_, nanosec_};
    return ffi::duration_to_nanoseconds(d);
}

double Duration::seconds() const {
    ffi::Duration d{sec_, nanosec_};
    return ffi::duration_to_seconds(d);
}

std::chrono::nanoseconds Duration::to_chrono() const {
    return std::chrono::nanoseconds(nanoseconds());
}

bool Duration::operator==(const Duration& rhs) const {
    return sec_ == rhs.sec_ && nanosec_ == rhs.nanosec_;
}

bool Duration::operator!=(const Duration& rhs) const {
    return !(*this == rhs);
}

bool Duration::operator<(const Duration& rhs) const {
    return nanoseconds() < rhs.nanoseconds();
}

bool Duration::operator<=(const Duration& rhs) const {
    return nanoseconds() <= rhs.nanoseconds();
}

bool Duration::operator>(const Duration& rhs) const {
    return nanoseconds() > rhs.nanoseconds();
}

bool Duration::operator>=(const Duration& rhs) const {
    return nanoseconds() >= rhs.nanoseconds();
}

Duration Duration::operator+(const Duration& rhs) const {
    ffi::Duration a{sec_, nanosec_};
    ffi::Duration b{rhs.sec_, rhs.nanosec_};
    auto result = ffi::duration_add(a, b);
    return Duration(result.sec, result.nanosec);
}

Duration Duration::operator-(const Duration& rhs) const {
    ffi::Duration a{sec_, nanosec_};
    ffi::Duration b{rhs.sec_, rhs.nanosec_};
    auto result = ffi::duration_sub(a, b);
    return Duration(result.sec, result.nanosec);
}

Duration Duration::operator-() const {
    return Duration::from_nanoseconds(-nanoseconds());
}

Duration& Duration::operator+=(const Duration& rhs) {
    *this = *this + rhs;
    return *this;
}

Duration& Duration::operator-=(const Duration& rhs) {
    *this = *this - rhs;
    return *this;
}

// ============================================================================
// Time implementation
// ============================================================================

Time::Time() : sec_(0), nanosec_(0) {}

Time::Time(int32_t seconds, uint32_t nanoseconds) : sec_(seconds), nanosec_(nanoseconds) {}

Time Time::from_nanoseconds(int64_t nanoseconds) {
    auto t = ffi::time_from_nanoseconds(nanoseconds);
    return Time(t.sec, t.nanosec);
}

Time Time::zero() {
    return Time(0, 0);
}

int64_t Time::nanoseconds() const {
    ffi::Time t{sec_, nanosec_};
    return ffi::time_to_nanoseconds(t);
}

double Time::seconds() const {
    ffi::Time t{sec_, nanosec_};
    return ffi::time_to_seconds(t);
}

bool Time::operator==(const Time& rhs) const {
    return sec_ == rhs.sec_ && nanosec_ == rhs.nanosec_;
}

bool Time::operator!=(const Time& rhs) const {
    return !(*this == rhs);
}

bool Time::operator<(const Time& rhs) const {
    return nanoseconds() < rhs.nanoseconds();
}

bool Time::operator<=(const Time& rhs) const {
    return nanoseconds() <= rhs.nanoseconds();
}

bool Time::operator>(const Time& rhs) const {
    return nanoseconds() > rhs.nanoseconds();
}

bool Time::operator>=(const Time& rhs) const {
    return nanoseconds() >= rhs.nanoseconds();
}

Time Time::operator+(const Duration& rhs) const {
    ffi::Time t{sec_, nanosec_};
    ffi::Duration d{rhs.sec(), rhs.nanosec()};
    auto result = ffi::time_add_duration(t, d);
    return Time(result.sec, result.nanosec);
}

Time Time::operator-(const Duration& rhs) const {
    ffi::Time t{sec_, nanosec_};
    ffi::Duration d{rhs.sec(), rhs.nanosec()};
    auto result = ffi::time_sub_duration(t, d);
    return Time(result.sec, result.nanosec);
}

Duration Time::operator-(const Time& rhs) const {
    ffi::Time a{sec_, nanosec_};
    ffi::Time b{rhs.sec_, rhs.nanosec_};
    auto result = ffi::time_diff(a, b);
    return Duration(result.sec, result.nanosec);
}

Time& Time::operator+=(const Duration& rhs) {
    *this = *this + rhs;
    return *this;
}

Time& Time::operator-=(const Duration& rhs) {
    *this = *this - rhs;
    return *this;
}

// ============================================================================
// QoS implementation
// ============================================================================

QoS::QoS(size_t depth) : depth_(depth) {}

QoS QoS::default_qos() {
    return QoS(10);
}

QoS QoS::sensor_data() {
    return QoS(5).best_effort();
}

QoS QoS::services() {
    return QoS(10).reliable();
}

QoS QoS::parameters() {
    return QoS(1000).reliable();
}

QoS& QoS::history(HistoryPolicy policy) {
    history_ = policy;
    return *this;
}

QoS& QoS::keep_last(size_t depth) {
    history_ = HistoryPolicy::KeepLast;
    depth_ = depth;
    return *this;
}

QoS& QoS::keep_all() {
    history_ = HistoryPolicy::KeepAll;
    depth_ = 0;
    return *this;
}

QoS& QoS::reliability(ReliabilityPolicy policy) {
    reliability_ = policy;
    return *this;
}

QoS& QoS::reliable() {
    reliability_ = ReliabilityPolicy::Reliable;
    return *this;
}

QoS& QoS::best_effort() {
    reliability_ = ReliabilityPolicy::BestEffort;
    return *this;
}

QoS& QoS::durability(DurabilityPolicy policy) {
    durability_ = policy;
    return *this;
}

QoS& QoS::durability_volatile() {
    durability_ = DurabilityPolicy::Volatile;
    return *this;
}

QoS& QoS::transient_local() {
    durability_ = DurabilityPolicy::TransientLocal;
    return *this;
}

// ============================================================================
// Clock implementation
// ============================================================================

struct Clock::Impl {
    ::rust::Box<ffi::RustClock> rust_clock;

    explicit Impl(::rust::Box<ffi::RustClock> clock) : rust_clock(std::move(clock)) {}
};

Clock::Clock() : impl_(nullptr) {}

Clock::~Clock() = default;

Clock::Clock(Clock&&) noexcept = default;
Clock& Clock::operator=(Clock&&) noexcept = default;

std::shared_ptr<Clock> Clock::create_system() {
    auto clock = std::shared_ptr<Clock>(new Clock());
    clock->impl_ = std::make_unique<Impl>(ffi::create_clock_system());
    return clock;
}

std::shared_ptr<Clock> Clock::create_steady() {
    auto clock = std::shared_ptr<Clock>(new Clock());
    clock->impl_ = std::make_unique<Impl>(ffi::create_clock_steady());
    return clock;
}

std::shared_ptr<Clock> Clock::create_ros() {
    auto clock = std::shared_ptr<Clock>(new Clock());
    clock->impl_ = std::make_unique<Impl>(ffi::create_clock_ros());
    return clock;
}

ClockType Clock::get_clock_type() const {
    if (!impl_)
        return ClockType::SystemTime;
    auto kind = ffi::clock_get_type(*impl_->rust_clock);
    switch (kind) {
        case ffi::ClockKind::SystemTime:
            return ClockType::SystemTime;
        case ffi::ClockKind::SteadyTime:
            return ClockType::SteadyTime;
        case ffi::ClockKind::RosTime:
            return ClockType::RosTime;
        default:
            return ClockType::SystemTime;
    }
}

Time Clock::now() const {
    if (!impl_)
        return Time::zero();
    auto t = ffi::clock_now(*impl_->rust_clock);
    return Time(t.sec, t.nanosec);
}

// ============================================================================
// NodeOptions implementation
// ============================================================================

NodeOptions::NodeOptions() = default;

NodeOptions::NodeOptions(std::string name) : name_(std::move(name)) {}

NodeOptions& NodeOptions::node_name(const std::string& name) {
    name_ = name;
    return *this;
}

NodeOptions& NodeOptions::namespace_(const std::string& ns) {
    ns_ = ns;
    return *this;
}

NodeOptions& NodeOptions::use_clock_type(ClockType clock_type) {
    clock_type_ = clock_type;
    return *this;
}

NodeOptions& NodeOptions::use_intra_process_comms(bool enable) {
    use_intra_process_comms_ = enable;
    return *this;
}

NodeOptions& NodeOptions::start_parameter_services(bool enable) {
    start_parameter_services_ = enable;
    return *this;
}

// ============================================================================
// Context implementation
// ============================================================================

struct Context::Impl {
    ::rust::Box<ffi::RustContext> rust_ctx;

    explicit Impl(::rust::Box<ffi::RustContext> ctx) : rust_ctx(std::move(ctx)) {}
};

Context::Context() : impl_(nullptr) {}

Context::~Context() = default;

Context::Context(Context&&) noexcept = default;
Context& Context::operator=(Context&&) noexcept = default;

std::shared_ptr<Context> Context::create() {
    try {
        auto rust_ctx = ffi::create_context();
        auto ctx = std::shared_ptr<Context>(new Context());
        ctx->impl_ = std::make_unique<Impl>(std::move(rust_ctx));
        return ctx;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create context: ") + e.what());
    }
}

std::shared_ptr<Context> Context::from_env() {
    try {
        auto rust_ctx = ffi::create_context_from_env();
        auto ctx = std::shared_ptr<Context>(new Context());
        ctx->impl_ = std::make_unique<Impl>(std::move(rust_ctx));
        return ctx;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create context from env: ") + e.what());
    }
}

bool Context::ok() const {
    if (!impl_)
        return false;
    return ffi::context_ok(*impl_->rust_ctx);
}

uint32_t Context::domain_id() const {
    if (!impl_)
        return 0;
    return ffi::context_domain_id(*impl_->rust_ctx);
}

std::shared_ptr<Node> Context::create_node(const std::string& name) {
    if (!impl_) {
        throw std::runtime_error("Context is not initialized");
    }
    try {
        auto rust_node = ffi::create_node(*impl_->rust_ctx, name);
        auto node = std::shared_ptr<Node>(new Node());
        node->impl_ = std::make_unique<Node::Impl>(std::move(rust_node));
        return node;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create node: ") + e.what());
    }
}

std::shared_ptr<Node> Context::create_node(const std::string& name, const std::string& ns) {
    if (!impl_) {
        throw std::runtime_error("Context is not initialized");
    }
    try {
        auto rust_node = ffi::create_node_with_namespace(*impl_->rust_ctx, name, ns);
        auto node = std::shared_ptr<Node>(new Node());
        node->impl_ = std::make_unique<Node::Impl>(std::move(rust_node));
        return node;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create node: ") + e.what());
    }
}

std::shared_ptr<Node> Context::create_node(const NodeOptions& options) {
    const auto& ns = options.get_namespace();
    if (ns.empty()) {
        return create_node(options.get_node_name());
    }
    return create_node(options.get_node_name(), ns);
}

// ============================================================================
// Node implementation
// ============================================================================

struct Node::Impl {
    ::rust::Box<ffi::RustNode> rust_node;

    explicit Impl(::rust::Box<ffi::RustNode> node) : rust_node(std::move(node)) {}
};

Node::Node() : impl_(nullptr) {}

Node::~Node() = default;

Node::Node(Node&&) noexcept = default;
Node& Node::operator=(Node&&) noexcept = default;

std::string Node::get_name() const {
    if (!impl_)
        return "";
    return std::string(ffi::node_name(*impl_->rust_node));
}

std::string Node::get_namespace() const {
    if (!impl_)
        return "";
    return std::string(ffi::node_namespace(*impl_->rust_node));
}

std::string Node::get_fully_qualified_name() const {
    if (!impl_)
        return "";
    return std::string(ffi::node_fully_qualified_name(*impl_->rust_node));
}

std::shared_ptr<Clock> Node::get_clock() const {
    // For now, return a system clock
    // TODO: Store clock reference in Node when created with NodeOptions
    return Clock::create_system();
}

Time Node::now() const {
    if (!impl_)
        return Time::zero();
    auto t = ffi::node_now(*impl_->rust_node);
    return Time(t.sec, t.nanosec);
}

// ============================================================================
// Publisher implementation
// ============================================================================

struct Publisher::Impl {
    ::rust::Box<ffi::RustPublisher> rust_pub;

    explicit Impl(::rust::Box<ffi::RustPublisher> pub_) : rust_pub(std::move(pub_)) {}
};

Publisher::Publisher() : impl_(nullptr) {}

Publisher::~Publisher() = default;

Publisher::Publisher(Publisher&&) noexcept = default;
Publisher& Publisher::operator=(Publisher&&) noexcept = default;

void Publisher::publish_raw(const std::vector<uint8_t>& data) {
    if (!impl_) {
        throw std::runtime_error("Publisher is not initialized");
    }
    try {
        auto slice = ::rust::Slice<const uint8_t>(data.data(), data.size());
        ffi::publisher_publish(*impl_->rust_pub, slice);
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to publish: ") + e.what());
    }
}

void Publisher::publish_raw(const uint8_t* data, size_t size) {
    if (!impl_) {
        throw std::runtime_error("Publisher is not initialized");
    }
    try {
        auto slice = ::rust::Slice<const uint8_t>(data, size);
        ffi::publisher_publish(*impl_->rust_pub, slice);
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to publish: ") + e.what());
    }
}

std::string Publisher::get_topic_name() const {
    if (!impl_)
        return "";
    return std::string(ffi::publisher_topic_name(*impl_->rust_pub));
}

// ============================================================================
// Subscription implementation
// ============================================================================

struct Subscription::Impl {
    ::rust::Box<ffi::RustSubscriber> rust_sub;

    explicit Impl(::rust::Box<ffi::RustSubscriber> sub) : rust_sub(std::move(sub)) {}
};

Subscription::Subscription() : impl_(nullptr) {}

Subscription::~Subscription() = default;

Subscription::Subscription(Subscription&&) noexcept = default;
Subscription& Subscription::operator=(Subscription&&) noexcept = default;

std::vector<uint8_t> Subscription::take_raw() {
    if (!impl_) {
        throw std::runtime_error("Subscription is not initialized");
    }
    try {
        auto rust_vec = ffi::subscriber_take(*impl_->rust_sub);
        return std::vector<uint8_t>(rust_vec.begin(), rust_vec.end());
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to receive: ") + e.what());
    }
}

bool Subscription::has_message() {
    // Take raw returns empty vector if no message
    // This is a quick check implementation
    auto data = take_raw();
    // Note: This consumes the message if there was one.
    // A proper implementation would need a peek method in Rust.
    // For now, we just return false since this is a basic implementation.
    return !data.empty();
}

std::string Subscription::get_topic_name() const {
    if (!impl_)
        return "";
    return std::string(ffi::subscriber_topic_name(*impl_->rust_sub));
}

// ============================================================================
// Node publisher/subscription creation
// ============================================================================

namespace {
/// Convert C++ QoS to FFI QoSProfile
ffi::QoSProfile qos_to_ffi(const QoS& qos) {
    ffi::QoSProfile profile;
    profile.depth = static_cast<uint32_t>(qos.get_depth());

    switch (qos.get_history()) {
        case HistoryPolicy::KeepLast:
            profile.history = ffi::HistoryPolicy::KeepLast;
            break;
        case HistoryPolicy::KeepAll:
            profile.history = ffi::HistoryPolicy::KeepAll;
            break;
    }

    switch (qos.get_reliability()) {
        case ReliabilityPolicy::Reliable:
            profile.reliability = ffi::ReliabilityPolicy::Reliable;
            break;
        case ReliabilityPolicy::BestEffort:
            profile.reliability = ffi::ReliabilityPolicy::BestEffort;
            break;
    }

    switch (qos.get_durability()) {
        case DurabilityPolicy::Volatile:
            profile.durability = ffi::DurabilityPolicy::Volatile;
            break;
        case DurabilityPolicy::TransientLocal:
            profile.durability = ffi::DurabilityPolicy::TransientLocal;
            break;
    }

    return profile;
}
}  // namespace

std::shared_ptr<Publisher> Node::create_publisher(const std::string& topic, const QoS& qos) {
    if (!impl_) {
        throw std::runtime_error("Node is not initialized");
    }
    try {
        auto ffi_qos = qos_to_ffi(qos);
        auto rust_pub = ffi::create_publisher(*impl_->rust_node, topic, ffi_qos);
        auto pub = std::shared_ptr<Publisher>(new Publisher());
        pub->impl_ = std::make_unique<Publisher::Impl>(std::move(rust_pub));
        return pub;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create publisher: ") + e.what());
    }
}

std::shared_ptr<Subscription> Node::create_subscription(const std::string& topic, const QoS& qos) {
    if (!impl_) {
        throw std::runtime_error("Node is not initialized");
    }
    try {
        auto ffi_qos = qos_to_ffi(qos);
        auto rust_sub = ffi::create_subscriber(*impl_->rust_node, topic, ffi_qos);
        auto sub = std::shared_ptr<Subscription>(new Subscription());
        sub->impl_ = std::make_unique<Subscription::Impl>(std::move(rust_sub));
        return sub;
    } catch (const ::rust::Error& e) {
        throw std::runtime_error(std::string("Failed to create subscription: ") + e.what());
    }
}

// ============================================================================
// SingleThreadedExecutor implementation
// ============================================================================

struct SingleThreadedExecutor::Impl {
    std::vector<std::shared_ptr<Node>> nodes;
    std::atomic<bool> spinning{false};
    std::atomic<bool> cancelled{false};
    std::mutex mutex;
    std::condition_variable cv;

    Impl() = default;
};

SingleThreadedExecutor::SingleThreadedExecutor() : impl_(std::make_unique<Impl>()) {}

SingleThreadedExecutor::SingleThreadedExecutor(const ExecutorOptions& /* options */)
    : impl_(std::make_unique<Impl>()) {
    // Options.context is currently unused - the executor manages nodes directly
}

SingleThreadedExecutor::~SingleThreadedExecutor() {
    cancel();
}

SingleThreadedExecutor::SingleThreadedExecutor(SingleThreadedExecutor&&) noexcept = default;
SingleThreadedExecutor& SingleThreadedExecutor::operator=(SingleThreadedExecutor&&) noexcept =
    default;

void SingleThreadedExecutor::add_node(std::shared_ptr<Node> node) {
    if (!impl_) {
        throw std::runtime_error("Executor is not initialized");
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->nodes.push_back(node);
}

void SingleThreadedExecutor::remove_node(std::shared_ptr<Node> node) {
    if (!impl_) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto it = std::find(impl_->nodes.begin(), impl_->nodes.end(), node);
    if (it != impl_->nodes.end()) {
        impl_->nodes.erase(it);
    }
}

void SingleThreadedExecutor::spin() {
    if (!impl_) {
        throw std::runtime_error("Executor is not initialized");
    }

    impl_->spinning.store(true, std::memory_order_release);
    impl_->cancelled.store(false, std::memory_order_release);

    // Block until cancel() is called
    // Note: Since C++ subscriptions use polling (take()), the executor
    // doesn't process callbacks. Users should call take() on subscriptions
    // from another thread or use spin_once() in a loop.
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->cv.wait(lock, [this] { return impl_->cancelled.load(std::memory_order_acquire); });

    impl_->spinning.store(false, std::memory_order_release);
}

uint32_t SingleThreadedExecutor::spin_once(int64_t timeout_ns) {
    if (!impl_) {
        throw std::runtime_error("Executor is not initialized");
    }

    // Note: Since C++ subscriptions use polling (take()), there are no
    // callbacks to process. This method waits for the timeout and returns.
    // Users should call subscription->take() to receive messages.

    if (timeout_ns > 0) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(timeout_ns));
    } else if (timeout_ns < 0) {
        // Infinite wait - but we can't block forever without cancel support
        // Sleep for a short period and return
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // timeout_ns == 0: non-blocking, return immediately

    return 0;  // No callbacks processed (polling-based subscriptions)
}

uint32_t SingleThreadedExecutor::spin_some(int64_t /* max_duration_ns */) {
    // Non-blocking check for work - with polling subscriptions, this is a no-op
    return 0;
}

void SingleThreadedExecutor::cancel() {
    if (impl_) {
        impl_->cancelled.store(true, std::memory_order_release);
        impl_->cv.notify_all();
    }
}

bool SingleThreadedExecutor::is_spinning() const {
    if (!impl_) {
        return false;
    }
    return impl_->spinning.load(std::memory_order_acquire);
}

size_t SingleThreadedExecutor::node_count() const {
    if (!impl_) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->nodes.size();
}

// ============================================================================
// PollingExecutor implementation
// ============================================================================

struct PollingExecutor::Impl {
    std::vector<std::shared_ptr<Node>> nodes;

    Impl() = default;
};

PollingExecutor::PollingExecutor() : impl_(std::make_unique<Impl>()) {}

PollingExecutor::PollingExecutor(const ExecutorOptions& /* options */)
    : impl_(std::make_unique<Impl>()) {
    // Options.context is currently unused - the executor manages nodes directly
}

PollingExecutor::~PollingExecutor() = default;

PollingExecutor::PollingExecutor(PollingExecutor&&) noexcept = default;
PollingExecutor& PollingExecutor::operator=(PollingExecutor&&) noexcept = default;

void PollingExecutor::add_node(std::shared_ptr<Node> node) {
    if (!impl_) {
        throw std::runtime_error("Executor is not initialized");
    }
    impl_->nodes.push_back(node);
}

void PollingExecutor::remove_node(std::shared_ptr<Node> node) {
    if (!impl_) {
        return;
    }
    auto it = std::find(impl_->nodes.begin(), impl_->nodes.end(), node);
    if (it != impl_->nodes.end()) {
        impl_->nodes.erase(it);
    }
}

uint32_t PollingExecutor::spin_once(uint32_t /* delta_ms */) {
    if (!impl_) {
        throw std::runtime_error("Executor is not initialized");
    }
    // Note: Since C++ subscriptions use polling (take()), there are no
    // callbacks to process here. The delta_ms parameter would be used for
    // timer processing, which is not yet implemented in the C++ bindings.
    // Users should call subscription->take() to receive messages.
    return 0;
}

size_t PollingExecutor::node_count() const {
    if (!impl_) {
        return 0;
    }
    return impl_->nodes.size();
}

// ============================================================================
// Convenience functions
// ============================================================================

void spin(std::shared_ptr<Node> node) {
    SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}

void spin_some(std::shared_ptr<Node> node) {
    SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_some(0);
}

}  // namespace nano_ros
