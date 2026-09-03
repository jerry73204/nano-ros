// nros-cpp: graph value types (phase-417 stage 2b, RFC-0087)
// Freestanding C++14 — no exceptions, no RTTI, no STL, no allocation.
//
// The graph QUERIES live on `nros::Executor` (`executor.hpp:205-301`) because
// one session per image makes the executor the graph's receiver (RFC-0002);
// `nros::Node` and `nros::LifecycleNode` forward to it so a ported rclcpp file
// reaches them where rclcpp puts them, on the node. This header holds the one
// thing those calls need that the FFI does not already give C++ a name for: a
// value type for a discovered endpoint.
//
// RFC-0019: the Rust API is the implementation source of truth. Nothing here
// implements graph behaviour — `TopicEndpointInfo` copies an FFI struct and
// renames its fields to rclcpp's accessors, and the trampoline converts and
// delegates. Both are ergonomics, which RFC-0087 §"Who implements an adopted
// name" allows in the wrapper; behaviour is not.

/**
 * @file graph.hpp
 * @ingroup grp_support
 * @brief `nros::TopicEndpointInfo` — one discovered endpoint on a topic.
 */

#ifndef NROS_CPP_GRAPH_HPP
#define NROS_CPP_GRAPH_HPP

#include <cstddef>
#include <cstdint>

#include "nros_cpp_ffi.h"

namespace nros {

/// Bytes in an endpoint GID — upstream's `RMW_GID_STORAGE_SIZE`.
constexpr size_t kEndpointGidSize = 24;

/// Which end of a topic an endpoint is — rclcpp's `rclcpp::EndpointType`,
/// values from upstream `rmw_endpoint_type_t` so a value crossing either seam
/// means the same thing.
enum class EndpointType : uint8_t {
    /// No endpoint — what a default-constructed [`TopicEndpointInfo`] reports.
    Invalid = 0,
    Publisher = 1,
    Subscription = 2,
};

/// One discovered endpoint on a topic — rclcpp's `rclcpp::TopicEndpointInfo`.
///
/// Handed to the visitor of `get_publishers_info_by_topic` /
/// `get_subscriptions_info_by_topic`. rclcpp returns a
/// `std::vector<TopicEndpointInfo>`; there is no allocator here (RFC-0022), so
/// the same information streams one endpoint at a time and this is the element.
///
/// **Copyable, but the three strings BORROW the backend's own storage for the
/// duration of the visit.** The GID is held by value and survives a copy; a
/// caller that needs a name afterwards copies it into its own
/// `nros::FixedString<N>`. That borrow is what lets the graph stream without an
/// allocator, and it mirrors `nros_rmw::GraphEndpointInfo` on the Rust side.
///
/// **No QoS profile, deliberately.** rclcpp's `qos_profile()` reports the
/// GRANTED profile, which is the answer to "why is nothing arriving" — and no
/// backend behind this API can read a remote's granted profile back. zenoh's
/// liveliness token carries the DECLARING side's own request, not a negotiated
/// grant, so reporting it would be the plausible wrong answer the omission
/// exists to avoid. The accessor is absent rather than present-and-empty: a
/// ported file that asks for it fails to COMPILE instead of reading a QoS
/// nobody granted (RFC-0087's rule — never compile and differ). Cyclone can
/// read a real grant and will need one; adding it then is additive.
class TopicEndpointInfo {
  public:
    /// An empty endpoint: no strings, a zero GID, `EndpointType::Invalid`.
    TopicEndpointInfo() : info_(), valid_(false) {}

    /// Wrap one endpoint handed over by the FFI visitor. The strings are
    /// borrowed from `info`, so this must not outlive the visit.
    explicit TopicEndpointInfo(const nros_cpp_endpoint_info_t& info) : info_(info), valid_(true) {}

    /// Name of the node that owns the endpoint. `""` when unset, never null.
    const char* node_name() const { return info_.node_name != nullptr ? info_.node_name : ""; }

    /// That node's namespace. `""` when unset, never null.
    const char* node_namespace() const {
        return info_.node_namespace != nullptr ? info_.node_namespace : "";
    }

    /// Fully-qualified type on the wire, e.g. `"std_msgs/msg/Int32"`. `""`
    /// when unset, never null.
    const char* topic_type() const { return info_.topic_type != nullptr ? info_.topic_type : ""; }

    /// Publisher or subscription — `EndpointType::Invalid` on a
    /// default-constructed value.
    EndpointType endpoint_type() const {
        if (!valid_) return EndpointType::Invalid;
        return info_.is_publisher ? EndpointType::Publisher : EndpointType::Subscription;
    }

    /// True when this value came from a visit rather than from the default
    /// constructor.
    bool is_valid() const { return valid_; }

    /// The endpoint's identity, [`kEndpointGidSize`] bytes; all-zero when the
    /// backend tracks none. Never null.
    ///
    /// rclcpp returns a `std::array<uint8_t, RMW_GID_STORAGE_SIZE> &`. The
    /// pointer-plus-constant pair is the freestanding spelling of the same
    /// thing, and deliberately NOT an `nros::Span`: `span.hpp` is reached by
    /// nothing else in these headers, so taking it here would newly expose
    /// `Span` / `StringView` / `LeSpan` on the public C++ surface as a side
    /// effect of a graph forwarder. That is a decision for whoever classifies
    /// those types, not a consequence of this one.
    ///
    /// The bytes live in THIS object, so they are valid for as long as this
    /// value is — unlike the three strings, they survive a copy and outlive
    /// the visit.
    const uint8_t* endpoint_gid() const { return info_.endpoint_gid; }

    /// Bytes readable through [`endpoint_gid`] — always [`kEndpointGidSize`].
    static constexpr size_t endpoint_gid_size() { return kEndpointGidSize; }

    /// The raw FFI struct, for a caller already speaking the C seam.
    const nros_cpp_endpoint_info_t& ffi() const { return info_; }

  private:
    nros_cpp_endpoint_info_t info_;
    bool valid_;
};

// The GID is copied BY VALUE out of the FFI struct, so its width must match the
// seam exactly — a wider view here would read past the struct.
static_assert(sizeof(nros_cpp_endpoint_info_t::endpoint_gid) == kEndpointGidSize,
              "nros_cpp_endpoint_info_t::endpoint_gid width diverged from kEndpointGidSize");

/// Visitor for the typed `get_{publishers,subscriptions}_info_by_topic`
/// overloads: `visit(ctx, info)` once per discovered endpoint, `false` to stop.
///
/// A plain function pointer plus an opaque `ctx`, not a `std::function`: these
/// headers compile `-nostdinc++` against Zephyr's minimal libcpp, where
/// `<functional>` does not exist (issue 0112), and a `std::function` is a heap
/// allocation per registration.
using TopicEndpointInfoVisitFn = bool (*)(void* ctx, const TopicEndpointInfo& info);

namespace detail {

/// Adapter that lets a [`TopicEndpointInfoVisitFn`] be passed to the C seam.
///
/// Pure conversion: it wraps the FFI struct and calls through. It holds the
/// caller's function and context for the duration of ONE query — nothing is
/// cached and no state survives the call.
struct EndpointInfoTrampoline {
    TopicEndpointInfoVisitFn visit;
    void* ctx;

    /// `nros_cpp_endpoint_info_visit_fn`-shaped entry point; `self` is the
    /// address of the trampoline.
    static bool thunk(void* self, const nros_cpp_endpoint_info_t* info) {
        EndpointInfoTrampoline* t = static_cast<EndpointInfoTrampoline*>(self);
        if (t == nullptr || t->visit == nullptr || info == nullptr) return false;
        return t->visit(t->ctx, TopicEndpointInfo(*info));
    }
};

} // namespace detail

} // namespace nros

#endif // NROS_CPP_GRAPH_HPP
