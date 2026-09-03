// phase-417 stage 2b (RFC-0087) — the graph surface must be reachable on
// `nros::Node` and `nros::LifecycleNode`, not only on `nros::Executor`.
//
// Why this probe exists: rclcpp puts the graph calls on the NODE, and 18
// ledger rows were open purely because ours were only on the executor. The
// capability shipped the whole time (`executor.hpp:205-301` over FFI that
// exists), so the thing that can regress is not the behaviour — it is whether
// a ported file can still REACH it where upstream put it. That is a
// compile-time question, so it gets a compile-time answer.
//
// Method POINTERS, as `graph_query_entry_points.cpp` does for the executor: a
// missing method, a signature that drifted, or a forwarder that quietly lost
// its `const` fails to compile here rather than at a user's build. There is no
// session to query, and none is needed.
//
// The forwarders are `const`, matching rclcpp's `Node::get_node_names() const`
// — taking the pointers through a `const`-qualified member type is what pins
// that, since a ported file often holds a `const Node &`.
//
// `-fno-exceptions -fno-rtti -std=c++14`, matching the embedded build (issue
// 0112): these headers compile `-nostdinc++` against Zephyr's minimal libcpp,
// which is why the visitors are plain function pointers plus `void *ctx` and
// `TopicEndpointInfo` carries no `std::string`.

#include <nros/lifecycle.hpp>
#include <nros/node.hpp>

#include <stddef.h>
#include <stdint.h>

namespace {

bool visit_node(void* ctx, const char* name, const char* ns, const char* enclave) {
    (void)ctx;
    (void)name;
    (void)ns;
    // nullptr is the contract's "this backend tracks no enclave".
    return enclave == nullptr;
}

bool visit_names_and_types(void* ctx, const char* name, const char* const* types,
                           size_t types_count) {
    (void)ctx;
    (void)name;
    (void)types;
    // 0 is legal on a partially discovered graph.
    return types_count == types_count;
}

bool visit_endpoint_ffi(void* ctx, const nros_cpp_endpoint_info_t* info) {
    (void)ctx;
    return info != nullptr;
}

// The typed visitor — the rclcpp-shaped half of stage 2b. Every accessor a
// ported file reaches for is named here, so dropping one is a build failure.
bool visit_endpoint_typed(void* ctx, const nros::TopicEndpointInfo& info) {
    (void)ctx;
    const char* n = info.node_name();
    const char* ns = info.node_namespace();
    const char* ty = info.topic_type();
    (void)n;
    (void)ns;
    (void)ty;

    // The GID is held BY VALUE (24 bytes), so it outlives the visit even
    // though the three strings above do not. Pointer-plus-constant, not an
    // `nros::Span` — see `graph.hpp`.
    const uint8_t* gid = info.endpoint_gid();
    if (gid == nullptr) return false;
    if (nros::TopicEndpointInfo::endpoint_gid_size() != nros::kEndpointGidSize) return false;
    uint8_t first = gid[0];
    (void)first;

    // rclcpp's `EndpointType`, upstream `rmw_endpoint_type_t` values.
    if (info.endpoint_type() == nros::EndpointType::Invalid) return false;
    if (info.endpoint_type() != nros::EndpointType::Publisher &&
        info.endpoint_type() != nros::EndpointType::Subscription) {
        return false;
    }

    // No `qos_profile()`, deliberately: no backend can read a remote's GRANTED
    // profile, and reporting the DECLARED one would be a confident wrong
    // answer. A ported file that asks for it must fail to compile rather than
    // read a QoS nobody granted (RFC-0087 — never compile and differ). Adding
    // the accessor makes the next line stop being true, which is the point of
    // writing it down here.
    return info.is_valid();
}

// A default-constructed endpoint is `Invalid` and has no strings — it must be
// constructible without a session, or it is not a value type.
bool default_endpoint_is_invalid() {
    nros::TopicEndpointInfo empty;
    return !empty.is_valid() && empty.endpoint_type() == nros::EndpointType::Invalid &&
           empty.node_name()[0] == '\0' && empty.node_namespace()[0] == '\0' &&
           empty.topic_type()[0] == '\0';
}

} // namespace

// The forwarders must exist on BOTH node types with the SAME shapes, so the
// checks are written once against a template parameter: a signature that
// drifts on one and not the other stops instantiating.
template <typename N> void graph_surface_exists() {
    using nros::Result;
    using nros::TopicEndpointInfoVisitFn;

    Result (N::*p_node_names)(nros_cpp_node_visit_fn, void*) const = &N::get_node_names;
    Result (N::*p_topics)(nros_cpp_names_and_types_visit_fn, void*) const =
        &N::get_topic_names_and_types;
    Result (N::*p_services)(nros_cpp_names_and_types_visit_fn, void*) const =
        &N::get_service_names_and_types;
    Result (N::*p_count_pub)(const char*, size_t*) const = &N::count_publishers;
    Result (N::*p_count_sub)(const char*, size_t*) const = &N::count_subscribers;

    // `get_subscription_names_and_types_by_node`, NOT `subscriber`: the C++
    // surface takes rclcpp's vocabulary and `nros::Executor` already spells it
    // this way. If someone "aligns" the three languages, this line stops
    // compiling.
    Result (N::*p_pub_by_node)(const char*, const char*, nros_cpp_names_and_types_visit_fn, void*)
        const = &N::get_publisher_names_and_types_by_node;
    Result (N::*p_sub_by_node)(const char*, const char*, nros_cpp_names_and_types_visit_fn, void*)
        const = &N::get_subscription_names_and_types_by_node;
    Result (N::*p_srv_by_node)(const char*, const char*, nros_cpp_names_and_types_visit_fn, void*)
        const = &N::get_service_names_and_types_by_node;
    Result (N::*p_cli_by_node)(const char*, const char*, nros_cpp_names_and_types_visit_fn, void*)
        const = &N::get_client_names_and_types_by_node;

    // Both endpoint-info overloads: the raw C seam and the rclcpp-shaped one.
    // Taking both as pointers also pins that the overload set is unambiguous.
    Result (N::*p_pubs_info)(const char*, nros_cpp_endpoint_info_visit_fn, void*) const =
        &N::get_publishers_info_by_topic;
    Result (N::*p_subs_info)(const char*, nros_cpp_endpoint_info_visit_fn, void*) const =
        &N::get_subscriptions_info_by_topic;
    Result (N::*p_pubs_info_typed)(const char*, TopicEndpointInfoVisitFn, void*) const =
        &N::get_publishers_info_by_topic;
    Result (N::*p_subs_info_typed)(const char*, TopicEndpointInfoVisitFn, void*) const =
        &N::get_subscriptions_info_by_topic;

    (void)p_node_names;
    (void)p_topics;
    (void)p_services;
    (void)p_count_pub;
    (void)p_count_sub;
    (void)p_pub_by_node;
    (void)p_sub_by_node;
    (void)p_srv_by_node;
    (void)p_cli_by_node;
    (void)p_pubs_info;
    (void)p_subs_info;
    (void)p_pubs_info_typed;
    (void)p_subs_info_typed;
}

// An uninitialized node answers every graph call without touching the wire, so
// the forwards can also be CALLED here — which type-checks the bodies, not
// just the declarations. `Node` reports `NotInitialized`; an unbound
// `LifecycleNode` reports `InvalidArgument` from the FFI. Neither runs.
template <typename N> bool graph_surface_calls(N& node) {
    size_t count = 0;
    bool ok = true;
    ok = ok && !node.get_node_names(visit_node, nullptr);
    ok = ok && !node.get_topic_names_and_types(visit_names_and_types, nullptr);
    ok = ok && !node.get_service_names_and_types(visit_names_and_types, nullptr);
    ok = ok && !node.count_publishers("/chatter", &count);
    ok = ok && !node.count_subscribers("/chatter", &count);
    ok = ok &&
         !node.get_publisher_names_and_types_by_node("talker", "/", visit_names_and_types, nullptr);
    ok = ok && !node.get_subscription_names_and_types_by_node("talker", "/", visit_names_and_types,
                                                              nullptr);
    ok = ok &&
         !node.get_service_names_and_types_by_node("talker", "/", visit_names_and_types, nullptr);
    ok = ok &&
         !node.get_client_names_and_types_by_node("talker", "/", visit_names_and_types, nullptr);
    ok = ok && !node.get_publishers_info_by_topic("/chatter", visit_endpoint_ffi, nullptr);
    ok = ok && !node.get_subscriptions_info_by_topic("/chatter", visit_endpoint_ffi, nullptr);
    // The typed overloads; `visit_endpoint_typed` is a
    // `nros::TopicEndpointInfoVisitFn`, so this is also the overload-resolution
    // check.
    ok = ok && !node.get_publishers_info_by_topic("/chatter", visit_endpoint_typed, nullptr);
    ok = ok && !node.get_subscriptions_info_by_topic("/chatter", visit_endpoint_typed, nullptr);
    return ok;
}

int main() {
    graph_surface_exists<nros::Node>();
    graph_surface_exists<nros::LifecycleNode>();

    nros::Node node;
    nros::LifecycleNode lifecycle;

    // `const` reachability: a ported file often holds a `const Node &`.
    const nros::Node& const_node = node;
    const nros::LifecycleNode& const_lifecycle = lifecycle;

    bool ok = graph_surface_calls(const_node);
    ok = ok && graph_surface_calls(const_lifecycle);
    ok = ok && default_endpoint_is_invalid();

    // Never true (the calls above all fail on an unopened session); the point
    // is that every line COMPILED.
    return ok ? 1 : 0;
}
