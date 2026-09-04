# Differences from standard ROS 2

Coming from `rclcpp`, `rclrs`, or `rclc`? This page calls out where
nano-ros looks the same, where it diverges, and the reason behind each
choice. It is an *orientation* page — the per-language API references
([Rust](../reference/rust-api.md) / [C](../reference/c-api.md) /
[C++](../reference/cpp-api.md)) cover the surface itself.

The short version: nano-ros keeps ROS 2's vocabulary (Node, Publisher,
Subscription, Service, Client, ActionServer, ActionClient, Timer,
Executor, QoS, message codegen) so existing nodes port cleanly. The
trade-offs that come from running on a Cortex-M3 with 64 KB of heap
shape every place where the surface diverges.

## Same vocabulary, same wire format

- ROS 2 entity model unchanged. A node owns publishers, subscriptions,
  services, clients, action servers, action clients, timers, and
  parameters.
- Topic / service / action names follow ROS 2 conventions
  (`/talker_node/chatter`, `/add_two_ints`, `/fibonacci`).
- Message types stay rosidl-shaped (`std_msgs/msg/Int32`,
  `geometry_msgs/msg/Twist`, …). CDR encoding on the wire.
- Default backend (`nros-rmw-zenoh`) is bit-compatible with the upstream
  [`rmw_zenoh`](https://github.com/ros2/rmw_zenoh) ROS 2 RMW. A
  nano-ros publisher and an `rclcpp` subscriber on the same zenohd
  router exchange messages without a bridge — see
  [ROS 2 Interoperability](../getting-started/ros2-interop.md).

## Where it diverges, and why

### 1. The executor is the entry point

Standard ROS 2 starts with a global init (`rclcpp::init` /
`rclrs::Context::default_from_env` / `rclc_support_init`) followed by
node creation, then optionally an executor.

nano-ros inverts this: an `Executor` opens the RMW session and owns the
runtime budget. Every node, publisher, subscription, service, client,
action handle, and timer is allocated *out of the executor's arena*.

```text
Executor::open(&config) → Node → Publisher / Subscription / Service / Client / Timer
```

**Why.** The arena is fixed-size and known at compile time
(`NROS_EXECUTOR_ARENA_SIZE` / `NROS_EXECUTOR_MAX_CBS`). On a 64 KB
heap MCU we cannot afford the indirection of a global allocator behind
every `create_publisher` call. The executor-as-arena pattern moves the
size negotiation up to the application's startup code, where it
belongs.

### 2. Both manual-poll and callback paths are first-class

`rclcpp` is callback-only — every subscription needs a callback, the
executor dispatches them. `rclrs` <0.7 was manual-poll only. `rclc`
exposes both but treats manual-poll as the second-class path.

nano-ros treats them as equals. A subscription created via
`node.create_subscription(...)` exposes `take()`. A subscription
registered via `executor.node_mut(nid).create_subscription::<M, _>("/topic", callback)`
runs the callback during `spin_once`. Pick whichever fits the control loop.

**Why.** Embedded apps often want the predictability of an explicit
poll inside a real-time loop. Callback dispatch is great for
event-driven services but adds indirection that real-time engineers
have to bound by hand. Offering both means the application picks.

### 3. Async pub/sub/service/action

`rclcpp` has no real `async/await`. `rclrs` 0.7+ added async but it
sits next to the synchronous executor, not in it. `rclc` has none.

nano-ros has it as a first-class path: `Executor::spin_async()` wakes
on RMW I/O, `Subscription::recv().await`, `Client::call().await`,
`ActionClient::send_goal().await`, etc. Runs on tokio (POSIX),
Embassy (FreeRTOS / RTIC), or any external `Future`-driver.

**Why.** Robotic control loops are stitched together from many
concurrent waits — sensor data, service replies, action feedback,
parameter updates. Async lets you write them as one straight-line
function instead of hand-rolling state machines on top of a
poll-based executor.

### 4. Heap is optional

ROS 2 RMW implementations all assume `std` + a heap. Even rclrs's
`no_std` story is "soon".

nano-ros runs in three modes that map onto target capability:

| Mode | Cargo features | What works |
|------|----------------|------------|
| `std` | `std` (default on POSIX) | Everything. POSIX threading, full async runtime. |
| `no_std + alloc` | `alloc` + a `#[global_allocator]` | Everything except features that need `std::sync::Mutex`. Used by FreeRTOS / NuttX / ThreadX / Zephyr / ESP32. |
| `no_std` cooperative | no `std`/`alloc` platform threading (bare-metal board crates, RTIC apps) | Cooperative single-task — no threading at all. Used by bare-metal MPS2-AN385, single-core RTIC. |

**Why.** Heap presence is not a binary "embedded yes/no" — it is a
spectrum. Stm32-class boards have a heap; Cortex-M0+-class might not.
The feature axis lets the same application code target both.

### 5. Backend selection at compile time, not runtime

Standard ROS 2 uses an `RMW_IMPLEMENTATION` env var read at process
start. The plugin loader pulls a shared library, dispatches calls
through C function pointers.

nano-ros bakes the backend in at compile time. The consuming
`Cargo.toml` adds the backend crate directly (`nros-rmw-zenoh` /
`nros-rmw-cyclonedds` / `nros-rmw-xrce-cffi`) alongside `nros` with the
`rmw-cffi` feature; CMake options (`-DNANO_ROS_RMW=zenoh`) decide it
for C/C++ builds. The backend's `#[ctor]` registers its vtable with
the `nros-rmw-cffi` runtime registry before `main`.

**Why.**

- **Dead-code elimination.** A 32 KB Flash budget cannot afford to
  link every backend's C client and pick at runtime. Linking only the
  selected backend cuts the binary substantially (the three backends
  differ by an order of magnitude in linked C footprint — measure your
  own image with `size`; no benchmarked figure exists).
- **No plugin loader.** Most embedded targets have no `dlopen`. The
  cost of the plugin abstraction is a permanent overhead with no
  payoff there.
- **Cross-compile sanity.** `RMW_IMPLEMENTATION` baked into the binary
  means the build system already knows which backend's C client to
  link — no separate "find shared library at runtime" step.

The trade-off is real: changing backends requires a rebuild. This is
the right trade-off for the embedded use case; it would be the wrong
trade-off for desktop ROS 2.

A binary can still link **multiple** backends and run them in
parallel — one node on backend A, another on backend B,
the executor draining both each tick. The
[Cross-backend Bridges](../user-guide/cross-backend-bridges.md)
chapter walks the build knobs (`NROS_RMW=...` primary
selector, explicit per-backend `register()`, `NANO_ROS_RMW=
none` cmake escape hatch) and the three shipped examples.

### 6. Message codegen lands inside your build, not a sibling library

Standard ROS 2 uses `ament` + `rosidl` to compile message packages
(`std_msgs`, `geometry_msgs`, …) into separate shared libraries that
your application links against.

nano-ros's `nros generate-rust` (Rust) and
`nano_ros_generate_interfaces()` (C / C++ via CMake — one of five
entry points; see the [spelling table](../user-guide/message-generation.md#which-cmake-spelling-one-table)) write message
type definitions *into your build tree*. No `_msgs` library, no ament
overlay, no colcon workspace required.

**Why.** Embedded cross-builds without a hosted ROS 2 install need to
generate message types from `package.xml` + `.msg` files alone. The
codegen tool ships its own bundled rosidl-flavoured `.msg` set inside
the `nros` CLI (built in-tree from `packages/cli/`, Phase 218), so you
don't even need the upstream message packages on disk.

### 7. QoS profile is the full DDS field set; backends advertise per-policy support

Standard ROS 2 supports the full DDS QoS profile family
(`reliability`, `durability`, `history`, `depth`, `deadline`,
`lifespan`, `liveliness`, `liveliness_lease_duration`,
`avoid_ros_namespace_conventions`) and performs profile *matching*
between endpoints.

nano-ros's `rmw_qos_profile_t` carries the same field set; standard
profile constants (`NROS_RMW_QOS_PROFILE_DEFAULT`, `_SENSOR_DATA`,
`_SERVICES_DEFAULT`, `_PARAMETERS`, `_SYSTEM_DEFAULT`) match
upstream `rmw_qos_profile_*` field-for-field. ROS 2 apps porting
across pull the equivalent constant unchanged.

Each backend advertises which policies it can enforce via
`Session::supported_qos_policies()`. The runtime validates the
requested QoS at entity-create time and returns
`IncompatibleQos` synchronously when the backend can't honour
a requested policy:

```rust
if session.supported_qos_policies().contains(QoSPolicyMask::DEADLINE) {
    // backend honours deadline; safe to set deadline_ms
} else {
    // app handles deadline monitoring itself
}
```

**No silent downgrade.** The runtime never quietly drops a requested
policy. Apps either get the QoS they asked for or a hard error.

**Why upstream-shaped struct, not a smaller subset.** ROS 2 QoS is
the established vocabulary; mismatched APIs make porting painful.
The field set is small (24 bytes); apps that don't request a policy
leave its field at zero ("off"). Per-backend implementation is a
separate question — which policies actually fire — answered by
the support mask.

**Why synchronous error instead of runtime event.** Upstream's
`RMW_EVENT_REQUESTED_INCOMPATIBLE_QOS` event surfaces mismatches
at run time. Most QoS mismatches are configuration errors visible
at startup; the runtime path doesn't need to handle them. The few
that aren't (cross-process QoS-mismatched discovery) the wire
protocol handles itself — DDS endpoints negotiate via DDS Discovery,
zenoh endpoints communicate intent through the topic-key encoding.

**Manual liveliness assertion.** Publishers configured with
`MANUAL_BY_TOPIC` / `MANUAL_BY_NODE` liveliness call
`assert_liveliness()` explicitly to refresh the lease. Available on
every language surface (Rust `Publisher<M>::assert_liveliness()`, C
`rcl_publisher_assert_liveliness(&pub)`, C++
`pub.assert_liveliness()`). The zenoh backend wires it: a shim-side
keepalive lease that `assert_liveliness()` refreshes, with
`LivelinessLost` fired on expiry. Backends without manual-assertion
wiring (xrce, cyclonedds) treat the call as a no-op. See [Status events](status-events.md) for the runtime-event
side of liveliness, deadline, and message-lost.

**Per-backend coverage** is documented in
[RMW vs upstream § 7](../design/rmw-vs-upstream.md#7-qos-full-dds-shaped-profile-per-backend-support-advertised).

### 8. No runtime backend swap, no runtime introspection

Standard ROS 2 ships `ros2 topic list`, `ros2 node info`, dynamic
endpoints discovery, `rmw_get_*` introspection.

nano-ros has none of that at runtime. The backend is fixed at compile
time, the wire-protocol introspection is whatever the backend natively
exposes (zenoh-pico's `z_query` for SPDP, Cyclone DDS's SPDP/SEDP
discovery, …). Use the host-side ROS 2 tools for introspection and connect via
the rmw_zenoh interop path.

**Why.** Every byte of "introspect what's running" is overhead a
microcontroller can't justify when a host-side ROS 2 environment is
one router-hop away.

### 9. Parameters: node-local server, no descriptors, no callbacks (yet)

Standard ROS 2 (`rclcpp::Node`) ships a rich parameter surface:
`declare_parameter<T>` with `ParameterDescriptor` (description, ranges,
read-only, dynamic typing), `set_parameter` returning a
`SetParametersResult`, atomic multi-set, three callback hooks
(`pre_set` / `on_set` / `post_set`), parameter overrides from the
launch file or CLI, and a service-backed remote-introspection surface
(`/<node>/get_parameters`, `/<node>/set_parameters`, …).

nano-ros's `nros::ParameterServer<Cap>` (C++) and the equivalent C
`nros_parameter_server_t` keep the **vocabulary** (`declare_parameter<T>`,
`get_parameter<T>`, `set_parameter<T>`, `has_parameter`) but trim the
surface aggressively for embedded use.

**What we keep**

- Same five scalar types: `bool`, `int64_t`, `double`, string, plus the
  `bool` / `int64_t` / `double` / `byte` / `string` array variants on the
  C side (`nros_parameter_*_array`).
- Same lifecycle: declare → get → set, with declare-once-then-typed-get
  semantics.
- Optional service-backed exposure (`~/get_parameters` /
  `~/set_parameters` / `~/list_parameters` / …) when the
  `param-services` feature is enabled. This pulls in ROS 2 wire compat:
  declared parameters are visible to `ros2 param list /<node>` and
  `ros2 param set`.

**What we drop, and why**

| Upstream feature | nano-ros status | Why dropped |
|---|---|---|
| `ParameterDescriptor` (description, ranges, read-only, dynamic_typing) | not exposed | descriptor metadata is host-side concern; embedded server enforces type at declare-time, range checks belong in `set` callbacks (deferred — see below) |
| `add_pre_set_parameters_callback` / `add_on_set_parameters_callback` / `add_post_set_parameters_callback` | one combined `nros_parameter_callback_t` (server-wide, fires after set) | three callbacks → three indirection slots × N subscribers; one callback covers the safety-island validation use case (`reject if out of range`) |
| `set_parameter` returning `SetParametersResult` (`successful: bool`, `reason: string`) | returns `nros_ret_t` | string `reason` would force heap or fixed-buffer; ret code captures the binary outcome |
| `set_parameters_atomically` | not exposed | atomic multi-set requires transaction log; not justified by current embedded use |
| `declare_parameters` (multi-declare with namespace) | not exposed | one-by-one declare is fine for compile-time-known parameter sets |
| Parameter overrides from CLI / launch / yaml | not exposed | embedded apps configure via Kconfig / `Config` struct; runtime overrides come over the wire via `~/set_parameters` (when `param-services` is on) |
| Storage allocation policy | compile-time `<Capacity>`, inline storage | no heap; capacity sizing belongs in the application's startup code, same as the executor arena |

**Storage shape difference**

| | rclcpp | nano-ros |
|---|---|---|
| Container | `std::map<string, ParameterValue>` (heap) | `nros_parameter_t storage[Capacity]` (caller-owned, inline) |
| String value | `std::string` (heap) | fixed 128-byte slot, copy semantics |
| Array params | `std::vector<T>` (heap) | caller-owned pointer + length (caller keeps storage alive) |
| Total fixed cost | unbounded | `Capacity × sizeof(nros_parameter_t)` known at compile time |

**Class shape difference**

`rclcpp::Node` owns the parameter store. nano-ros splits them:

```cpp
// rclcpp
auto node = std::make_shared<rclcpp::Node>("ctrl");
node->declare_parameter<double>("ctrl_period", 0.15);
double v = node->get_parameter("ctrl_period").as_double();

// nano-ros
nros::Node node;
nros::ParameterServer<8> params;
NROS_TRY(nros::Node::create(node, "ctrl"));
NROS_TRY(params.declare_parameter<double>("ctrl_period", 0.15));
double v;
NROS_TRY(params.get_parameter<double>("ctrl_period", v));
```

**Why split.** Adding a parameter store to `Node` would require
templating `Node` on capacity, which propagates through every
`create_publisher` / `create_subscription` site. Composing
`ParameterServer<N>` alongside the node keeps `Node` non-templated and
matches the rest of the freestanding C++14 surface (callers own
storage). `params.raw()` exposes the underlying
`nros_parameter_server_t*` for future ROS 2 service-backed registration.

**Why no `Box<dyn FnMut>` callback yet.** The same constraint that
shapes the QoS event callbacks applies here: nano-ros's
`#[no_std]` core forbids alloc-style indirection. A future
descriptor-+-validation-callback path will use a function pointer +
`void* user_context` pair, registered at declare-time. Tracked under
the upstream parity backlog.

**Going further.** When upstream's full parameter surface matters —
`describe_parameter`, ranges, three-stage validation callbacks,
override files — fall back to running a host-side ROS 2 node that
exposes them and uses the embedded node only as the leaf publisher /
subscriber.

## What this means in practice

If you are coming from `rclcpp`:

- Open an [`Executor`](../api/cpp/classnros_1_1Executor.html), then
  create the node from it.
- Decide poll vs. callback per subscription, not globally.
- If the platform has `std`, `nros::init()` looks identical; if it is
  RTOS / bare-metal, plan the executor arena up front.
- Pick `nros-rmw-zenoh` for ROS 2 interop; everything else is a
  different trade-off.

If you are coming from `rclrs`:

- The umbrella crate is [`nros`](../reference/rust-api.md), not split
  into `rclrs_*`. `nros::prelude` gives you everything.
- `Executor::open(&config)` is the equivalent of
  `Context::default_from_env()` + `Executor::new(...)`.
- The async surface is on the executor + handles themselves (re-exported at the crate
  root). Compatible with tokio out of the box.

If you are coming from `rclc`:

- The C entry points carry rclc's own names (`rclc_node_init_default`,
  `rclc_publisher_init_default`, `rclc_subscription_init_default`), in
  rclc's argument order and at rclc's arity. Memory ownership rules are
  the same — the caller owns storage, the API initialises it.
- See the [C API reference](../reference/c-api.md) for the full
  surface.

## Going deeper

- API surface, type by type → per-language references at
  [Rust](../reference/rust-api.md) / [C](../reference/c-api.md) /
  [C++](../reference/cpp-api.md).
- Why the executor / RMW / platform layers split this way →
  [Architecture Overview](architecture.md).
- Cooperative `no_std` model → [no_std Support](no-std.md).
