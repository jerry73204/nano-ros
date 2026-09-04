# Cross-backend bridges

Most nano-ros applications pick one RMW backend at compile
time and never look back. A cross-backend bridge is the one
case where a single binary needs **two** backends in parallel
— one node receiving on backend A, another publishing on
backend B, the executor draining both each tick.

Typical reasons:

- **Drone link.** A flight controller speaks XRCE-DDS over a
  serial radio link; the ground-station LAN runs standard
  DDS. A bridge forwards telemetry verbatim between the two.
- **Field gateway.** Sensor pods talk zenoh-pico over an
  unreliable Wi-Fi mesh; the ROS 2 datacenter side wants
  Cyclone DDS. A bridge republishes after a session-buffered
  drop check.
- **Safety carve-out.** Mission-critical traffic stays on
  the deterministic backend; introspection / logging
  republishes to a best-effort backend so dashboards never
  starve the critical pipeline.

This chapter walks the model, the build knobs, and the three
shipped examples.

## The mental model — `rclcpp::Node`, twice

nano-ros mirrors rclcpp:

```
Executor exec;                   // owns the primary session
Node ingress = exec.node_builder("ingress").rmw("xrce").build();
Node egress  = exec.node_builder("egress").rmw("cyclonedds").build();
auto sub = ingress.create_subscription<Foo>(...);
auto pub = egress.create_publisher<Foo>(...);
```

The executor opens one primary session at `open*` time. Each
extra `node_builder(...).rmw(name)` call:

1. Looks up `name` in the registry of linked backends.
2. If `name == primary_rmw` and the locator matches → the
   Node binds to the primary session (slot 0). No second
   session opened.
3. Else → opens a fresh session through that backend's
   `open_with_rmw` and stores it in `extra_sessions[N-1]`.
   The new Node's `session_idx = N`.

`spin_once()` drains every session in turn. Handles created
through a multi-Session Node route through
the node's resolved session record instead of the legacy support/session
pointer kept for single-backend callers.

`NodeRecord.session_idx` is the dispatch key. Print it to
verify which session each Node landed on:

```rust
exec.node(node_in).unwrap().session_idx   // 0 = primary
exec.node(node_out).unwrap().session_idx  // 1 = first extra
```

## Build knobs — three audiences, three shapes

### Rust binary

Add both backend crates to `Cargo.toml` and call each
`register()` early in `main`:

```toml
[dependencies]
nros            = { ..., features = ["rmw-cffi"] }
nros-rmw-zenoh  = { ... }
nros-rmw-xrce-cffi = { ... }
```

```rust
fn main() {
    nros_rmw_zenoh::register().expect("register zenoh");
    nros_rmw_xrce_cffi::register().expect("register xrce");
    let mut exec = nros::Executor::open_with_rmw("zenoh",
        &nros::ExecutorConfig::from_env())?;
    let ingress = exec.node_builder("ingress").rmw("zenoh").build()?;
    let egress  = exec.node_builder("egress").rmw("xrce").build()?;
    // ...
}
```

**Why explicit `register()`?** Stable Rust drops un-referenced
rlib CGUs from the final link even when the backend's
`#[used] static RMW_INIT_ENTRIES` exists. The `register()`
call doubles as the symbol reference that drags the rlib in
**and** the registration trigger. C / C++ builds dodge this
because `--whole-archive` pulls every section unconditionally.

### C binary

`NANO_ROS_RMW=none` switches off the root CMake's
RMW auto-pull. Pull each backend's staticlib through
`corrosion_import_crate` and wrap with `--whole-archive` so
the registry walker finds both names:

```cmake
set(NANO_ROS_RMW none)
add_subdirectory(${nano_ros_root} nano_ros)

corrosion_import_crate(
    MANIFEST_PATH ${nano_ros_root}/Cargo.toml
    CRATES nros-rmw-xrce-cffi
    NO_DEFAULT_FEATURES
    FEATURES std)
corrosion_import_crate(
    MANIFEST_PATH ${nano_ros_root}/Cargo.toml
    CRATES nros-rmw-zenoh-staticlib
    NO_DEFAULT_FEATURES
    FEATURES "platform-posix;ros-humble")

target_link_libraries(my_bridge PRIVATE
    NanoRos::NanoRos
    -Wl,--whole-archive
    nros_rmw_xrce_cffi-static
    nros_rmw_zenoh_staticlib-static
    -Wl,--no-whole-archive)
```

Then declare + call the register functions early in
`nros_app_main`:

```c
extern int8_t nros_rmw_xrce_register(void);
extern int8_t nros_rmw_zenoh_register(void);

int nros_app_main(int argc, char** argv) {
    if (nros_rmw_xrce_register() != 0)  return 1;
    if (nros_rmw_zenoh_register() != 0) return 1;
    // ... nros_support_init / nros_executor_init / nros_executor_node_init
}
```

`nros_executor_node_init` honours per-Node `rmw_name` +
`locator` via the `nros_node_options_t` struct; the executor
sets `node.executor` so subsequent `nros_*_init` calls
(publisher, subscription, service, action) route to the right
session.

### C++ binary

Same CMake shape as C. The C++ surface (`nros-cpp`) is a
thin header layer; the linker work is identical. Use the
`Executor::node_builder(name)` chain:

```cpp
auto exec = nros::Executor::open_with_rmw("zenoh", cfg);
auto ingress = exec.node_builder("ingress").rmw("zenoh").build();
auto egress  = exec.node_builder("egress").rmw("xrce").build();
auto pub = egress.create_publisher<std_msgs::String>("/chatter");
exec.register_subscription_on<std_msgs::String>(ingress, "/chatter",
    [&pub](const auto& msg) { pub->publish(msg); });
```

## `NROS_RMW` environment variable

When a binary links multiple backends, **set `NROS_RMW` to
pin the primary** before `open()`:

```sh
NROS_RMW=zenoh ./my_bridge
```

The C-side `nros_support_init` reads it and routes
`Executor::open` through `open_with_rmw(name, ...)`. Without
this, the linkme walker returns whichever backend's ctor
fired first, which is non-deterministic across link
orderings. The bridge then opens *another* session against
the same backend when the per-Node `.rmw(name)` matches the
unintended primary — and most singleton-state backends
(XRCE-DDS-Client's uxrSession, zenoh-pico's global
`g_session`) refuse a second open.

The Rust path mirrors this: `Executor::open` consults
`$NROS_RMW` first; the per-Node `resolve_session_slot`
detects the primary-name match and returns slot 0 instead
of opening a duplicate.

## Memory + WCET budget

Each extra session adds:

- One `ConcreteSession` (RMW-specific; see [RMW
  Backends](../internals/rmw-backends.md#real-time-budget-per-backend)
  for sizing).
- One `register_wake_signal_on_extra` wake-callback slot
  (`std` only; bare-metal targets share the primary wake).
- One round-trip through `spin_once` per backend per tick.

The bridge's per-tick WCET is the **sum** across linked
backends:

```
bridge_wcet = Σ poll_wcet_i + Σ dispatch_wcet_j
```

Read the per-backend numbers in the [Real-time budget per
backend](../internals/rmw-backends.md#real-time-budget-per-backend)
table and add them up — there is no parallelism between
backends inside a single executor.

For deadline-critical bridges, partition by SchedContext
instead of running everything on the default Fifo slot:

```rust
let ingress = exec.node_builder("ingress")
    .rmw("xrce")
    .sched(critical_sc)   // RT priority
    .build()?;
let egress = exec.node_builder("egress")
    .rmw("cyclonedds")
    .sched(best_effort_sc)
    .build()?;
```

The PiCAS-style per-callback OS-priority dispatcher (gated
behind the `scheduler-os-priority` feature) routes each Node's
callbacks to its own OS priority slot so the slow backend cannot
block the fast one.

## Shipped examples

### `examples/bridges/tt-zenoh-to-xrce/`

Pure-Rust bridge. Zenoh ingress → XRCE-DDS egress under an
ARINC-653-style time-triggered cyclic schedule. Read this
first — it shows the multi-RMW
`Executor::open_with_rmw("zenoh", ...)` plus
`node_builder.rmw("xrce")` per-session pin, with raw byte
forwarding and no codegen.

```sh
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &
build/xrce-agent/MicroXRCEAgent udp4 -p 8888 &
NROS_XRCE_LOCATOR=udp/127.0.0.1:8888 \
    cargo run -p native-rs-bridge-tt-zenoh-to-xrce
```

### `examples/bridges/tt-zenoh-to-cyclonedds/`

The stock-Cyclone-DDS sibling: same TT schedule, but the egress is
`.rmw("cyclonedds")`, forwarding onto the DDS databus where a stock
`rmw_cyclonedds_cpp` (e.g. an Autoware listener) or another nano-ros cyclonedds
node receives the samples. **One structural difference** from the XRCE variant:
Cyclone rejects a raw publisher whose topic type has no registered
`dds_topic_descriptor_t`, so the egress type's schema is staged **before** the
raw publisher is created —

```rust
// The Cyclone backend installs the registrar during its register():
nros_rmw_cyclonedds_sys::register()?;
// Stage std_msgs/msg/String's schema (NUL-terminated key — it is handed
// straight to Cyclone's C descriptor table):
nros_rmw::register_type_descriptor(
    "std_msgs/msg/String\0",
    &[nros_serdes::schema::Field { name: "data\0", ty: FieldType::String, offset: 0 }],
)?;
let pub_out = node_out.create_publisher_raw("/chatter", "std_msgs/msg/String", hash)?;
```

XRCE registers lazily from name+hash; Cyclone needs the descriptor up front. The
backend links the vendored CycloneDDS (no `-DNANO_ROS_RMW` needed for the Rust
binary — the `nros-rmw-cyclonedds-sys` dep is the selection).

```sh
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &
ROS_DOMAIN_ID=0 cargo run -p native-rs-bridge-tt-zenoh-to-cyclonedds
# subscribe on Cyclone DDS /chatter (stock ROS 2 or a nano-ros cyclone node on
# the same ROS_DOMAIN_ID) and publish on zenoh /chatter to see bridged samples.
```

## Coverage matrix

Bridge examples live under
`examples/bridges/<name>/` (cross-platform, transport-
spanning) or under their canonical
`examples/<plat>/<lang>/bridge/<name>/` cell when the bridge
is platform-specific. The
[examples README coverage
matrix](https://github.com/NEWSLabNTU/nano-ros/blob/main/examples/README.md#coverage-matrix)
lists which `<plat> × <lang>` combinations ship a bridge today
(a bridge spans RMW backends by nature, so RMW is not a directory axis).

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `Transport(ConnectionFailed)` on `open_with_rmw("X", ...)` | Backend X's rlib not pulled into the link line. Rust: add a `register()` call. C / C++: confirm `--whole-archive` wraps the staticlib. |
| Second node's `.rmw("zenoh")` returns `Transport(...)` | Both nodes try to open against the same singleton-state backend. Set `NROS_RMW=zenoh` so the primary lands on zenoh + the second Node hits the session-cache (slot 0) instead of double-opening. |
| `rclc_publisher_init_default -> -7` after `nros_executor_node_init` | Stale build — the C-side multi-Session dispatch predates nros-v0.5.0; rebuild `nros-c` after updating. |
| `Bridge spinning` marker never reaches piped test harness | Add `setvbuf(stdout, NULL, _IOLBF, 0)` at the top of `nros_app_main`. glibc full-buffers piped stdout; line-buffering surfaces readiness markers before the long-lived `spin_period` loop. |

## See also

- [Choosing an RMW Backend](rmw-backends.md) — single-
  backend selection criteria.
- [RMW Backends — Host-Language
  Policy](../internals/rmw-backends.md) — registry +
  per-backend sizing.
- [`docs/roadmap/archived/phase-104-multi-backend-bridges.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/archived/phase-104-multi-backend-bridges.md)
  — design rationale + acceptance criteria.
- [`docs/roadmap/archived/phase-156-bridge-runtime-blockers.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/archived/phase-156-bridge-runtime-blockers.md)
  — the four sub-bugs that gated the D.3 / D.4 E2E tests +
  how each was resolved.
