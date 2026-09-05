# Cyclone DDS RMW — Known Limitations

Phase 117 ships `nros-rmw-cyclonedds` as the fourth RMW backend
alongside `rmw-zenoh`, `rmw-xrce`, and `rmw-dds`. The implementation
is **complete enough for nano-ros ↔ nano-ros pub/sub + service
round-trips on the native (host) build**, but several pieces are explicitly deferred.
This page tracks them so consumers (Autoware safety-island, future
follow-up phases) plan around them rather than rediscovering them at
integration time.

## Pin

- Submodule pinned to **tag `0.10.5`** (`third-party/dds/cyclonedds/`).
- Wire-compatible with `ros-humble-cyclonedds` 0.10.5 +
  `ros-humble-rmw-cyclonedds-cpp` 1.3.4.
- Upgrading the pin requires matching the consumer's ROS 2
  distribution; Cyclone does not commit to wire compat across `0.x`
  minor releases.

## Data plane: RESOLVED — the backend carries CDR

**This section used to describe a 2× CDR round-trip per message, and a
blocker that turned out not to exist.** Both are gone; the history is kept
because the retracted reasoning is the useful part.

`publish_raw` hands the caller's bytes to `dds_write` as an `NrosCdrBlob`, and
`subscription_take` returns the serdata's own bytes via `dds_takecdr` +
`ddsi_serdata_to_ser`. Neither direction builds a typed C sample, so nothing is
decoded or re-encoded. See `src/nros_sertype.{hpp,cpp}`.

**What the old text got wrong.** It said the zero-copy path was unreachable
because Cyclone 0.10.5 does not expose `dds_writer_lookup_serdatatype()`, and
pointed at upstream
[cyclonedds#1342](https://github.com/eclipse-cyclonedds/cyclonedds/issues/1342)
as the thing to wait for. Two corrections
([#0969](../issues/0969-cyclone-take-cdr-round-trip.md),
[#0970](../issues/archived/0970-cyclone-rmw-should-own-its-sertype.md)):

* `dds_takecdr` takes a reader entity and nothing else, so the RECEIVE side was
  never blocked on any sertype lookup.
* That API recovers a sertype you do not own. `rmw_cyclonedds_cpp` never calls
  it, because it registers its own with `dds_create_topic_sertype` — public,
  and present in our vendored 0.10.5. Owning the sertype makes the question
  moot rather than answering it.

**What a consumer sees now.** The bytes handed back are the WIRE bytes. Two
consequences that are not obvious:

* An XCDR2 publisher's framing survives instead of being flattened to XCDR1.
* A big-endian peer is no longer normalised. `nros-serdes` decodes
  little-endian unconditionally, so a BE peer was never supported end to end;
  this backend was the only one papering over that.
* The length is the wire length. RTPS pads a submessage's payload to 4 bytes at
  the SENDER, so a 25-byte message arrives as 28 (`WIRE=len:28 hdr:00010000
  cdr:25`, measured against ROS 2 Humble). A receive buffer sized to a type's
  exact bound is up to 3 bytes short — which is why the derived RX bound is
  rounded up to 4 (`rosidl_codegen::bounds::transport_framed`).

**Still round-tripping:** `src/service.cpp`, which is why `sertype_min.{hpp,cpp}`
still exists.

It was "blocked, and not on effort" on #0976 — nothing could observe whether the
five CDR-reshaping adapters produce ROS 2's bytes, so converting the path risked
breaking a wire format with no witness. **That blocker is lifted in one
direction**: `6ec8c4d21` added `ros2_action_e2e.rs`, a stock `ros2 action
send_goal` against the nano-ros action server, and it passes.

The remaining work is effort, not a blocker. And the WRITE-side adapters turn
out to have nothing to protect: instrumenting both branches of `write_typed` and
running the Rust, C and C++ action clients against a stock ROS 2 server shows
`strip_goal_id_len_at` and `strip_nested_cdr_at` DECLINING in all three, with
identical decline counts — three independent producers agreeing the corrections
have nothing left to correct. The runtime already emits the fixed `octet[16]`
(publisher.cpp's 233.6 note). So
[#0969](../issues/0969-cyclone-take-cdr-round-trip.md)'s expectation that
converting this path would DELETE these adapters rather than preserve them is
measured on the write side.

What is still open is the TAKE side and the test surface:
`take_fibonacci_get_result_response_wire` fires only when nano-ros is the client
taking a result, and no automated test exercises either it or the write-side
strips — the action witness (`ros2_action_e2e.rs`) runs the server direction
only, and the measurement above was an instrumented run rather than a registered
test. See
[#0976](../issues/archived/0976-service-action-adapters-tested-only-against-ourselves.md).

## Phase 108 status events: NULL slots

The vtable's three event hooks are NULL:

```c
.register_subscription_event   = NULL,
.register_publisher_event    = NULL,
.assert_publisher_liveliness = NULL,
```

Liveliness changes, deadline misses, and message-lost events are
**not delivered to the runtime** even though Cyclone tracks them
internally via `dds_set_listener`. Apps that rely on
`add_subscriber_event_callback` will silently see no firings on
this backend.

**Path forward:** wire Cyclone's reader/writer listener trampolines
through to `rmw_event_callback_t` in a separate phase. Each
status callback maps cleanly to one event kind; ~150 LOC.

## Service request-id correlation — done (Phase 117.7.B)

Service traffic now wraps every Request and Reply in a
backend-defined envelope:

```idl
struct ServiceEnvelope {
    unsigned long long client_id;
    long long          seq;
    sequence<octet>    payload;
};
```

Each `service_call_raw` stamps a unique random `client_id` (per
client, allocated at create time) plus a monotonic `seq` (per
client, atomic). The server stores `(client_id, client_seq)` in a
32-slot table when it takes a request, returns the slot index as
the runtime-visible `seq`, and echoes the original
`(client_id, seq)` on the matching `service_send_response`. The
client's `service_call_raw` poll filters incoming replies on
`(client_id, seq) == (mine, my_seq)`. Concurrent calls from
parallel clients no longer interleave.

`service_concurrent` CTest fires 5 calls from each of two parallel
clients against one server and asserts each client receives only
its own replies in order.

**Wire compat — done (Phase 117.12.B).** The interim
`ServiceEnvelope` was replaced by stock `rmw_cyclonedds_cpp`'s
`cdds_request_header_t` (`{uint64_t guid; int64_t seq;}`, 16 bytes,
see upstream `src/serdata.hpp:73-77`). Codegen now injects
`unsigned long long rmw_writer_guid; long long rmw_sequence_number;`
into every `_Request_` / `_Response_` IDL struct, and the
backend's `(build|split)_wire_header` helpers serialise / parse the
matching 16-byte header at the front of each CDR. Bidirectional
interop validated by `nros_rmw_cyclonedds_ros2_srv_e2e` against
`ros2 service call` and `ros2 run demo_nodes_cpp
add_two_ints_server`.

**Caveat — cap:** the server-side slot table is fixed at 32. A
server with more than 32 outstanding requests will report
`NROS_RMW_RET_WOULD_BLOCK` from `take_request` until the
application drains via `send_response`. Tune by editing
`kRequestSlots` in `src/service.cpp`.

**Caveat — Cyclone same-participant local-delivery race:**
creating two service clients on the same `rmw_session_t`
back-to-back occasionally results in only the second writer
matching the server's reader (Cyclone 0.10.5 local-delivery
shortcut). Stagger client creation by ≥ 100 ms, or move to one
participant per service client.

**Caveat — `service_concurrent` test disabled by default
(Phase 117.X.5).** With the per-client-participant workaround
in place, cross-participant SEDP discovery on the native (host) build still
consistently drops the last reply on one of the two clients
(Cyclone 0.10.5). The `(writer_guid, seq)` filter logic is
functionally validated by `service_roundtrip` (single client,
single call) and `mangling_test` (descriptor + type-name
correctness). The concurrent harness can be re-enabled with
`-DNROS_RMW_CYCLONEDDS_RUN_SERVICE_CONCURRENT=ON` for local
investigation; closing the gap likely requires explicit
publication-matched-status polling in
`service_client_create` and is tracked separately.

## ROS 2 wire interop — done (Phase 117.12)

Native (host) E2E against stock ROS 2 nodes on the same domain is
validated bidirectionally by two CTest harnesses
(`nros_rmw_cyclonedds_ros2_pubsub_e2e`,
`nros_rmw_cyclonedds_ros2_srv_e2e`):

- **Pub/sub:** nano-ros publisher ↔ `ros2 topic echo /chatter`
  (byte-equal `std_msgs/msg/String` payload) and `ros2 topic pub`
  ↔ nano-ros subscriber.
- **Services:** nano-ros server ↔ `ros2 service call /add_two_ints
  example_interfaces/srv/AddTwoInts` and stock `ros2 run
  demo_nodes_cpp add_two_ints_server` ↔ nano-ros client.

The harness picks a multicast-capable ethernet interface
(auto-detect, override via `NROS_RMW_CYCLONEDDS_E2E_IFACE`) and
writes a per-test `CYCLONEDDS_URI` config so SPDP works on hosts
where `lo` is non-multicast. Both harnesses skip cleanly with
`[SKIPPED]` if `/opt/ros/humble/setup.bash`, the `ros2` CLI, or a
suitable interface is missing.

## QoS coverage

`make_dds_qos` honours the full `rmw_qos_profile_t` field set
(reliability, durability, history+depth, deadline, lifespan,
liveliness+lease) **except**:

- `MANUAL_BY_NODE` liveliness — folded to `MANUAL_BY_TOPIC` (Cyclone
  has no node-scoped variant).
- `max_blocking_time` on reliable writers — hard-coded to 100 ms
  to match `rmw_cyclonedds_cpp`. Surfacing it through
  `rmw_qos_profile_t._reserved` is a follow-up.

## Type discovery (XTypes metadata) — Phase 117.X.6 opt-in

By default the codegen helper passes `idlc -t` which **omits the
XTypes type-information section** from the generated descriptor.

**Why.** Cyclone 0.10.5's `idlc` segfaults emitting type-info on
**any** input — verified with the trivial `@final struct Simple {
long x; };` (runs `idlc -l c`, prints `Failed to compile`, output
`.c` is truncated mid-ops-array, no descriptor emitted). The bug
is independent of our IDL shape. Tag `0.10.5` is the latest patch
on the upstream `0.10.*` branch (no `0.10.6`).

**Why we default to `-t`.** Type-info is optional on the wire —
peers fall back to typename matching, which is what nano-ros
publishers / subscribers / services already use end-to-end. Stock
ROS 2 `rclcpp` apps interop fine. Only `ros2 topic info -v` (which
queries `DCPSPublication` / `DCPSSubscription` builtin topics for
XTypes metadata) shows blank type info for nano-ros endpoints.

**Opt-in once the upstream bug is fixed.** Set
`-DNROS_RMW_CYCLONEDDS_INCLUDE_TYPE_INFO=ON` (cmake cache var) or
`NROS_RMW_CYCLONEDDS_INCLUDE_TYPE_INFO=1` (env). The cmake helper
runs an idlc probe at configure time against a synthetic minimal
IDL — if Cyclone still has the type-info bug the configure errors
out with a pointer to this doc; otherwise the helper drops `-t`
and the regenerated descriptors carry full type-info. Today the
probe fails on the bundled Cyclone 0.10.5 submodule; the option
lights up automatically the day the pin moves past the fixed
release.

## Test rpath / `LD_LIBRARY_PATH`

`packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/CMakeLists.txt` sets
`ENVIRONMENT "LD_LIBRARY_PATH=<prefix>/lib:..."` on every CTest
target so the binaries resolve `libddsc.so.0` from `build/install/`
instead of `/opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so.0`. The
ROS 2 environment that `.envrc` sources adds the system path
first. Without the env override:

- `ldd` shows the system `libddsc.so` resolving first.
- The binary launches but crashes inside Cyclone with SIGSEGV
  because `nros_rmw_cyclonedds.a` was compiled against our 0.10.5
  headers and the runtime is the system Cyclone (different build
  flags, possibly different layout).

Downstream consumers (nros-cpp examples linking via
`add_subdirectory(<repo>)` with `NANO_ROS_RMW=cyclonedds`) inherit a
build-tree rpath via `CycloneDDS::ddsc`'s own `INTERFACE_LINK_OPTIONS`,
so the issue is contained to the in-tree CTest harness. Document
this in any external integration guide.

## Heap allocations on the data path

**Measured, and 5× lower than this section used to describe.** Per publish+take
round trip, on the same harness before and after
[#0969](../issues/0969-cyclone-take-cdr-round-trip.md) /
[#0970](../issues/archived/0970-cyclone-rmw-should-own-its-sertype.md):

| | allocs @1 msg | allocs @200 msgs | per message |
| --- | ---: | ---: | ---: |
| before | 984 | 2,960 | **9.93** |
| after | 968 | 1,366 | **2.00** |

The remaining two are one serdata object and its payload buffer. On the loopback
path Cyclone hands the same serdata to the local reader by reference, so one
message costs one serdata.

The before figure is not an integer, and that is the part worth keeping: the old
path's `dds_ostream` grew by `realloc`, so its allocation count depended on the
sample. Part of what the round trip cost VARIED WITH THE MESSAGE — the property
a real-time budget most dislikes. The after figure is exactly 2.00 across 199
messages.

The payload buffer comes from `ddsrt_malloc`, not `new[]`: on ThreadX and
FreeRTOS the libc heap is separate from the ddsrt heap Cyclone was given.
`scripts/rmw-alloc-sites.py` accounts for it.

**Reproduce it:** `data_roundtrip` takes `NROS_ROUNDTRIP_ITERS`; two runs at
different counts under valgrind cancel session and entity setup and give the
per-message cost as a slope. The claim that "the smoke tests don't measure
allocation pressure" is no longer true, which is why it is no longer here.

**Still allocating per message:** `src/service.cpp` — three sites, one per
request and two per reply. The take side is
[#0969](../issues/0969-cyclone-take-cdr-round-trip.md)'s third site
(`take_typed_wire`, which re-encodes XCDR1 native-endian — a correctness
question, not only a cost one); the witness situation is
[#0976](../issues/archived/0976-service-action-adapters-tested-only-against-ourselves.md).

## Boards

*(phase-321 W3.c — this section said `nros-board-fvp-aemv8r-smp` and
`nros-board-s32z270dc2-r52` were "**not yet implemented**" and that the Cyclone
backend "works on POSIX only". Both were overtaken by phase-292/298 and stayed
wrong for months. Current status is generated, not asserted here — see
[Board Support Tiers](../../book/src/reference/board-support-tiers.md), checked
by `just check board-tiers`.)*

- **`nros-board-fvp-aemv8r-smp` — tier 3, build-only.** The board exists and
  Cyclone runs on it: phase-298 booted the two-tier workspace Entry on
  `FVP_BaseR_AEMv8R`, brought up ethernet and published `/ctrl` + `/telem`. But
  the model is license-gated, so no CI lane can boot it; the runtime gate is
  maintainer-run via `just zephyr verify-fvp-runtime`.
- **`nros-board-s32z270dc2-r52` — scaffold.** Config and skeleton only, zero
  cargo consumers, and its one build recipe has no caller. Do not plan against
  it without reading issue tracking first.
- Zephyr Cortex-A / Cortex-R targets still need the `aarch64-zephyr-elf`
  toolchain in the Zephyr SDK install, and hardware or the license-gated model
  for any runtime claim.

See `docs/roadmap/phase-117-cyclonedds-rmw.md` for the per-item
breakdown.

## Runtime type registry sizing (Phase 212.K.7)

Generated msg crates are RMW-agnostic — Cyclone DDS sertypes are
built lazily on first `create_publisher<M>` / `create_subscription<M>`
for a given message type and cached in a bounded `no_std` registry
inside `nros-rmw-cyclonedds`. The cap is a build-time env knob:

- **`NROS_CYCLONEDDS_MAX_TYPES`** — default **32**. Wired through
  the `nros-sizes` build probe (same pattern as
  `EXECUTOR_OPAQUE_U64S`). Each slot costs ~16 bytes static (one
  `u64` type-hash + one `NonNull<ddsi_sertype>`); default footprint
  ~512 bytes.
- Overflow on first-use registration trips a compile-time
  `const _: () = assert!(...)` from the `nros-sizes-build` hook —
  no runtime failure mode.
- Raise the knob for bridge / aggregator nodes that touch many
  distinct message types; lower it on Cortex-M0+ where every
  static byte counts.

The descriptor itself (the `ddsi_sertype` plus Cyclone's internal
type-cache entries) is still allocated from Cyclone's `ddsrt` heap.
On FreeRTOS + ThreadX that heap is `kEmbeddedCycloneConfig`'s fixed
pool (Phase 177.22) — pre-budget it for the worst-case set of
message types the participant will publish or subscribe to.

See section 212.K.7 of
`docs/roadmap/phase-212-ux-cargo-native-and-file-consolidation.md`
for the full design + work-item ledger.

## `rx_buffer_hint` is inapplicable here, not unimplemented

`rmw_subscription_options_t::rx_buffer_hint` tells a backend how many bytes a
receive buffer needs for a type, so a size-classing backend (zenoh-pico) can pick
a class. **This backend ignores it, deliberately, and there is nothing to wire
up.** phase-392 W3f asked for either the wiring or this statement; this is the
statement.

There is no receive buffer here to size:

* the sample arrives in a serdata, sized by the sample, allocated when it
  arrives;
* the destination is the CALLER's buffer, whose capacity arrives on every `take`
  and is authoritative there.

Before [#0969](../issues/0969-cyclone-take-cdr-round-trip.md) there was exactly
one candidate consumer — the `dds_ostream` that re-serialised the typed sample
grew by `realloc`, and an initial size would have saved those reallocs. That
ostream went with the round trip, and with it the last thing a hint could have
sized.

**So do not measure the backend for an effect from the sizing campaign.** A
Cyclone consumer can set the hint, do everything phase-392 asks, and correctly
observe nothing change here. What DOES change is the executor's arena, which
nano-ros sizes itself from the same bound. Measure the arena, not the backend.

The ABI declares the field advisory and permits a backend to ignore it
(`rmw_entity.h`). [Issue 0958](../issues/archived/0958-cyclonedds-ignores-rx-buffer-hint.md)
was not that it was ignored — it was that it was ignored *silently*, discarded at
a bare `/*options*/` with nothing for a reader to find. `subscription_create`
now carries the same explanation at the parameter itself.

## No E2E message-integrity (safety-e2e / CRC)

The `safety-e2e` capability (CRC attach on publish + validate on receive, surfaced via
`ctx.integrity()` / `nros_subscription_take_validated`) is **zenoh-only**. The CRC
machinery lives in the zenoh shim's wire attachment (`nros-rmw-zenoh`); CycloneDDS (and
XRCE) carry no `safety-e2e` feature, so a declared `[safety]` axis no-ops on them. The
`NANO_ROS_SAFETY_E2E=ON` CMake option **warns and is ignored** when `NANO_ROS_RMW` is not
`zenoh`. Adding a CycloneDDS integrity path (a DDS-side CRC + a C surface) is unscoped —
see [issue 0300](../issues/archived/0300-safety-e2e-c-cpp-cmake-path-missing.md).
