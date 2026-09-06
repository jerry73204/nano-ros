---
id: 1033
title: "The XRCE session budgets eight 32-deep subscriber rings, so a one-subscriber image pays 266,368 bytes for seven it will never use"
status: resolved
area: rmw, memory
severity: high
found: 2026-09-04
related: [0968, 0827, 0900, 0965, 0460, phase-392, phase-412]
---

# 62% of the XRCE session struct is slots the image does not have

## Measured

`sizeof(xrce_session_state_t)` on the zephyr cpp-listener-xrce image, read from
its own DWARF, after the MTU fix in issue 0968 took it from 427,968:

| member | bytes | share |
| --- | ---: | ---: |
| `subscriber_slots[8]` @ 33,296 | **266,368** | 86% |
| `service_server_slots[4]` @ 4,384 | 17,536 | 6% |
| `output` + `input_reliable_buf` @ 8,192 | 16,384 | 5% |
| `service_client_slots[4]` @ 1,040 | 4,160 | 1% |
| rest | 5,248 | 2% |
| **total** | **309,696** | |

Against a `CONFIG_NROS_ZEPHYR_HEAP_SIZE` of 65,536. The image is a LISTENER: it
has **one** subscriber, no service server and no service client.

A slot is `XRCE_SUBSCRIBER_RING_DEPTH` (32) x `XRCE_BUFFER_SIZE` (1024) plus 528
bytes of bookkeeping. So the image reserves 32 buffered samples for each of
eight subscriptions, having declared one.

## This is issue 0827's class, one backend over

0827 is "unused RMW pools dominate static RAM" for zenoh — `SERVICE_BUFFERS` and
`LARGE_PAYLOADS` sized for entities a talker does not have. This is the same
shape on XRCE, and it is worse in one respect: zenoh's pools are `.bss` that
`just mem-report` can see, while this is a single heap allocation that only
shows up as a boot-time failure.

## The knobs exist and are honoured — nothing sets them

`NROS_XRCE_MAX_SUBSCRIBERS`, `NROS_XRCE_MAX_SERVICE_SERVERS`,
`NROS_XRCE_MAX_SERVICE_CLIENTS` and `NROS_XRCE_SUBSCRIBER_RING_DEPTH` all reach
`build.rs` and all have a minimum of 1. `internal.h`'s own header comment says
what they buy:

> The default `xrce_session_state_t` is ~390 KB; a pub-only bare-metal node can
> drop it well below 32 KB by setting subscribers/services to 0 and smaller
> per-entity buffers.

So this is not a defect in the mechanism. It is that every Zephyr XRCE example
ships the 8/4/4 defaults, and those defaults are sized for a workload none of
them run.

`CONFIG_NROS_XRCE_SUBSCRIBER_RING_DEPTH` is the exception worth checking: the
other three have Kconfig entries and this one appears not to, so on a Zephyr
image the ring depth may not be settable at all.

## Two ways to fix it, and the choice is the campaign's

1. **State the caps per example.** Correct immediately, and it is the twenty
   hand-picked numbers issue 0827 declined — planted in the demos people copy.
2. **Derive them**, from the entity inventory issue 0965 built. A C++ component
   image CAN declare `ENTITIES`, and `examples/workspaces/cpp` now does; these
   zephyr example leaves do not. The count of `sub:` entries IS
   `XRCE_MAX_SUBSCRIBERS`, exactly as `NROS_DERIVED_MAX_SUBSCRIBERS` already
   works for the zenoh pools (phase-412 W1).

(2) is the campaign's answer and reuses machinery that exists. What it needs is
the zephyr example leaves to declare, plus the XRCE caps joining the derivable
ladder beside the zenoh ones.

## Do not just raise the heap

Raising `NROS_ZEPHYR_HEAP_SIZE` past 310 KB would make the three cells in
[issue 0968](../0968-tier2-runtime-failures-unreproduced.md) pass while leaving an
image that reserves 86% of its session struct for entities it does not create.
That is the shape this campaign exists to remove, and the boot failure is
currently the only thing making it visible.


## Option 2 taken (2026-09-04) — see the section after this one for the composition fix

The derivation is wired and the declarations are written; the composition step
that joins them does not run for these leaves. All three facts are measured.

**Wired.** `NROS_XRCE_MAX_SUBSCRIBERS` is on the derivable ladder
(`_nros_resolve_derivable_knob`), reading the SAME
`NROS_DERIVED_MAX_SUBSCRIBERS` the zenoh pools use. Reusing that value is the
point rather than counting `sub:` entries again here: it already carries
`ACTION_CLIENT_SUBSCRIPTIONS`, the multiplier `check-infra-queryable-counts`
holds, so an action-carrying image is sized right. A second count would be a
second derivation of one fact, and the copy that forgot the multiplier would
under-size the image and fail at registration.

Its Kconfig default becomes the `-1` DERIVE sentinel. Verified inert where
nothing derives: the image still requests 309,696 bytes, because rung 4 leaves
the knob unresolved and `build.rs` falls to the crate default of 8 — exactly
what every image did before.

**Declared.** All six `examples/zephyr/cpp/*` leaves now state `ENTITIES`, read
off their own sources. The declaration reaches the metadata:

```
$ jq .components[].entities build-cpp-listener-xrce/nros-metadata.json
["sub:std_msgs/msg/String:/chatter"]
```

**And the join does not happen.** The inventory reads:

```
set(NROS_ENTITY_INVENTORY_STATUS "refused")
set(NROS_ENTITY_INVENTORY_REASON "no entity inventory composed yet")
```

`nros_derive_entity_inventory_knobs` has exactly ONE caller —
`NanoRosEntry.cmake:890`, inside `nano_ros_entry()`. These zephyr examples are
standalone Zephyr applications that register a node and never call it, so their
declaration is captured and never composed. A second configure does not help;
this is not issue 0991's one-configure lag, it is a step that never runs.

### What that leaves

The remaining work is to compose the inventory on the non-entry path, so a
standalone leaf that declares can derive. That is a change to configure
ORDERING — the composer must run after every registration, which is precisely
why it lives in `nano_ros_entry()` today — and it belongs with whoever owns
that seam rather than bolted on at the call site.

Until then the two changes here are correct and dormant: the ladder is right,
the declarations are right, and the image keeps the default it always had.
Nothing regressed and nothing is yet saved.

**Still do not raise the heap.** The reasoning above is unchanged.


## The composition gap is CLOSED (2026-09-04) — 427,968 -> 76,624

The step named above as missing is done. `nros_derive_entity_inventory_knobs`
now has a second caller: a composition DEFERRED to the top-level directory,
armed by `nano_ros_node_register` itself.

**Deferred to the TOP-LEVEL scope**, for the reason the support-library flush
documents one file over: the composer must run after EVERY registration, and
deferring to the current directory fires at the end of whichever package
registered first. **The deferred call takes no arguments** —
`cmake_language(DEFER ... CALL fn(arg))` delivers them empty — so the CLI path
travels as a `GLOBAL` property.

**The reconfigure is not optional here, and the trace proves why.** On Zephyr
the reader runs inside `find_package(Zephyr)`, before the app's CMakeLists body,
so no ordering lets one configure both compose and read its own answer. Issue
0991's machinery carries it across:

```
nros: NROS_XRCE_MAX_SUBSCRIBERS left to its crate default -- none derivable
nros: entity inventory DERIVED from 1 components -- 1 entities, 1 callback slots
nros: this image's entity inventory (standalone) changed after the readers of
      this pass had run, so cmake will run once more (issue 0991)
nros: NROS_XRCE_MAX_SUBSCRIBERS=1 DERIVED from this image's entity inventory
```

Four knobs derive on the second pass where none did before:
`NROS_XRCE_MAX_SUBSCRIBERS=1`, `NROS_EXECUTOR_MAX_CBS=1`,
`NROS_EXECUTOR_MAX_NODES=1`, `NROS_RMW_SUBSCRIBER_SLOTS=1`.

### Measured, on the image that motivated this

| | request | note |
| --- | ---: | --- |
| original | 427,968 | |
| after the MTU fix (issue 0968) | 309,696 | dead knob |
| **after this** | **76,624** | subscriber slots 8 -> 1 |

**82% off the session struct**, and the arena is 66,048 — so it is 10,576 short
of booting.

### What is left, and why it is not a five-minute follow-up

The remainder is `service_server_slots[4]` (17,536) and `service_client_slots[4]`
(4,160) on a LISTENER that has neither. Deriving the servers from
`NROS_DERIVED_MAX_QUERYABLES` — which already carries `ACTION_SERVER_QUERYABLES`,
so an action image is sized right — would take the request to about 63,472 and
the image WOULD boot.

It is not done here because of a collision worth stating rather than resolving
at speed:

* the derived value for this listener is **0**;
* `build.rs` PANICS below its minimum of 1 (`XRCE_MAX_SERVICE_SERVERS=0 too
  small`), because a zero-length array is not standard C;
* so the derivation must clamp to 1 — and silently rounding a knob up is
  precisely what issue 0827 forbids: *"This used to be silently rounded up to
  {min}, which reserved the memory anyway while reading as though the knob had
  been honoured."*

**The clamp was unnecessary, and the reasoning above was wrong on every count.**
Asked why a count with no user cannot be zero, each premise failed a test:

* *"zero-length arrays aren't standard C"* — true of ISO C, and irrelevant here.
  `slot_t arr[0];` MID-struct compiles on gnu11, c11 and gnu99; only
  `-pedantic` rejects it, and nothing in this build passes `-pedantic`.
* *the flexible-array-member error* this repo has hit came from an EMPTY define
  (`arr[]`), not a zero one (`arr[0]`). These are plain arrays mid-struct, not
  flexible array members. Two different errors that read alike.
* *nothing indexes slot 0 unconditionally* — every walk is
  `for (i = 0; i < MAX; ++i)`, which at 0 does not run.
* and issue 0827's rule argued FOR honouring zero, not against it: a clamp is
  precisely the silent rounding it forbids.

So the minimum is 0 for all three entity caps, and
`NROS_XRCE_MAX_SERVICE_SERVERS` joins the ladder on
`NROS_DERIVED_MAX_QUERYABLES` — the audited count, because an XRCE service
server is what a zenoh queryable is and that value already carries
`ACTION_SERVER_QUERYABLES`. Service CLIENTS stay stated: no audited aggregate
exists for them, only the raw count, which would under-size any action-client
image.

### The image fits

| | `sizeof(xrce_session_state_t)` |
| --- | ---: |
| original | 427,968 |
| after the MTU fix (issue 0968) | 309,696 |
| subscribers derived (8 -> 1) | 76,624 |
| **service servers derived (4 -> 0)** | **59,088** |

Against a 66,048-byte arena. `NROS_XRCE_MAX_SERVICE_SERVERS=0 DERIVED` is
honoured, the allocation succeeds, and the image boots past it — **zero heap
failures**, where every previous build died there.

One practical note for anyone re-measuring: Zephyr treats an existing `.config`
as user input, so changing a Kconfig DEFAULT does not reach a configured build
dir. The first attempt at this read `CONFIG_NROS_XRCE_MAX_SERVICE_SERVERS=4`
from a `.config` written before the sentinel existed and looked like the
derivation had failed. `west build -p always` is what proves it.


## Closed (2026-09-06) — the two things the derivation left open

The saving above is real and shipped. Two things were still wrong once it
landed, and both are fixed here.

### 1. The other factor of a slot was a DEAD KNOB on Zephyr

A slot is `XRCE_SUBSCRIBER_RING_DEPTH x XRCE_BUFFER_SIZE`. The derivation moved
the slot COUNT; `BUFFER_SIZE` was already forwarded; `RING_DEPTH` — the `32` in
`32 x 1024` — was not settable on Zephyr **by any means**, which is what this
issue's opening section suspected and nobody had confirmed.

Confirmed now, and the mechanism is issue 0460's, in the direction its gate does
not look:

* `nros-rmw-xrce-cffi/build.rs` has read `NROS_XRCE_SUBSCRIBER_RING_DEPTH` since
  phase-207, `book/src/reference/environment-variables.md` documented it with a
  default and a minimum, and `internal.h` invites `-DXRCE_SUBSCRIBER_RING_DEPTH`
  — so it read as live from three directions;
* there was **no `config NROS_XRCE_SUBSCRIBER_RING_DEPTH`** in `zephyr/Kconfig`
  and **no `_nros_resolve_knob`** line in `nros_cargo_build.cmake`;
* the reader's `$DOTCONFIG` fallback keys on `CONFIG_<name>`, so with no symbol
  it finds nothing, and the curated cargo environment forwards only what the
  resolver resolved (`NROS_RESOLVED_KNOBS` -> `_nros_knob_env`) — a shell export
  does not survive it;
* it also has no RFC-0049 rung: `XRCE_KNOBS` in
  `nros-platform-config/src/platform_config.rs` is `custom_transport_mtu` and
  `stream_history` only, so the descriptor was not a second carrier either.

Fixed: a Kconfig symbol (default **32**, the value every Zephyr XRCE image has
always compiled, so no image moves) plus the `_nros_resolve_knob` line.
Deliberately NOT on the derivable ladder — the count is a fact about the image,
the depth is a policy about bursts, and no entity inventory knows the second.

**The class was swept and is larger than this issue.** A read-side inventory
across the seven build scripts `check-kconfig-knob-forwarding` names finds ~10
more sizing knobs the resolver never forwards — `NROS_MAX_PARAM_NAME_LEN`,
`NROS_MAX_ARRAY_LEN`, `NROS_MAX_STRING_VALUE_LEN`, `NROS_MAX_BYTE_ARRAY_LEN`,
`NROS_RMW_MAX_BACKENDS`, `NROS_RMW_MAX_NODES`, `NROS_RMW_MESSAGE_INFO_SLOTS`,
`NROS_KEYEXPR_STRING_SIZE`, `NROS_SERVICE_TIMEOUT_MS`. **That is a lead, not a
verdict**: several of them have a second carrier (the params ones read RFC-0049
`rungs`, as `NROS_XRCE_CUSTOM_TRANSPORT_MTU` does), and which are genuinely
unreachable was not established here. It is not filed as an issue for exactly
that reason — a confident wrong root cause is worse than no filing (0859-0862).
Whoever takes it should re-run the sweep and check each knob's rung before
claiming any of them is dead:

```
python3 - <<'PY'
import re
cmake = open('zephyr/cmake/nros_cargo_build.cmake').read()
fwd = set(re.findall(r'_nros_resolve(?:_derivable)?_knob\(\s*([A-Z0-9_]+)', cmake))
kc  = set(re.findall(r'^config\s+([A-Z0-9_]+)', open('zephyr/Kconfig').read(), re.M))
for f in ['packages/rmw/zenoh/nros-zpico-build/src/runner.rs',
          'packages/rmw/zenoh/nros-rmw-zenoh/build.rs',
          'packages/core/nros-node/build.rs',
          'packages/rmw/xrce/nros-rmw-xrce-cffi/build.rs',
          'packages/core/nros-params/build.rs',
          'packages/rmw/cffi/build.rs',
          'packages/platform/nros-platform/build.rs']:
    for k in sorted(set(re.findall(r'"(NROS_[A-Z0-9_]+)"', open(f).read()))):
        if k not in fwd:
            print(('KCONFIG' if k in kc else '   -   '), k, f)
PY
```

`check-kconfig-knob-forwarding` asks whether every FORWARDED knob has a reader.
Nothing asks whether every READ knob is forwarded, which is why this hid.

### 2. Exceeding a derived cap was not LOUD

Before the derivation, exhausting a cap meant creating a ninth subscriber on an
image budgeted for eight. Now the cap IS the declared count, so the ordinary way
to hit it is to create an entity that was never declared — and all three
exhaustion sites returned a bare `NROS_RMW_RET_ERROR`, the same value a NULL
`backend_data` returns three lines up, with nothing logged. A correct
derivation had made its own failure mode common and left it undiagnosable.

Not silent truncation — it did fail the create — but nothing named the cap, the
knob, or the declaration the number came from.

Fixed, one shared spelling (`xrce_report_capacity_exhausted`, `session.c`):

* the code is **`NROS_RMW_RET_INVALID_CONFIG`**, issue 0468's, added for exactly
  this on the zenoh side — "a COMPILE-TIME capacity or configuration made the
  call impossible ... the remedy is a rebuild, never a different argument". Same
  failure one backend over, so the same code rather than a second spelling. It
  round-trips to `TransportError::InvalidConfig`;
* the diagnostic names the entity, its name, the cap, the knob, and *that the
  default is derived from the declaration* — which is the sentence a reader
  needs and could not have guessed;
* delivery is `nros_platform_log_write`, the "platform's printk-equivalent"
  `nros/rmw_ret.h` already names as where a backend logs at the failure site. It
  reaches every platform, native_sim included, where Rust `std` stdio is fatal
  (issue 0589).

The session allocation gained the same treatment: `xrce_report_session_alloc_failed`
prints `sizeof(xrce_session_state_t)` and the knobs that decide it. That is the
one request this whole issue is about, and for as long as issue 0968 was open it
presented as an anonymous boot failure because the number was never printed.

### The ring depth's saving, and how it was measured

`sizeof` on the host, with `xrce_subscriber_ring_entry` and
`xrce_subscriber_slot` copied VERBATIM out of `internal.h` by a script (no
retyping), at three depths:

| `XRCE_SUBSCRIBER_RING_DEPTH` | one slot | vs default |
| ---: | ---: | ---: |
| 32 (default) | 33,296 | — |
| 4 | 4,176 | -29,120 |
| 1 | 1,056 | -32,240 |

The depth-32 row reproduces this issue's DWARF measurement (33,296) exactly,
which is what makes the other two trustworthy. Carried onto the listener's
59,088-byte session: **29,968 at depth 4, 26,848 at depth 1** — computed by
subtraction from that measured total, not re-measured on a target ELF.

**The default does not move**, so no shipped image changes size today. What
changed is that a Zephyr image can now claim that saving at all.

### Not done

* The ~10 swept sibling knobs above — a lead, not a verdict; see the caveat.
* A gate for "every knob a build script reads is forwarded or has a rung".
  Writing one means adjudicating each of those ~10, which is the work above.
* No target build. The C changes compile clean under `cc-rs` with
  `nros_cc_flags::strict_decls` (`cargo check -p nros-rmw-xrce-cffi`, both new
  strings verified present in `libnros_rmw_xrce_c_inline.a`); the Zephyr image
  was not rebuilt on this host.
## The derivation was retired out from under these leaves (2026-09-06)

The section above closes this issue and says, in its own last line, "the Zephyr
image was not rebuilt on this host". Rebuilding it is how the following was
found, and it is the whole reason this survived: every claim above is about a
saving that, on `main`, no longer builds. Measured,
not inferred: `west build -p always` on `examples/zephyr/cpp/listener` with
`prj-xrce.conf` fails at CONFIGURE.

```
CMake Error at cmake/NanoRosNodeRegister.cmake:1129 (message):
  nano_ros_node_register(listener): ENTITIES was retired (phase-412).
Call Stack (most recent call first):
  cmake/NanoRosVerbs.cmake:471 (nano_ros_node_register)
  CMakeLists.txt:45 (nros_components_register_node)
```

phase-412 (`3016c16a9`, 2026-09-05 21:59Z) retired `ENTITIES` and moved the
declaration to a `<bringup>/launch/<stem>.contract.yaml` sidecar. It migrated
the six `examples/workspaces/cpp` packages. It did **not** migrate the six
`examples/zephyr/cpp` leaves this issue had taught to declare 41 hours earlier
(`0768e0c59`), and those six are the only remaining `ENTITIES` callers in the
tree.

**The blast radius is not six files.** The error is raised before the RMW
matters, so it takes all 19 `examples/fixtures.toml` rows over
`examples/zephyr/cpp/` — zenoh, xrce and cyclonedds alike. Confirmed by
building `cpp/talker` against `prj-zenoh.conf` and getting the same error.

**And nothing said so**, because those leaves are configured only by
`build-test-fixtures`, which needs the Zephyr SDK and west and therefore runs on
no `pull_request` lane. The retirement's `FATAL_ERROR` is the right mechanism
reporting to nobody — the shape of issues 1025 and 1070.

### The replacement has no shape that fits a standalone application

`nros_derive_entity_inventory_knobs` has two callers. Only
`NanoRosEntry.cmake:1028` supplies `MODEL`, and it sits inside
`_nros_entry_invoke_codegen`, reachable only from the `LAUNCH`/`MODEL` arm of
`nano_ros_entry()`. These leaves are standalone Zephyr applications: no
`nano_ros_entry`, no launch file, no bringup, no `system.toml`, so no contract
sidecar and no SystemModel.

The deferred standalone composer this issue added (`d11a14f5f`) still exists at
`NanoRosNodeRegister.cmake:144`, and still runs — it just cannot produce
anything any more, because it passes no `MODEL` and `nano_ros_node_register` no
longer emits an `entities` metadata key. Every component reads `Absent`, and the
refusal it returns said, in as many words:

> Add `ENTITIES ...` to each `nano_ros_node_register()` above -- `ENTITIES NONE`
> for a component that really creates none.

**The one remedy the diagnostic named was the one thing the caller could not
do.** A message that outlives the mechanism it describes aims the next reader at
a wall, and this one did it for every standalone image in the tree. Fixed here.

### Measured, on the listener that motivated this issue

`sizeof(xrce_session_state_t)` from each image's own DWARF
(`gdb -batch -ex "ptype /o xrce_session_state_t" zephyr.elf`), all three built
`west build -p always -b native_sim/native/64` on one host, one tree:

| state | request | outcome |
| --- | ---: | --- |
| `main` as it stands | — | **configure `FATAL_ERROR`** |
| `ENTITIES` deleted, nothing else | **309,696** | builds; dies at boot |
| this change | **54,928** | boots clean |

The middle row is the naive way to complete phase-412's sweep, and it is worth
stating because it looks like a fix. It is the pre-issue-1033 number exactly,
and the image says so itself:

```
nros: HEAP EXHAUSTED: request 309696 bytes, arena 66048 bytes, caller 0x42c11e
```

Against the same run of the image built here, which prints no such line.

Across the leaves rebuilt to check the multipliers:

| leaf | subscriber / server / client slots | `sizeof` |
| --- | --- | ---: |
| `cpp/talker` | 0 / 0 / 0 | **21,632** |
| `cpp/listener` | 1 / 0 / 0 | **54,928** |
| `cpp/action-client` | 1 / 0 / 3 | **58,048** |

All three boot with no heap message. 54,928 is also the number
`nros-rmw-xrce-cffi/build.rs` has claimed in a comment since `28d916c66` while
the tree delivered 59,088 — the service-client cap was never stated, so the
comment described an end state nothing reached. It does now.

### Divergence from this issue's own preference, and why

This issue chose option (2), DERIVE, over option (1), "state the caps per
example", calling the latter "twenty hand-picked numbers". This change states
them. Not because the reasoning changed — because the mechanism (2) depended on
was retired, and phase-412 left nothing in its place for an image that runs from
no launch file. phase-412's own `FATAL_ERROR` says what such an image gets:

> A system with no contract yet keeps its configured `NROS_EXECUTOR_MAX_CBS`,
> exactly as it did before phase-403.

So the numbers are stated in each leaf's `prj-xrce.conf`, and they are not
hand-invented: each is the leaf's own retired `ENTITIES` line put through the
same multipliers the derivation uses — `ACTION_SERVER_QUERYABLES` (3) for the
action server, `ACTION_CLIENT_SUBSCRIPTIONS` (1) plus the three service clients
`RawActionClientSpec` documents for the action client. Every conf carries the
declaration it came from and the arithmetic.

This is a HOLDING POSITION, not the campaign's answer. The campaign's answer is
a declaration path for an image with no launch file, and that is a change to the
contract/SystemModel seam rather than something to bolt on at a call site —
the same conclusion this issue reached about the composition gap, one mechanism
later.

### The class, and the gate

`check-retired-cmake-keywords` (fast line): a keyword any cmake verb retires
must appear in no caller. The retired set is DISCOVERED from the
`message(FATAL_ERROR "<fn>(...): <KEYWORD> was retired|removed ...")` guards
under `cmake/`, so the next retirement arms the gate by itself, and a guard
whose wording moves makes the gate BLIND rather than silently permissive — that
case fails. It finds `ENTITIES` (`nano_ros_node_register`) and `HOST`
(`nano_ros_entry`) today; run against the tree before this change it names all
six leaves and their line numbers.

Sweep command:

```
python3 scripts/check-retired-cmake-keywords.py
```

### A lead, measured but not fixed here

`floor1` in `entity_inventory.rs` (issue 1015 — "a derived pool that sizes a C
array has a floor of ONE") is applied at the PRODUCER, so
`NROS_DERIVED_MAX_SUBSCRIBERS` and `NROS_DERIVED_MAX_QUERYABLES` are never zero.
Issue 1015's evidence is entirely about zenoh: `zpico.c`'s
`queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]` at zero transmitted nothing
for 15 seconds on the reference island. The XRCE consumer reads the same two
values, and this issue measured the opposite there — 0 is legal, honoured, and
boots. So an XRCE image that DERIVES (rather than states, as these leaves now
do) pays 33,296 bytes for a subscriber ring and 4,384 for a server slot it
declared none of: the floor belongs to the zenoh pool, not to the fact. Latent
for these six leaves while they state their caps; live for any XRCE image that
derives. Not fixed here because moving the floor to the consumers is a change to
both the Zephyr ladder and `leaf_entity_env.rs`, and it wants its own
measurement.
