# Phase 206 — The user's own RMW config reaches the backend

*(Filename keeps the original `multi-homing-transport-interfaces` slug; the phase
NUMBER is the identity. The scope was rewritten 2026-09-04 — see "Rescoped".)*

**Goal.** Give nano-ros the ROS 2 experience: **the OS/board owns the devices,
and the user configures the middleware in the middleware's own language.** On
ROS 2 a user writes `CYCLONEDDS_URI` or a zenoh config naming an IP or a device,
and the application code knows nothing about NICs. nano-ros should be the same —
Ethernet/serial/CAN brought up by the board before ROS exists, and the user's
Cyclone XML or zenoh config attached on top, parsed by the backend that owns the
format.

**Multi-homing is then not a feature of nano-ros at all.** It is
`<General><Interfaces>` in a Cyclone config the user wrote. That is the whole
reason this phase was rescoped: the old plan built a nano-ros abstraction over
something the backends already have words for.

**Status (2026-09-05). W1, W2, W3 and W5 LANDED; W4 landed
(issue 1067 resolved); W6 needs no work by design.** The phase is functionally complete: a user writes
their backend's own config, the bytes reach the backend's own parser, and
nano-ros models no NICs. What remains is W4's decision, which is about board API
rather than configuration, and W2's RTOS-image acceptance, which needs a fixture
row and a tier-2 run.

*The status below is the 2026-09-04 record of how the phase got here.*

**Status (2026-09-04). Proposed, and its foundation is GONE.** Still unstarted
as proposed 2026-05-29 — but the "Landed" section below is no longer true, and
the way it stopped being true is the useful part.

**172.K.7's half really did land. Three separate retirements took most of it
back, and none of them knew it.**

| landed 2026-05-29 | removed by | when |
| --- | --- | --- |
| generator emits `c.set_interfaces(&[…])`, + the test `multi_homed_interfaces_emit_set_interfaces_call` | `11a00b0f8` — #202, "retire the dead standalone-package pipeline" (it deleted `orchestration/generate.rs` whole) | 2026-07-16 |
| `NanoRosConfig.cmake` + `NanoRosReadConfig.cmake` parsing `interfaces` → `NROS_CONFIG_INTERFACES` | `f473b78b4` / `53f2ddce7` — phase-212 M-F.10, "retire cmake codegen of `NROS_APP_CONFIG`" | phase-212 |
| the `config.toml` readers those parsers lived in | phase-256 W9, the legacy `config.toml` reader removal | — |

Each retirement had a good local reason and each was right on its own terms.
Together they dismantled this phase one plank at a time, and nobody updating
those commits was looking at this doc.

**What actually survives, verified on `main` 2026-09-04:**

* `PlanTransport.interfaces: Vec<String>` — `orchestration/plan.rs:684`
* its ethernet/wifi-only validation — `plan.rs:854-856`
* its copy into the IR — `model_ingest.rs:1457`
* three of the four tests — `plan.rs:1022, 1036, 1052`, all **parse/validate
  only**; none tests emission, because there is no emission
* `BoardTransportConfig::set_interfaces` — `nros-platform/src/board/config.rs:108`,
  a default no-op with **zero overrides and zero call sites tree-wide**

So the value parses, validates, reaches the IR, is written into
`<bake>/nros-plan.json` — **and is read by nothing.** `nros explain` cannot even
print it (`explain.rs:384-396` prints `ip`/`device`/`baudrate` only).

**This phase is now an instance of the class it would have to fix.** phase-349
names the shape while flagging its sibling: `NROS_NETSTACK` is *"emitted too …
and nothing reads it — the same declared-but-unread shape."* `set_interfaces` is
the second live instance. Any revival that adds another declaration without a
consumer repeats the defect.

*Original status, 2026-05-29:* Proposed. Extracted from Phase 172.K.7 (archived)
— its schema + plumbing half landed; this phase is the deferred
**wire-emission** half.

**Priority.** P2 — no shipped capability depends on it; meaningful only once a
multi-NIC target exists. Cyclone is the one backend where it's both meaningful
*and* testable today.

**Depends on.** Phase 172.K.7 schema/plumbing (landed); Phase 175.A (native
Cyclone build path) for the Cyclone config seam (206.3).

## Rescoped 2026-09-04 — from "multi-homing wire emission" to "config passthrough"

The original phase asked *"how does nano-ros express a NIC list and lower it to
each backend?"* Every answer to that question requires nano-ros to own a second
vocabulary for interfaces — plus a resolver, a gate, and a per-platform story
that only Linux can satisfy. `"eth0"` is a name in Linux's namespace; Zephyr's
`net_if_get_default()` cannot be named at all, and smoltcp has exactly one
`Interface` and no names. A portable `interfaces = ["eth0","eth1"]` is issue
0623's shape one layer up: a value authored in one vocabulary and resolved in
another, with nothing checking they agree.

**The principle that replaces it:**

> **nano-ros transports the user's backend config VERBATIM; it does not
> re-model it.** Devices are the board's job and are up before ROS exists. The
> middleware's configuration is the middleware's own format, parsed by the
> middleware's own parser.

This is what ROS 2 does, and it is why a ROS 2 user never writes an interface
list into their node.

## Overview

`[[transport]].interfaces = ["eth0", "eth1"]` already parses → `PlanTransport.interfaces:
Vec<String>` and validates (ethernet/wifi only). **The rest of this paragraph
was true in May and is not now** — there is no generator emitting
`set_interfaces` and there are no CMake parsers; see the status table above.
**Nothing binds an actual NIC**, and the seam is not merely inert, it is
unreachable: the only surviving consumer of the parsed value is the serializer
that writes it into `<bake>/nros-plan.json`.

Three blockers gate real binding, in dependency order: a multi-endpoint runtime
`SessionSpec` (206.1), the per-backend mapping (206.2 zenoh decision, 206.3
Cyclone `<Interfaces>` emission), and a multi-NIC target to verify against
(206.4). Distinct from Phase 172.K.5 (multi-domain = *segregate* sessions); this
is *merge* — one session, many NICs.

Design: [`docs/design/0004-configuration-and-transports.md`](../design/0004-configuration-and-transports.md)
("Two axes" taxonomy, cases B/C).

## Landed (Phase 172.K.7 schema + plumbing, 2026-05-29) — MOSTLY SINCE REMOVED

Read this list as the record of what K.7 built, not as the state of the tree.
The status table above says what took each piece back; **✓** survives, **✗**
does not.

- **✓** `PlanTransport.interfaces: Vec<String>` (serde default, skip-when-empty);
  `validate_transports` rejects it on serial/can (ethernet/wifi only).
  (`plan.rs:684`, `:854-856`)
- **✗** Generator emits `c.set_interfaces(&[…])` (mirrors `set_ssid`/`set_mac`),
  backed by a default-no-op `BoardTransportConfig::set_interfaces` seam.
  — the generator went with `orchestration/generate.rs` in `11a00b0f8`. The
  no-op trait method survives with no callers; `set_ssid` and `set_mac` lost
  their emitters in the same commit, so all three are dead together.
- **✗** Both CMake parsers (`NanoRosConfig.cmake`, nros-c `NanoRosReadConfig.cmake`)
  accept `interfaces = ["eth0","eth1"]` (legacy scalar `interface` mirrored in) →
  `NROS_CONFIG_INTERFACES` list var.
  — both files are gone; `NROS_CONFIG_INTERFACES` appears in no source or build
  output today, only in this doc and archived phase-172.
- **✓/✗** Tests: `transport_tests::{multi_homed_interfaces_parse_and_validate,
  interfaces_absent_round_trips_empty_and_skips_serialization,
  interfaces_are_ethernet_wifi_only}` **survive** (`plan.rs:1022, 1036, 1052`) —
  `multi_homed_interfaces_emit_set_interfaces_call` **does not**, deleted with
  `tests/orchestration_e2e.rs` in `11a00b0f8`.

> **Anyone reviving this phase should re-plan rather than resume.** 206.2 and
> 206.3 below name a "generator `set_interfaces` seam" as the thing to emit
> from, and that seam does not exist — an hour spent looking for it is the
> predictable cost of leaving this section unmarked. The surviving architecture
> reaches a backend by other routes (the RFC-0049 knob ladder already carries
> `NROS_BOARD_TOML` into `nros-zpico-build`), and the per-backend picture has
> changed too: zenoh-pico is structurally a client with one locator (peer mode
> is refused, `MULTICAST_TRANSPORT = false`), which bears directly on 206.2's
> open decision. None of that is decided here; this note only records that the
> plan below is written against a tree that no longer exists.

## What blocks it today — two measured defects

### Defect 1 — Cyclone: the user's config REPLACES the tuned baseline

`packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/session.cpp:310-313`:

```cpp
const char* user_uri = env_lookup("CYCLONEDDS_URI");
const char* cyc_config = (user_uri != nullptr && user_uri[0] != '\0') ? user_uri
                         : (kKconfigCycloneConfig[0] != '\0')         ? kKconfigCycloneConfig
                                                                       : kEmbeddedCycloneConfig;
dds_entity_t domain = dds_create_domain(domain_id, cyc_config);
```

A three-way ternary: exactly one string wins. Set `CYCLONEDDS_URI` and the whole
baked baseline is **gone** — `<Threads>` stack sizes (64 KiB for `dq.builtins`,
`recv`, `dq.user`), `<Sizing>` receive buffers, `<Internal>
<MultipleReceiveThreads>false`, and the platform's `AllowMulticast` choice
(`session.cpp:89-183`).

On FreeRTOS and ThreadX those stack sizes are load-bearing. **So a user doing the
exact thing this phase exists to support — attaching their own Cyclone config —
silently detunes their RTOS image today.** That is a live bug, not a missing
feature.

And it is gratuitous: **Cyclone composes natively.** Verified in the pinned tree
we link (`third-party/dds/cyclonedds`, `src/core/ddsi/src/ddsi_config.c:2538-2570`):

```c
copy = ddsrt_strdup (config);
cursor = copy;
while (*cursor && (isspace ((unsigned char) *cursor) || *cursor == ','))
  cursor++;
while (ok && cursor && cursor[0]) {
    if (tok[0] == '<')  qx = ddsrt_xmlp_new_string (tok, cfgst, &cb);   /* inline XML */
    else if ((fp = config_open_file (tok, &cursor, domid)) == NULL) ...  /* or a file */
```

Every comma-separated token parses into the **same `cfgst`**. Upstream's own
tests rely on it (`"${CYCLONEDDS_URI}${CYCLONEDDS_URI:+,}<Discovery>…"`).

**And multi-homing is already expressible in Cyclone's own schema**, which is
the fact the whole rescope rests on. `src/core/ddsi/include/dds/ddsi/ddsi_cfgelems.h:79-88`,
in the same pinned 0.10.5 tree:

```c
static struct cfgelem interfaces_cfgelems[] = {
  GROUP("NetworkInterface", NULL, network_interface_attributes, INT_MAX,
    MEMBER(network_interfaces), ...
```

`INT_MAX` is the multiplicity — **an unbounded `<NetworkInterface>` list**, each
carrying `name` / `address` / `autodetermine` / `priority`. The `<Interfaces>`
group's own description says it: *"Multiple interfaces can be specified with an
assigned priority. The list in use will be sorted by priority."* (`:139-146`).

So the entire feature phase-206 originally set out to build already exists,
fully, in the config language of the backend that has to implement it. nano-ros
does not need to express it, lower it, or resolve it — only to stop discarding
the user's config and to carry the bytes to the parser.

### Defect 2 — zenoh: there is no user config surface off hosted-Rust

Two independent cut points:

* `packages/rmw/zenoh/nros-rmw-zenoh/src/shim/session.rs:370` — the `ZENOH_*`
  env block is `#[cfg(feature = "std")]`, so **no embedded target reads it**.
* `packages/rmw/cffi/src/rust_adapter.rs:517` — the C boundary builds
  `RmwConfig { …, properties: &[] }`, hardcoded. So a **C or C++ entry gets
  nothing even on Linux**, and `rmw_session_options_t` is passed `nullptr` by
  every caller in the tree anyway (`packages/rmw/cffi/src/lib.rs:1884-1893`).

Only a hosted Rust caller building `TransportConfig` by hand can set `listen`,
`multicast_scouting` or `multicast_locator` — all of which zenoh-pico supports
and the shim already maps (`zpico-sys/c/zpico/zpico.c:1213-1237`).

### Device bring-up — WORKS, but not through the contract that describes it

**This section said "already right" until W4 measured it, and that was wrong.**
The boot order at `nros-platform/src/board/entry.rs:15-20`

    init_hardware -> init_transport -> wait_link_up -> open executor -> setup -> spin

is a **doc comment, not code**. Measured 2026-09-05:

| symbol | impls | production callers |
| --- | ---: | ---: |
| `BoardEntry::run` / `run_with_deploy` | **12** of 17 board crates | live — the real path |
| `NetworkWait::wait_link_up` | 1 (`nros-board-zephyr`) | **0** |
| `TransportBringup::init_transport` | **0** | **0** |

Devices *are* brought up — inside each board's own `run`, or the family helper
it delegates to (`nros_board_freertos::run_entry`, which carries the board
`Config` with MAC/IP/netmask/gateway). So the premise this phase rests on holds:
**the board owns the devices and they are up before ROS exists.** What does not
hold is that the two mixin traits are how it happens.

Filed as [issue 1067](../issues/1067-board-device-contract-traits-have-no-callers.md)
and **resolved by W4 below**: the two mixins are gone, and the contract is now
`BoardEntry::run` — the one that runs.

## Can we use the backends' NATIVE parsers? — measured 2026-09-04

The answer differs per backend, and the difference decides the design.

### Cyclone — YES, completely

`dds_create_domain(domain_id, config)` accepts **inline XML**: any comma token
beginning with `<` goes to `ddsrt_xmlp_new_string`, Cyclone's own parser
(`ddsi_config.c:2549-2551`). So a baked file's bytes can be handed over verbatim
as one token and **nano-ros parses no XML at all**. Composition and native
parsing come from the same mechanism.

### zenoh-pico — upstream's own model has three rungs and NO file format

**Checked against upstream's documentation, not inferred.** zenoh-pico's manual
(`zenoh-pico.readthedocs.io/en/latest/config.html`) defines exactly three
configuration categories:

| rung | what | who owns it here |
| --- | --- | --- |
| **Run-time options** — *"the primary configuration method"*, set with `zp_config_insert(config, KEY, value)` | mode, connect, listen, scouting, TLS, session zid | **the user** — and this is the surface nano-ros must expose |
| **Manual compile-time options** — edit `config.h.in` | session ID length, protocol parameters | upstream; not ours to touch |
| **Generated compile-time options** — CMake flags | buffer sizes, transport timeouts, feature toggles | **nano-ros already does this correctly** via `nros-zpico-build` and RFC-0049's `builtin < platform toml < board toml < env` ladder |

**There is no configuration file format, and that is a design position rather
than a gap.** zenoh's JSON5/YAML config files are documented for **`zenohd`**,
the router — `zenoh.io/docs/manual/configuration/` does not mention zenoh-pico
at all. Upstream's own examples do exactly what the manual says:
`z_config_default()` then `zp_config_insert(…, Z_CONFIG_CONNECT_KEY, …)` from
CLI args (`examples/unix/c11/z_pub.c:46-49, 133-139`).

So nano-ros should **expose rung 1 and stop there** — not invent a fourth rung,
and not duplicate rung 3 into the user-facing surface. The verbatim unit for
zenoh is a **`key = value` list whose key names are upstream's own**, which is
what makes a user's zenoh knowledge transfer.

### zenoh-pico — and therefore no document to parse

zenoh-pico's entire configuration API is three functions
(`include/zenoh-pico/api/primitives.h:360-385`):

```c
z_result_t  z_config_default (z_owned_config_t *config);
const char *zp_config_get    (const z_loaned_config_t *config, uint8_t key);
z_result_t  zp_config_insert (z_loaned_config_t *config, uint8_t key, const char *value);
```

The key is a **`uint8_t`**, not a string: ~22 numbered constants
(`include/zenoh-pico/config.h:224-314`, `Z_CONFIG_MODE_KEY 0x40` through the TLS
block). There is no `zc_config_from_str`, no config file reader, and **no JSON5
parser** — `include/zenoh-pico/utils/json_encoder.h` is an *encoder* for the
admin space, inherited from upstream, not a config parser.

**JSON5 is zenoh-rs / zenoh-c's format, and this tree has no zenoh-rs backend** —
every crate under `packages/rmw/zenoh/` is zenoh-pico (`zpico-sys`,
`nros-rmw-zenoh`, `zpico-*`).

So: **adding a JSON5 parser would mean nano-ros owning a parser for a format its
own backend does not speak.** That is the exact mistake the principle forbids,
wearing a friendlier hat. zenoh-pico's *native* config language IS the numbered
key/value table, so the verbatim unit for zenoh is a **flat `key = value` file**
whose keys are the zenoh-pico key names the shim already maps. The work for
zenoh is therefore **unblocking a path that exists**, not building one.

*If a zenoh-rs backend is ever added, JSON5 becomes its native format and this
decision should be revisited for that backend only — never retrofitted onto
pico.*

## Work Items

### 206.W1 — Cyclone composes; the baseline always survives — **DONE 2026-09-05**

Prerequisite for 206.3, which wants to *add* a `<General><Interfaces>` fragment
to whatever Cyclone config an image already has. There was no way to add one:
`session.cpp` chose exactly one of three sources with a three-way ternary —
`CYCLONEDDS_URI`, else `CONFIG_NROS_CYCLONE_CONFIG_XML`, else the baked
baseline — so any override *replaced* the baseline instead of extending it.

What that cost, silently: the baked `<Threads>` stack sizes (64 KiB for
`dq.builtins` / `recv` / `dq.user`), the `<Sizing>` receive buffers, the
`<Internal><MultipleReceiveThreads>false`, and the platform's `AllowMulticast`
policy. On FreeRTOS and ThreadX the stack sizes are why `recv` survives a real
ROS payload (ddsrt defaults a thread to 1 KiB there), so "point me at a
different peer" also meant "and reinstate that stack overflow" — in a config
that mentions neither.

- [x] `packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/cyclone_config.hpp` — the
      baked baseline moves here verbatim, beside `compose_cyclone_config()`:
      joins the non-empty sources with `,` into a fixed 4 KiB buffer, and
      **fails loud** (session creation returns `NROS_RMW_RET_ERROR`) rather than
      truncating. A clipped config is unterminated XML, which Cyclone would
      report as a parse error against a string nobody wrote.
- [x] `session.cpp` composes `baseline, kconfig, user` in that order. The
      platform `#if` around `dds_create_domain` is unchanged — hosted POSIX and
      plain Zephyr still let Cyclone's own loader read `CYCLONEDDS_URI`.
- [x] **Precedence verified, not assumed.** CycloneDDS splits the config string
      on commas and parses every token into ONE `cfgst`
      (`ddsi_config.c:2505-2593`); opening a `<CycloneDDS>` root shifts
      `cfgst->source` left one bit, and `do_update` resets an already-set scalar
      when `source > n->sources`. **Later tokens win**, so baseline-first is the
      correct order. Measured against the pin by dumping the resolved config:
      `baseline,user` → `AllowMulticast=false` (user's) with the baseline's
      thread stacks intact; `user,baseline` → `AllowMulticast=spdp` (baseline's).
      One documented exception: `<Thread>` is a *list* element, so a later token
      appends rather than overrides and `lookup_thread_properties` takes the
      first match — a user cannot override a stack size the baseline names.
- [x] Test: `tests/cyclone_config_compose.cpp` (ctest
      `nros_rmw_cyclonedds_config_compose`). Composes the real baseline with a
      user fragment stating only `<Discovery>`, brings a domain up on it, and
      reads the RESOLVED values back out of Cyclone's `config` trace category —
      asserting the five baked thread stacks, the 64 KiB receive buffer and
      `MultipleReceiveThreads=false` all survive, *and* that the user's peer and
      `MaxAutoParticipantIndex` apply. Carries its own negative control (the
      user fragment alone, i.e. what the ternary passed), so the assertions
      cannot be met by Cyclone's defaults. Against the pre-fix selection it
      fails 10 checks, 4 of them the "did not survive" ones.

**The one thing this does NOT buy, stated because it would otherwise be assumed:**
`<Thread>` is a LIST element (multiplicity 0), so a later token APPENDS an entry
rather than replacing one, and `lookup_thread_properties`
(`q_thread.c:280-288`) returns the FIRST match. Measured: `recv` at 64 KiB then
`recv` at 128 KiB yields two entries and the 64 KiB one is used. So a user
**cannot override a stack size the baseline names**. That is safe in the
direction this phase cares about — the baseline survives — but it is not "the
user always wins", and anyone who needs a bigger Cyclone thread stack has to
change the baseline, not their config.

### 206.W2 — one config-passthrough seam, present on every target — **DONE 2026-09-05**

The user writes the document ROS 2 documents; the build bakes its bytes; the
backend's own parser reads them. nano-ros parses nothing.

- [x] **Where the file lives — convention, not declaration.**

      <bringup>/rmw/cyclonedds.xml     authored, tracked, reviewed
      <bringup>/rmw/zenoh.conf         (W3)
      <bringup>/config/*.yaml          GENERATED, gitignored (phase-330 W4.a)

      `<bringup>/config/` already means "build artifact, never committed", so an
      authored file there would put the one thing a user must edit inside the one
      directory the tree calls disposable. `rmw/` sits beside `launch/`, the
      existing precedent for authored, typed, per-bringup input. No `system.toml`
      key names it: the active backend is already declared (`[system] rmw`), so
      the build knows which name to look for, and a second declaration is a
      second thing to keep in sync — RFC-0087's "derived from convention" one
      axis over.
- [x] **The bake.** `nros_bake_rmw_user_config()`
      (`cmake/NanoRosRmwUserConfig.cmake`) reads the file at configure time and
      writes `nros_<backend>_user_config.h` holding the bytes as a raw string
      literal. `CMAKE_CONFIGURE_DEPENDS` on the source, so editing the config
      re-configures — without it the image keeps the previous bytes, which is
      issue 0196's museum-binary class applied to a config file. The generator
      refuses a document containing the raw-string terminator rather than
      mangling it.
- [x] **A VARIABLE, not a `#define`** — and this is the bug the test caught. A
      raw string literal **cannot span physical lines inside a macro
      definition**: the preprocessor ends the directive at the first unescaped
      newline, so every multi-line config failed to compile with "unterminated
      raw string" pointing at the generated file. The payload is a
      `constexpr const char*`; only the PRESENT marker is a macro, because that
      is the part `#ifdef` has to see.
- [x] **Precedence.** `baseline, kconfig, bringup, env` — lowest first. The
      bringup outranks Kconfig because it is what the user authored for this
      system; the environment outranks the bringup because that is how a ROS 2
      user overrides a shipped config at run time without rebuilding, and taking
      that away would invert this phase's goal.
- [x] **Hosted reach.** The domain-creating block was gated to
      FreeRTOS/ThreadX/native_sim, because hosted POSIX and plain Zephyr let
      Cyclone's own loader read `CYCLONEDDS_URI` — which is the ROS 2 experience
      and must not be intercepted. But a bringup that SHIPS a config has stated
      one for every target it builds, and a file that silently applies on the
      RTOS and not on the host is worse than no file. So a baked config — and
      only a baked config — makes the hosted path compose too. An image with no
      bringup config keeps the old hosted behaviour byte for byte, which the
      test's negative control pins.
- [x] **Acceptance met.** `tests/cyclone_user_config.cpp` (ctest
      `nros_rmw_cyclonedds_user_config`) drives the REAL generator against a
      fixture laid out as a bringup, then asserts the baked bytes are
      **byte-identical to the file on disk** — not a copy of themselves — and
      that composing them with the baseline keeps the 64 KiB thread stacks.
      Carries a negative control: with no user config the composition must BE
      the baseline, unchanged. Against the pre-W2 state (baked header never
      reaching the backend) it fails 5 checks; after, `26/26` ctest green.

**The RTOS half — measured 2026-09-06, and it found the seam UNWIRED.**

The previous note said this needed "a fixture row and a tier-2 run". Adding the
row found something the row could not have assumed: **`nros_bake_rmw_user_config`
had exactly one caller in the whole tree, the backend's own ctest CMakeLists.**
No image build called it, so `session.cpp`'s `__has_include` guard could never
fire and no bringup could ship a config. The mechanism was complete, documented,
tested on the host lane — and unreachable from any image. Phase-349's
declared-but-unread class, one layer up: the CONSUMER existed and the PRODUCER
was never invoked.

Now wired, in `zephyr/cmake/nros_system_generate.cmake` — the function that
resolves the bringup and already attaches the include dir, so the generated
header reaches the backend's TUs by the same route `system_config.h` does. It
could not go in `nros_rmw_cyclonedds.cmake`: `zephyr/CMakeLists.txt` includes
that at line 162, long before any leaf calls `nros_system_generate`, so the
backend module could only ever read a bringup left in the cache by a PREVIOUS
configure.

Fixture row `west_bringup_zephyr_cyclone_user_config` (west-build,
native_sim/native/64), sharing `multi_pkg_workspace_zephyr`'s bringup with the
existing zenoh row.

**WHAT IT PROVES:**

- the bake fires on an RTOS configure —
  `nano-ros: baked .../demo_bringup/rmw/cyclonedds.xml -> .../nros_cyclonedds_user_config.h`
- the header holds the SSoT's bytes **byte-identically** (extracted from the raw
  string literal and compared to the file: `True`)
- `session.cpp` **compiles those bytes into its object on the RTOS target** —
  `strings session.cpp.obj | grep -c cdds.io/config` = 1
- the negative control holds: the same bringup built with `prj-zenoh.conf` bakes
  NOTHING (`rmw=zenoh`, no bake line, `rc=0`), because `rmw/` ships
  `cyclonedds.xml` and no `zenoh.conf` — the active backend picks the file

**WHAT IT DOES NOT PROVE, and this is the honest limit.** The bytes are in the
OBJECT, not in the IMAGE: `strings zephyr.elf | grep -c ddsi_` is **0**. The
whole Cyclone backend is garbage-collected out, because this fixture's
`zephyr_app/src/main.c` is a 212.H.1 stub that never creates a node, and the link
runs `--gc-sections`. So this row is a COMPILE-stage measurement. "The same file
works byte-identically hosted and embedded" is now proven as far as the
compiler; it is not yet proven as far as a running participant.

**BOTH bringup paths now bake.** There are two, and neither covers the other:

| path | who uses it | bakes? |
| --- | --- | --- |
| `nros_system_generate()` (212.H.1 shim) | the test fixtures only | yes |
| `nano_ros_entry(... BRINGUP ...)` (`cmake/NanoRosEntry.cmake`) | every real workspace image | yes |

No in-tree EXAMPLE calls `nros_system_generate` — its only callers are
`multi_pkg_workspace_zephyr` and `zephyr_self_pkg` — so wiring the shim alone
left the mechanism reachable from the fixtures and from nothing a user builds.

**The include goes on the BACKEND target, not on the entry.** `session.cpp` is
the TU that reads the generated header and it compiles into `nros_rmw_<backend>`,
a different target from the executable. Attaching the directory to the entry
alone configures cleanly, compiles cleanly, and bakes nothing into the image —
the exact failure this work item exists to make impossible. Where the backend is
IMPORTED (an installed NanoRos, which cannot be recompiled) the entry path warns
rather than staying silent: a config that does not apply must not look like one
that did.

**PROVEN ON AN IMAGE THAT ACTUALLY LINKS CYCLONE** —
`workspace-cpp-native-cyclonedds` (`examples/workspaces/cpp`, `[image.native_cyclonedds]`),
whose bringup now ships `src/demo_bringup/rmw/cyclonedds.xml`:

```
-- nano-ros: baked .../src/demo_bringup/rmw/cyclonedds.xml -> .../nros_cyclonedds_user_config.h
-- nano-ros: cyclonedds user config reaches nros_rmw_cyclonedds (entry native_cyclonedds_entry)
```

and in the linked executable:

| check | result |
| --- | --- |
| the SSoT's exact bytes, contiguous, in the ELF | **True** |
| `strings \| grep -c 'cdds.io/config'` | 1 |
| `strings \| grep -c ddsi_` (Cyclone really linked) | 41 |

Negative control, same workspace: `workspace-cpp-native` (zenoh) builds `rc=0`
with **zero** bake lines — `rmw/` ships `cyclonedds.xml` and no `zenoh.conf`, so
the active backend picks the file and an image whose backend has no config
compiles with no cmake participation at all.

**What the RTOS row adds, and what it still does not.** The Zephyr row above is a
COMPILE-stage measurement: the bytes reach `session.cpp.obj` but not
`zephyr.elf`, because that fixture's `main.c` is a 212.H.1 stub that never
creates a node and the link runs `--gc-sections`. An RTOS image that links
Cyclone AND carries a bringup config is the remaining gap — smaller than it was,
and no longer blocking the mechanism, which is now proven end to end on a real
image.

A conf-file trap found on the way, recorded because it fails GREEN: omitting
`CONFIG_CPP=y` leaves `NROS_RMW_CYCLONEDDS`'s `depends on NET_SOCKETS &&
POSIX_API && CPP` unmet, and Kconfig does not complain — it silently keeps the
default and builds ZENOH while every conf file in the command says Cyclone
(`rc=0`, zenoh-pico TUs in the log, `rmw=zenoh` in the bake line).

### 206.W3 — zenoh's run-time rung is exposed, complete, and fails loud — **DONE 2026-09-05**

A defect in its own right, and the thing that makes zenoh multi-endpoint
configuration expressible at all. zenoh-pico's manual names
run-time options — `zp_config_insert(config, Z_CONFIG_<X>_KEY, value)` — as *"the
primary configuration method"*, and the pico client has **no config-file format**
at all (the JSON5/YAML in the docs is `zenohd`, the router). So that key map is
the whole configuration surface, and three things were wrong with it:

1. **The C boundary carried nothing.** `create_session_trampoline` read
   `let _ = options;` and built `properties: &[]`, and `CffiRmw::open` dropped
   `RmwConfig::properties` on the way in. A C or C++ entry could state no
   transport configuration on any platform; only a hosted Rust caller building an
   `RmwConfig` by hand could reach `listen`, `multicast_scouting` or a TLS
   certificate. `rmw_session_options_t` now carries
   `rmw_session_property_t properties[] / property_count` (bounded by
   `RMW_SESSION_MAX_PROPERTIES`), and `CffiSession::open_with_properties` /
   `open_named_with_properties` marshal them across.
2. **The key map covered 10 of 23.** `user`, `password`, `scouting_what` and ten
   of the thirteen TLS keys had no spelling at all. The map is now DERIVED from
   zenoh-pico's own `config.h` — `scripts/gen-zpico-config-keys.py` emits the
   committed `zpico_config_keys.h`, gated by `just check zpico-config-keys`, the
   same committed-generated-plus-staleness-gate shape as RFC-0054's bindgen
   output. Four legacy names (`scouting_timeout_ms`, `root_ca_certificate`,
   `root_ca_certificate_base64`, `verify_name_on_connect`) are the file's only
   authored rows.
3. **An unknown key was silently ignored** (`else { continue; }`). It is
   `ZPICO_ERR_CONFIG` now, so the session does not open — this map is the only
   place a typo can ever be caught, because upstream's keys are a bare numbered
   enum with no schema validation. A TLS key on a build without
   `Z_FEATURE_LINK_TLS` is refused the same way rather than being accepted into a
   map nothing reads. Every silent drop on the Rust side went with it: the
   `.min(MAX_SESSION_PROPERTIES)` truncation, the oversized-property `continue`,
   and the env-var arm that fell off the end of the array.

`MAX_SESSION_PROPERTIES` is 16 hosted / 8 embedded (it is a stack array of
`2 × CONFIG_PROPERTY_SIZE` per slot, and issue #64 is what a 4 KB frame costs on
an esp32-c3). 23 keys do not fit 16 and do not need to: `mode` and `connect`
arrive as dedicated `create_session` arguments, and the 13 TLS keys split into a
listen-side and a connect-side set, so a full mTLS client states about ten.

**What W2 (embedded config baking) has to hand this.** The rung ends at
`RmwConfig::properties`: W2 must produce a `&[(&str, &str)]` from the baked
config and give it to `Transport::open` / `CffiSession::open_with_properties`.
Names are the canonical derived spellings in `zpico_config_keys.h`. The
`ZENOH_*` env block in `shim/session.rs` is `#[cfg(feature = "std")]` and reads
six variables; it is a hosted convenience, not the embedded path, and W2 should
not extend it — a `no_std` target has no environment to read.

**What this buys 206.2.** `listen` and `connect` are both reachable now, and
`connect` is the one key upstream inserts with `_z_str_intmap_insert_push`
(append, not replace) — so several endpoints on one session is expressible at
this layer without a new seam.

## Notes

- **Merge vs segregate.** phase-172 K.5 (multi-domain) opens one session *per*
  domain and is correct as-is. Under this phase there is no "merge" primitive to
  build: merging NICs is something Cyclone does inside one participant when its
  config says so.
- **Why the original framing was reasonable.** In May 2026 the tree had a
  `config.toml` reader and a generated standalone package, so a nano-ros-owned
  interface list had somewhere to live. Three retirements removed that world (see
  the status table). The rescope follows the architecture, it does not overrule
  the original author.
- **phase-419's gate saw this doc and let it pass.** Its own W3 measurement
  records `phase-206  ticked=0  landed-marks=3  weak — likely prose, not a
  claim`. The marks were a real claim. R1 keys on ticked boxes and this doc has
  none, so the miss is structural rather than a tuning error — worth knowing
  before anyone widens that rule.

**One decision here overrides what the RFC-0004 amendment says, and the
amendment is being corrected rather than this being worked around.** That
amendment states `rmw_session_options_t` "does not grow a list field … config
does not need it". That is true of CycloneDDS, whose config is a DOCUMENT the
build bakes and the domain parses — and it was written from that case. It is
FALSE of zenoh-pico, which has no document format at all: its configuration IS
`zp_config_insert(key, value)` at session-open time, so key/value pairs have
nowhere to travel except the session-open path. The struct grows by APPENDING
after `enclave`, so `localhost_only`, `_reserved[7]` and `enclave` keep their
offsets — the growth the header's own note sanctioned ("one break, then the
struct grows").

### 206.W4 — device bring-up stated as the board's contract — **DONE 2026-09-05**

- [x] **Audit.** Done; the table above. It inverted the item's premise — there
      was no working contract to document.
- [x] **Root cause: the documented design was not buildable in Rust.** The book
      described a family-crate blanket impl,
      `impl<B: Board + TransportBringup + NetworkWait> BoardEntry for B`, and
      two things make that impossible: it **overlaps** the twelve direct
      `BoardEntry` impls (coherence), and *"skipped if the board doesn't impl the
      mixin"* has no expression in the language — there is no way to call a
      method only when the type happens to implement a trait. The order was
      written as if specialization existed. That is why 212.N.2's impls
      disappeared and the blanket impl sits commented out in the book.
- [x] **Removed `TransportBringup` entirely** (0 impls, 0 callers) and the
      `NetworkWait` trait (1 impl, 0 callers). `NetworkError` STAYS — it is the
      shared vocabulary a board reports a link failure in, used trait or no.
- [x] **`ZephyrBoard::wait_link_up` survives as an INHERENT method.** It is
      public API and its own README example calls it that way already; the trait
      was the part nothing used. The `nros::main!` Zephyr arm keeps calling
      `nros_platform::zephyr::wait_network`, which is what actually links.
- [x] **The book was actively teaching the trap.** `book/src/porting/board-trait.md`
      told a board author to implement `TransportBringup`, with a worked example
      — a method nothing would ever call. Rewritten: device bring-up is the
      `BoardEntry::run` body's job, usually delegated to a family helper, and a
      note records why there is no mixin so nobody re-adds one.
- [x] **Swept the prose that outlived the traits**: `entry.rs`'s boot-order doc
      comment, `nros-board-linux`'s "there is no `TransportBringup` impl" note
      (which described an absence that is now universal), and
      `nros-board-zephyr`'s crate description, README and Cargo.toml, all of
      which called it "NetworkWait-only".

**Verified:** `cargo check -p nros-platform` clean; `nros-board-zephyr` (a
workspace-excluded crate) checks clean standalone; `just check fast` — **202
gates ran, 0 failed** (4 skipped for a missing in-tree `nros` CLI in a fresh
worktree, unrelated).

**Why B and not "wire the traits in".** Making the documented order real means
changing the three family helpers (`nros_board_freertos::run_entry`,
`nros_board_nuttx::run_entry`, `nros_board_threadx::run_entry`) that own RTOS
boot — code this session could not build, because FreeRTOS, ThreadX and NuttX
need cross toolchains. Shipping an unverifiable change to embedded boot order to
make a doc comment true would be a worse defect than the one being fixed. The
option is recorded in issue 1067 if a future phase wants it with the toolchains
in reach.

### 206.W5 — delete nano-ros's interface abstraction — **DONE 2026-09-04**

#### W5 landed 2026-09-04 — the sweep

**nano-ros should not model NICs.** The user configures the middleware in the
middleware's own language (CycloneDDS XML, a zenoh config's key/values); devices
are the board's job and are up before ROS exists. A nano-ros-owned interface
vocabulary needs a resolver, a gate and a per-platform story only Linux can
satisfy, in exchange for re-spelling a setting both backends already accept.

Measured against the tree:

| symbol | impls | callers | action |
|---|---|---|---|
| `BoardTransportConfig::set_interfaces` | 0 | 0 | **deleted** |
| `BoardTransportConfig::set_mac` | 0 | 0 | **deleted** |
| `BoardTransportConfig::set_gateway` | 0 | 0 | **deleted** |
| `BoardTransportConfig::set_ssid` | 0 | 0 | **deleted** |
| `BoardTransportConfig::set_password` | 0 | 0 | **deleted** |
| `BoardTransportConfig::set_ipv4` | 5 boards | 0 | **kept** — see below |
| `BoardTransportConfig::set_baudrate` | 2 boards | 0 | **kept** — see below |

The single caller of all seven was the orchestration generator, deleted with the
standalone-package pipeline in `11a00b0f8` (#202) along with
`orchestration/generate.rs`. The seams outlived their only writer. Both CMake
parsers this doc credits with accepting `interfaces` (`NanoRosConfig.cmake`,
`NanoRosReadConfig.cmake`) no longer exist either, so `NROS_CONFIG_INTERFACES` is
gone with them.

`set_ipv4` and `set_baudrate` are **kept and flagged, not deleted**: they have
real bodies on five and two boards respectively (threadx-linux, threadx-qemu-
riscv64, freertos, mps2-an385, esp32-qemu) but no caller anywhere. That is a
different defect from a dead seam — boards doing work nobody asks for — and it is
a decision about the `NanoRosOwned` net-stack path, not about NIC vocabulary. It
does not belong in this phase.

*The original W5 items, all now done:*
- [x] Remove `BoardTransportConfig::set_interfaces`
      (`nros-platform/src/board/config.rs:108`) — zero callers, wrong layer
      (RFC-0049's duty rule: *platform toml = software-stack facts, board toml =
      hardware facts*), and a silent default no-op.
- [x] Remove `PlanTransport.interfaces` and its validation. **Bare removal,
      not a deprecated alias** — checked first: no `[[transport]]` block in the
      tree, in `examples/`, or in any fixture `system.toml`/`nros.toml`/
      `nros-plan.json` carries the key, so an alias would have re-introduced the
      vocabulary to serve a path no input uses. The upstream schema
      (`ros-launch-manifest` `TransportBlock.interfaces`, pinned v0.1.21) is not
      ours, so the key can still ARRIVE: `plan_transports` now bails naming the
      backend config that owns NIC binding, via one shared
      `plan::RETIRED_INTERFACES_REMEDY` so the remedy has a single spelling.
      It currently parses, validates, reaches the IR, is serialized into
      `<bake>/nros-plan.json`, and is read by nothing.
- [x] `set_ssid` / `set_mac` / `set_gateway` / `set_password` lost their emitter
      in the same commit (`11a00b0f8`) and are dead by the same argument. Sweep
      them together or state why not — the repo's fix-the-class rule.

- [x] **`set_ipv4` and `set_baudrate` KEPT, deliberately.** They have real
      bodies on five and two boards (threadx-linux, threadx-qemu-riscv64,
      freertos, mps2-an385, esp32-qemu) and **no caller either**. Deleting a
      seam with no impl and no caller is a cleanup; deleting one with five impls
      is a decision about who writes a board's IP now the generator is gone.
      Flagged on the trait and filed as [issue 1064](../issues/1064-board-transport-setters-have-impls-but-no-callers.md) rather than swept in here.
### 206.W6 — Fast DDS whitelist (unchanged: out of scope)
- [ ] Not actionable; no Fast DDS backend exists. Under the new principle it
      needs no nano-ros work at all when one does — a Fast DDS user writes a Fast
      DDS profile, and W2's seam carries it.

## Acceptance

- [x] A user attaches a Cyclone XML naming an IP or a device, and it takes
      effect **without losing the baked baseline**. The composition is
      `nros_rmw_cyclonedds_user_config` on the host lane; the attachment is
      `workspace-cpp-native-cyclonedds`, whose ELF carries the SSoT's exact
      bytes. *On an RTOS target* the claim is still COMPILE-stage only — see
      W2's note; the RTOS row's stub app gc-strips the backend.
- [x] The same file works byte-identically hosted and embedded. Byte-identity is
      measured in both places (exact SSoT bytes in the hosted ELF; the raw
      literal compared to the file on the Zephyr configure). "Embedded" means
      as far as the object, not yet as far as an RTOS image that links Cyclone.
- [ ] A zenoh user sets `listen` from a C entry and from an embedded image.
- [ ] Multi-homing is demonstrated **with no nano-ros feature involved**: a
      hosted Cyclone node reachable on `lo` and one real NIC, configured purely
      by `<General><Interfaces>` in the user's own XML. This is the original
      phase's acceptance criterion, met by deleting the mechanism it proposed.
- [x] `nano-ros parses neither XML nor JSON5.` Cyclone's parser reads the XML;
      zenoh's key/value table takes the pairs — and both spellings are the ones
      the backends' own documentation uses, so a user's existing knowledge
      transfers instead of being re-learned.

## What this phase DELETED, and why

**206.1, the multi-endpoint `SessionSpec`.** Multi-homing on Cyclone is a config
property of ONE participant, not N endpoints on one session, and no backend here
expresses it as multiple runtime endpoints. `open_multi` already exists and
correctly SEGREGATES — it is phase-172 K.5, and it should stay that way
(`nros-node/src/executor/spin.rs:300-388`).

**Growing `rmw_session_options_t`.** It is
`{u8 localhost_only; u8 _reserved[7]; const char *enclave;}` and **nothing in
the tree ever passes it non-NULL**. A list-shaped field would break the struct,
force a bindgen regen, and touch the hand-mirrored C++ FFI structs that have
already drifted three times (CLAUDE.md). Config does not need it.

**An intermediate design (drafted and discarded 2026-09-04):** multi-homing as
an `[rmw.capabilities]` entry, a `[board.links]` inventory modelled on
`[board.priority_plan]`, and a per-platform name resolver. It was better placed
than `set_interfaces` and still wrong — it kept nano-ros in the business of
modelling interfaces. Recorded because the reason it was discarded is the
principle at the top of this doc.

## The constraint any revival must satisfy

`set_interfaces` and `NROS_NETSTACK` are **two live instances of declared-but-
unread** — phase-349 names the shape while flagging its own:
*"`NROS_NETSTACK` is emitted too … and nothing reads it — the same
declared-but-unread shape."* Every work item above pairs a declaration with a
consumer AND an acceptance test that fails today. A third instance is the one
outcome that would leave the tree worse than not doing this phase.
