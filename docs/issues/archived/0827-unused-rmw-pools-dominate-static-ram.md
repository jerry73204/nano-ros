---
id: 827
title: "Static RAM is a property of the RMW, not of the node — a talker reserves
  275 KB of service and large-payload pools it can never reach"
status: resolved
type: performance
area: rmw
related: [phase-392, phase-391, phase-412, issue-0815, issue-0739]
---

## Problem

Measured with `just mem-report` over the four native Rust example roles, at
knob defaults, one row per (role, RMW):

| role | zenoh | cyclonedds | xrce |
| --- | ---: | ---: | ---: |
| `talker` | 345,379 | 69,381 | 10,340 |
| `listener` | 345,379 | 69,381 | 10,340 |
| `service-server` | 345,379 | 69,381 | 10,340 |
| `action-server` | 345,395 | 69,381 | 10,356 |

RAM attributed to symbols, bytes, `nros-relwithdebinfo`, measured on the
fixtures `just build-test-fixtures lane=native` writes under
`build/cargo-fixtures/linux*/nros-relwithdebinfo/`.

**Every role costs within 16 bytes of every other, and three of the four are
identical.** The 16 bytes on `action-server` are its own statics; no pool moves.
What the node does has essentially no influence on what it reserves. A `talker` — one publisher, no subscription, no service, no action —
reserves the same 144,128 bytes of `SERVICE_BUFFERS` as the service server, and
the same 131,072 bytes of `LARGE_PAYLOADS` as a node that subscribes to point
clouds.

For the talker, from `just mem-report`:

```
       144,128   40.4%  nros_rmw_zenoh::shim::service::SERVICE_BUFFERS
       131,072   36.7%  nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS
        32,768    9.2%  nros_rmw_zenoh::shim::subscriber::SMALL_PAYLOADS
        24,416    6.8%  g_sessions
```

275,200 of 345,379 bytes — **80% of the image's static RAM** — is two pools the
node cannot reach. `SMALL_PAYLOADS` is the only one it can, and it uses one of
its eight subscriber slots.

## Why it is this way

The pools are unconditional `static mut` arrays in the backend crate. Linking
the backend reserves all of them; nothing about the node's entity set reaches
the decision. This is deliberate and correct as a starting point — a static pool
is what makes the allocation statically provable, which is the property
[phase 392](../../roadmap/phase-392-static-memory-space-campaign.md) explicitly
protects when it declines to move payload buffers to the heap.

The defect is not that the pools are static. It is that their SIZE is fixed at
the backend, where the entity set is unknown, rather than at the image, where it
is known. Two of the three inputs already exist:

- **The entity set is known at build time for a generated entry.** `nros sync`
  resolves the SystemModel; the entry codegen emits the entities. Phase 392 W2
  is already planning to sum the executor arena from exactly that source
  (`NROS_ARENA_REQUIRED`).
- **The knobs already exist and are already enumerable.**
  `ZPICO_MAX_QUERYABLES`, `ZPICO_MAX_LARGE_SUBSCRIBERS`, `ZPICO_MAX_SUBSCRIBERS`
  are in the static pool inventory.
  A node with no service server wants `ZPICO_MAX_QUERYABLES = 0`.

So the saving is available without inventing a mechanism: it is one more
consumer of the resolved model, setting knobs that are already read. What is
missing is anything that connects the two.

## The trap in the obvious fix

`ZPICO_MAX_QUERYABLES` cannot simply be lowered to the app's service count.
Per CLAUDE.md and issue 0460, **a service server IS a zenoh queryable**, and
`[param_services]` (6) plus `[lifecycle]` (5) claim eleven slots before the app
declares anything. A knob derived from "how many services does the app create"
would be wrong by eleven and fail at runtime with an exhausted table — the
failure mode 0460 already cost a phase.

Any derivation has to count the infrastructure queryables the runtime creates on
the node's behalf, which means it belongs next to the code that creates them,
not in a codegen template that only sees the user's entities.

## Also worth noting

`LARGE_PAYLOADS` is 131,072 bytes at defaults (`ZPICO_MAX_LARGE_SUBSCRIBERS *
ZPICO_SUBSCRIBER_RING_DEPTH * ZPICO_SUBSCRIBER_LARGE_SIZE`) and is reserved even
when the image has no subscription at all. It is the single easiest win in the
list: an image whose resolved model contains zero subscriptions needs zero of
it, and that derivation needs no infrastructure accounting — unlike the
queryable one above, the runtime creates no large-payload subscriber of its own.

## Measured 2026-08-29 — the "easiest win" is NOT available with the existing knob

The section above calls `LARGE_PAYLOADS` "the single easiest win ... an image
whose resolved model contains zero subscriptions needs zero of it". Zero is
**not expressible**. `packages/rmw/zenoh/nros-rmw-zenoh/build.rs:45`:

```rust
let max_large: usize = env_usize("ZPICO_MAX_LARGE_SUBSCRIBERS", 2).max(1);
```

The `.max(1)` floor means the smallest reachable pool is
`1 * ZPICO_SUBSCRIBER_RING_DEPTH(4) * ZPICO_SUBSCRIBER_LARGE_SIZE(16384)` =
**65,536 bytes**, not 0. So wiring the resolved model to this knob — the fix
this issue proposes — buys half of what the issue claims: 131,072 -> 65,536 on
a talker, with the other half structurally out of reach until the floor goes.

MEASURED, not read — three builds of `examples/native/rust/talker`, the knob
varied, `llvm-nm -S` on each binary:

| `ZPICO_MAX_LARGE_SUBSCRIBERS` | `LARGE_PAYLOADS` |
| ---: | ---: |
| 2 (default) | 131,072 |
| 1 | 65,536 |
| **0** | **65,536** |

Note the third row: asking for zero does not fail, it silently yields one. A
codegen deriving "no subscriptions -> 0" would emit a config that reads as
satisfied while still reserving 64 KiB — the same shape as every other defect
this campaign has found, a value that looks applied and is not. If the floor
stays, the knob should REJECT 0 rather than round it up.

Baseline confirming the arithmetic, `just mem-report` on
`build/cargo-fixtures/linux-14372940/nros-relwithdebinfo/talker`:

```
       131,072   35.8%  nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS
       131,072  LARGE_PAYLOADS  — agrees with `ZPICO_MAX_LARGE_SUBSCRIBERS *
                                  ZPICO_SUBSCRIBER_RING_DEPTH * ZPICO_SUBSCRIBER_LARGE_SIZE`
```

Removing the floor is not free to reason about: `.max(1)` exists so a pool
index is always valid, and the sibling `max_nodes` floor documents exactly that
intent ("so a session always has room for its own primary node"). A zero-length
pool needs the *lookup* path to refuse a large subscription rather than index an
empty array — a code change, not a knob change. Whoever takes this should price
both halves separately: the knob wiring (65,536, mechanical) and the floor
removal (a further 65,536, needs the refusal path).

The same floor applies to `ZPICO_MAX_QUERYABLES`-style derivations. Check for it
before quoting a saving off the inventory: the inventory prints the formula, not
the floor.

## Reproduce

```sh
just build-test-fixtures lane=native
just mem-report build/cargo-fixtures/linux-3263301353/nros-relwithdebinfo/talker
just mem-report build/cargo-fixtures/linux-3263301353/nros-relwithdebinfo/service-server
```

Both print the same pool figures.

**Measure the fixtures, not `examples/**/target-*/`.** The first draft of this
issue took its numbers from
`examples/native/rust/talker/target-zenoh/nros-fast-release/talker`, which was
three weeks stale: phase 340 P2 moved fixture builds into the shared cargo group
under `build/cargo-fixtures/`, and the per-leaf directories are leftovers that
nothing rewrites. `just build-test-fixtures` reported success without touching
them. The pool figures happened to be unchanged, so the conclusion survived, but
the totals were wrong by 2,417–2,598 bytes and the "identical to the byte"
claim was wrong by 16. Trust the group directory, and check an artifact's mtime
against its sources before quoting it. `--json` plus `--baseline` shows the delta
between any two images, which is how a fix should be reported.

## Design settled 2026-08-29 — and why const generics are not the answer

**Const generics reach no non-Rust consumer, and are still wrong here.**
Traced the boundary: `SERVICE_BUFFERS` is a private `static mut` — no header,
no `#[no_mangle]`, no `repr(C)`. C round-trips one opaque `*mut c_void` token
(`session_index * ZPICO_MAX_QUERYABLES + local`) that it never does arithmetic
on, and validates its own handles against its own table. So the two tables are
NOT layout-coupled; this is not the 0135 class. If Rust's N exceeded C's, Rust
would admit a server C then refuses — a clean error, not corruption.

They are wrong anyway because a const generic needs a TYPE to carry it. A Rust
entry has one (`Node::ENTITY_BOUNDS`, the W5 shape). A C/C++ entry does not —
it is cmake-driven through `nano_ros_entry()`. Pushing the generic up leaves
two exits, both bad: a non-generic C entry point that picks some N (a
hand-picked number again, the thing being removed), or a sizing parameter on
the C API, which stops it being a thin wrapper of Rust.

**The direction this need runs is C -> Rust.** `*_OPAQUE_U64S` is the existing
thin-wrapper channel and it flows Rust -> C: Rust owns the type, a build step
computes `size_of`, a generated header carries the number, C declares opaque
storage. Here the application declares how many services it has and the backend
consumes it — the reverse. That channel is specified but unbuilt: phase-392 W2's
`NROS_ARENA_REQUIRED`. So this is a second consumer of a planned mechanism, not
a new one.

**One declaration site serves both languages.** The feature-forwarding plan (add
`param-services`/`lifecycle-services` passthrough to `zpico-sys`, forward at
~15 leaves, gate the forwarding) is unnecessary: the resolved model already
carries it. `system.toml` declares `features = ["param_services", "lifecycle"]`
and the plan schema has `PlanParamServices` / `PlanLifecycle` as first-class
fields. Rust entries reach it through the macro, C/C++ entries through
`nano_ros_entry()` reading the same model.

Division of labour, which keeps issue 0460 closed: the model says WHETHER the
infrastructure services exist; Rust says HOW MANY, beside the code that creates
them. Codegen must never own that number — it sees the user's entities and
never the runtime's.

**Net C/C++ API change: none.** No new function, no parameter, no generic in a
header. One more `#define` in a file that is already generated.

**Decided: an undeclared image fails loudly.** A bare `cargo build` of a leaf,
and the standalone `check-rmw-*` projects, have no model. Rather than keep
today's generous 32 (safe, and 144,128 bytes of service buffers whether or not
the image has a single service), the model-less path is a build-time failure
that names what to declare. Per this issue's own `.max(1)` finding, a fallback
that silently yields a working-but-wasteful number is the shape that reads as
satisfied and is not.

### Landed already (the count itself)

The counts now have one definition each, beside the code that creates them:
`parameter_services::PARAM_SERVICE_QUERYABLES` (6) and
`lifecycle_services::LIFECYCLE_SERVICE_QUERYABLES` (5). They had SEVEN
spellings and no definition; two were wrong, both saying lifecycle was 6, which
is where "twelve slots" came from. It is eleven.

`check-infra-queryable-counts` ties each constant to the number of
`create_param_srv::<_>` / `create_lc_srv::<_>` statements, so a seventh service
fails until the constant moves — a constant alone is still a hand-typed literal
and would have drifted exactly as the prose did. It also forbids an RMW backend
from restating a count it cannot derive: that gate found the seventh mirror on
its first run.

### Still open

The sizing itself. Wants its own work item — it is the W5 restructuring one
layer down, and should not be smuggled in beside a comment fix.


## Re-measured 2026-09-01 — the claim HOLDS, and the derivation does not reach here

Rebuilt the native fixtures against current HEAD (`just build-test-fixtures
lane=native`) and re-ran `just mem-report` on the zenoh talker, because this
issue's numbers were three days old and the pool knobs had moved under them
(phase-403 W4 removed the `.max(1)` floor; `NanoRosMessageBounds.cmake` gained
`NROS_DERIVED_MAX_LARGE_SUBSCRIBERS`).

`build/cargo-fixtures/linux-3263301353/nros-relwithdebinfo/talker`, built
03:31 on 2026-09-01:

```
RAM attributed to symbols:       415,469 bytes
       144,128   33.3%  nros_rmw_zenoh::shim::service::SERVICE_BUFFERS
       131,072   30.3%  nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS
        90,136   20.8%  g_sessions
        32,768    7.6%  nros_rmw_zenoh::shim::subscriber::SMALL_PAYLOADS
```

**144,128 + 131,072 = 275,200, byte-identical to the 2026-08-29 figures**, on a
talker with one publisher, no subscription, no service. The two pools it cannot
reach are unchanged.

The share fell (80% -> 64%) only because the total GREW, 345,379 -> 415,469, and
`g_sessions` at 90,136 is most of that growth. Not investigated here; noted so
the next reader does not mistake a smaller percentage for progress.

## Why the mechanism landed and the bytes did not

Both halves of the fix this issue asked for now exist:

* `ZPICO_MAX_LARGE_SUBSCRIBERS = 0` is legal (phase-403 W4 removed the
  `.max(1)` floor, so "no large class" is expressible rather than silently
  rounded to one block);
* `cmake/NanoRosMessageBounds.cmake` derives `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS`
  from the type set and treats a count of zero as an ANSWER.

**Neither reaches this image.** `LARGE_PAYLOADS` still agrees exactly with
`ZPICO_MAX_LARGE_SUBSCRIBERS * RING_DEPTH * LARGE_SIZE` at crate defaults,
because the derivation is published by CMake and consumed by
`zephyr/cmake/nros_cargo_build.cmake`'s `_nros_resolve_derivable_knob` — and a
native cargo fixture never goes through CMake at all. The knob is read by
`nros-rmw-zenoh/build.rs` from the ENVIRONMENT, and on this path nothing sets
it.

That is this repo's recurring shape, and [phase 412](../../roadmap/phase-412-derived-counts-and-sizes.md)
names it in its own opening: *a mechanism that is correct, tested, and
unreachable from a real build* — `rx_buffer_hint` sizing nothing (0896), the
bound inventory with no reader (0963), a cap that could not reach codegen
(#152).

## Ownership — this issue's implementation belongs to phase-412, not here

phase-412 is the campaign for exactly this, and it has already solved the part
that made this issue hard. The obvious wiring, `MAX_QUERYABLES =
COUNT_SERVICE_SERVER`, is WRONG: a declared action is one entity costing several
session slots (`action server -> 3 queryables + 2 publishers`, `action client ->
1 subscription`), so the raw per-kind count under-sizes every image with an
action. Its multipliers now live beside the calls that decide them, held by
`check-infra-queryable-counts`. It also deliberately does NOT count
`PARAM_SERVICE_QUERYABLES` (6) and `LIFECYCLE_SERVICE_QUERYABLES` (5), because a
feature enables those and the inventory cannot see it — which is the eleven-slot
trap issue 0460 already cost a phase.

Measured there on the island: `NROS_MAX_QUERYABLES` 4 -> 0 and **-17,170 bytes**,
RAM headroom 0.87% -> 4.76%.

**What this measurement adds that the island audit could not see:** phase-412
came from a Zephyr board, where every knob arrives through CMake and Kconfig. The
native cargo path has no such carrier, so a derivation consumed by
`nros_cargo_build.cmake` leaves every `cargo`-built image — every native example,
every fixture this repo tests against — at crate defaults. Same knob, same
derived number, no way in.


## What the derivation is WORTH here — measured A/B, 2026-09-01

The sections above say the derivation cannot reach a cargo-built image. This is
what it would recover if it could. Same leaf, same profile, same target dir, one
variable:

```
cd examples/native/rust/talker
cargo build --profile nros-relwithdebinfo                       # default
ZPICO_MAX_QUERYABLES=0 ZPICO_MAX_LARGE_SUBSCRIBERS=0 \
    cargo build --profile nros-relwithdebinfo                   # honest
```

| symbol | default | knobs at 0 | delta |
| --- | ---: | ---: | ---: |
| RAM attributed to symbols | 415,469 | 136,293 | **-279,176** |
| `SERVICE_BUFFERS` | 144,128 | 0 | -144,128 |
| `LARGE_PAYLOADS` | 131,072 | 0 | -131,072 |
| `g_sessions` | 90,136 | 86,168 | -3,968 |
| `SMALL_PAYLOADS` | 32,768 | 32,768 | — |

**67.2% of this image's attributed static RAM, from two environment
variables.** The default build reproduces the fixture's 415,469 to the byte, so
the leaf and fixture paths agree and the comparison is sound.

Both zeroes are TRUE of this image, not chosen for effect: a talker declares one
publisher, no service and no subscription. `ZPICO_MAX_QUERYABLES = 0` is what
phase-412 derives for exactly this shape; `ZPICO_MAX_LARGE_SUBSCRIBERS = 0` is
what `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS` derives from a type set with nothing
over the small-class ceiling.

Note `g_sessions` moves too. It is not one of the two pools this issue names —
it carries the C shim's per-session queryable table, which
`cmd/entity_facts.rs` already documents as a second consumer of
`ZPICO_MAX_QUERYABLES`. So the knob is worth 3,968 B more than the pool
arithmetic alone predicts.

## Deliberately NOT fixed here

Two fixes were available and both were declined, with reasons:

* **Hand-set the knobs in each native example's `.cargo/config.toml`.** It
  would work and the numbers would be true. It is also twenty hand-picked
  numbers of exactly the kind phase-412 exists to delete, planted in the demos
  people copy from. A number that is correct today and unaudited tomorrow is
  the thing this campaign keeps finding.
* **Build a second derivation path for cargo.** phase-412 owns the carrier
  question, and the sizes-header mirror class (0088 -> 0114 -> 0122 -> 0123 ->
  0245 -> 0268) is what two implementations of one derivation produce. The
  right move is one carrier that both CMake and cargo consume, and that is a
  design decision for the phase, not a second mechanism bolted on from an
  issue.

What this issue now hands phase-412 is the number: the carrier is worth 279 KB
on the smallest possible image, and a standalone leaf has no model to derive
FROM, so "state the knob" may be the only honest answer for the examples even
after the carrier exists.

## Re-measured 2026-09-04, after phase-412 W1 — unchanged to the byte

W1 landed 2026-09-03 and moved these exact knobs, so the A/B above was re-run
rather than trusted (the 0859-0862 rule: a measurement is about the tree its
artifacts were built from).

| | 2026-09-01 | 2026-09-04 (HEAD 9ba86845d) |
| --- | ---: | ---: |
| stock | 415,469 | **415,469** |
| both knobs at 0 | 136,293 | **136,293** |

Byte-identical on both halves. W1 did not reach this path, which is what the
section above predicted and is now measured rather than argued.

## The carrier EXISTS, and it is the one `nros sync` already writes (2026-09-04)

The handoff above said a leaf "has no model to derive FROM" and left the carrier
open. Two findings narrow it a long way, and one of them contradicts the
pessimism.

**1. `.cargo/config.toml`'s `[env]` reaches the pools. Measured, not assumed.**
Same leaf, same profile, nothing in the shell environment, no CMake anywhere:

```
[env]
ZPICO_MAX_QUERYABLES = "0"
ZPICO_MAX_LARGE_SUBSCRIBERS = "0"
```

| build | RAM attributed to symbols |
| --- | ---: |
| stock `.cargo/config.toml` | 415,469 |
| + the `[env]` block above | **136,293** |

Exactly the env-var figure, through a file. Sequenced stock -> instrumented from
a clean rebuild with the variables explicitly UNSET in the shell (`env -u`),
because the first attempt reused the earlier env-var build and finished in 0.7 s
— a cached result that would have "confirmed" the carrier without testing it.

> Note for anyone re-running the A/B: the `cd examples/native/rust/talker &&
> cargo build` in this issue's own "Reproduce" section writes a leaf `target/`,
> which `check-example-leaf-target-dirs` refuses (phase-340 P2 gave every
> fixture a shared per-coordinate group dir, and a bare leaf build predates
> that). It is genuine residue, not a false positive — `rm -rf` it when done.

**2. Why the value MUST come from outside the cargo graph — an ordering
argument, not a preference.** The pools are sized in `nros-rmw-zenoh`'s build
script. That crate is a DEPENDENCY of the leaf. The entities are declared in the
leaf's own `src/lib.rs` (`node.create_publisher_for_topic::<StringMsg>`). So the
crate that must know the counts is compiled BEFORE the crate whose source states
them, and no leaf-side derivation — build script, proc macro, manifest metadata
read at the wrong time — can reach backwards across that edge. `nros ws
entity-inventory` cannot help either: it reads the `nros-metadata.json` a CMake
configure writes, and a cargo leaf runs no configure.

**What that does to this issue's two declined fixes.** The first one — "hand-set
the knobs in each example's `.cargo/config.toml`" — was declined for being twenty
hand-picked numbers. That objection stands, but it was aimed at the wrong half:
the LOCATION was right and only the AUTHORSHIP was wrong. `nros sync` already
generates leaf `.cargo/` content and already keeps the generated part in a
gitignored sidecar, separate from the authored part (issues 0457/0463). A derived
`[env]` block belongs in exactly that sidecar — generated, regenerable, never
committed, and never hand-picked.

So what phase-412 has to build is smaller than "a second derivation path": one
derivation, published into a carrier that already exists on both sides. The open
question is no longer WHERE the number goes, it is what a cargo leaf derives
FROM — and that is a question about where entities are declared, not about
transport.


## The SOURCE exists too, and it is already in the tree (2026-09-05)

The section above closed the carrier question and left one open: *"what a cargo
leaf derives FROM"*. It has an answer that needs no new mechanism, and the
answer was sitting beside the leaf the whole time.

**`<leaf>/metadata/<component>.json` — the metadata probe's output.** It is
present today for every probeable native leaf:

```
examples/native/rust/talker/metadata/talker.json
examples/native/rust/listener/metadata/listener.json
examples/native/rust/action-client/metadata/fibonacci_action_client.json
examples/native/rust/action-server/metadata/fibonacci_action_server.json
```

and it carries exactly the counts the pools are sized by. Per node, the list
keys are `publishers`, `subscribers`, `timers`, `services`, `actions` — read out
of the files, not from a schema doc:

| leaf | what its metadata states |
| --- | --- |
| `talker` | `publishers: 1`, `timers: 1` |
| `listener` | `subscribers: 1` |
| `fibonacci_action_client` | `action_clients: 1` |

So `ZPICO_MAX_SUBSCRIBERS` comes from `subscribers`, `ZPICO_MAX_QUERYABLES` from
`services` plus the infra floor, and so on. The declaration is already parsed;
nothing needs a second parser.

### Why this does not contradict the ordering argument above

It does not reach backwards across the dependency edge — it does not try to.
The probe runs BEFORE the build, as its own step, and writes a file. The build
script then reads an `[env]` value, exactly as it does today for a hand-written
one. The ordering argument rules out deriving the number *inside* the cargo
graph; it says nothing against deriving it before the graph is built, which is
what `nros sync` already is.

### Both artifacts are gitignored, and that is consistent rather than a problem

`examples/**/metadata/*.json` is ignored (`.gitignore:184`), and so is the
`.cargo/` sidecar the derived `[env]` belongs in (issues 0457/0463). Both are
per-host, regenerated, never committed. A fresh clone has neither and gets both
from `nros sync` — the same contract `generated/` already has.

### What is left, precisely

One step: read the probe output, apply the counting rules
`entity_inventory` already encodes, write the `[env]` block into the sidecar.
Carrier measured (415,469 -> 136,293 B), source verified present, rules already
written. No new mechanism, no second SSoT.

### The one place this does NOT reach, and it has an issue

Leaves whose metadata is `<component>.json.unprobeable` — the probe cannot run
for them, twice over: `[unstable] build-std` with a foreign `[build] target`
(`ProbeBlocker::BuildStdForForeignTarget`), and a board crate that does not build
for a host triple at all. Every `examples/qemu-esp32-baremetal/rust/*` leaf is in
this set. That is **issue 1061**, and it is the reason those leaves still state
their budgets by hand in `.cargo/config.toml`.

So the derivation splits by leaf kind, and the split is measured rather than
assumed: native leaves have a source today, cross-compiled ones need 1061 first.


## The derivation is BUILT (2026-09-05) — `nros_cli_core::leaf_entity_env`

The step the section above described as "one step" is written and tested:
read the probe output, apply `entity_inventory`'s rules, render the `[env]`
block. What is NOT yet done is calling it from `nros sync`; that is the
remaining work and it is named at the bottom.

**It reuses the rules, it does not copy them.** The module turns each
`<leaf>/metadata/<component>.json` into the same `EntityDecl` rows a
`nano_ros_node_register(... ENTITIES ...)` produces, then hands them to the
existing `EntityInventory::derive()`. A CMake image and a cargo leaf therefore
cannot disagree about what a declaration costs — which they would the moment
there were two counting implementations.

That reuse paid immediately. The first test asserted `max_subscribers == 0` for
a talker; the shared derivation returned **1**, because issue 1015 puts a FLOOR
OF ONE on any pool backing a fixed C array (`queryable_entry_t
queryables[0]` is not a smaller pool, it is a different kind of object). The
test was wrong and the rule was right — a private counter here would have
shipped the zero.

**Verified against the real in-tree probe files, not only fixtures:**

| leaf | entities | `MAX_CBS` | subs | pubs | queryables | heavy |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `native/rust/talker` | 2 | 1 | 1 | 1 | 1 | 0 |
| `native/rust/listener` | 1 | 1 | 1 | 1 | 1 | 0 |
| `native/rust/action-client` | 1 | 1 | 1 | 1 | 1 | **1** |

The action client's `heavy=1` is issue 0900's rule firing (a heavy arena slot),
and its `subs=1` is the feedback subscription an action client opens — neither
is declared in the `.msg` and neither is guessed here. Against the shipped
defaults of 8 per pool, those are the numbers this issue set out to recover.

**Decisions worth stating:**

* the sidecar sets no `force = true`, so a value the CALLER states still wins —
  a number a human chose beats one derived on their behalf;
* an entity row the module cannot parse REFUSES the whole leaf rather than
  skipping, because a skipped row lowers a pool below what the image creates and
  short halts the board;
* an empty probe result is `Declaration::None` ("creates nothing"), never
  `Absent` — the probe having run IS a statement;
* `.json.unprobeable` files are counted and reported rather than ignored, since
  their presence is exactly why a leaf may get no sidecar (issue 1061).

### What is left

Calling it from `nros sync`: render to a gitignored `[env]` sidecar beside
`nros-managed-patch.toml` and add the `include` entry, in
`cmd::ws.rs` where `MANAGED_PATCH_FILE` is written today. That path is delicate
for a known reason — a missing `include` target is a HARD cargo error during
manifest parse (issue 0463) — so the file and its `include` must appear and
disappear together, which is the same invariant the patch sidecar already holds
and the reason to put it in the same place rather than a new one.


## WIRED and MEASURED (2026-09-05) — `nros sync` writes the budget

The remaining step is done: `write_patch_config` derives the leaf's budget and
writes `.cargo/nros-managed-env.toml`, with the `include` entry added in the
same decision that writes the file.

**A/B on one host, one tree, one profile — not against a number from another
day**, because a stale baseline is how this repo has been wrong before:

| `examples/native/rust/talker` | RAM attributed to symbols |
| --- | ---: |
| without the derived sidecar | 417,316 |
| with it | **240,596** |
| **saved** | **176,720 (42.3 %)** |

That lands within 2 KB of this issue's earlier hand-set `[env]` measurement
(415,469 stock), reached from a different direction — the agreement is the
cross-check.

The mechanism is confirmed end to end rather than inferred from the file: the
build's generated shim constants read

```
/// Maximum number of concurrent subscribers (set via ZPICO_MAX_SUBSCRIBERS, default 8).
pub const ZPICO_MAX_SUBSCRIBERS: usize = 1;
```

so the derived number reached the crate that sizes the pool.

### Two sidecars, not one section

`nros-managed-env.toml` is a SEPARATE file from `nros-managed-patch.toml`
because the two empty independently: a leaf can have a generated message dep and
no probeable component, or the reverse. Sharing a file would make each one's
presence depend on the other's. Both follow the same rule — the file and its
`include` entry appear and disappear together, since a missing include target is
a hard cargo error during manifest parse (issue 0463).

Gitignored, like its sibling: it is derived from a probe output that is itself
per-host and ignored, so committing it would pin one host's answer for everyone.

### What is still NOT derived

`ZPICO_MAX_LARGE_SUBSCRIBERS` — `LARGE_PAYLOADS` remains 131,072 B and is the
largest single symbol left in that image. The entity inventory has no notion of
a "large" subscriber, so nothing here can state that number honestly; it stays a
knob a human sets. Recording it rather than leaving the impression this issue
recovered everything.

And the whole mechanism reaches only PROBEABLE leaves. A cross-compiled leaf
whose metadata is `.json.unprobeable` still states its budgets by hand — issue
1061 — and `nros sync` now says so on the terminal instead of leaving it to be
inferred from an absent file.
