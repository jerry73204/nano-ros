---
rfc: 0006
title: "Portable RMW / Platform Interface — Design Review"
status: Stable
since: 2026-05
last-reviewed: 2026-05
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# Portable RMW / Platform Interface — Design Review

**Status:** Design note (no code yet)
**Date:** 2026-05-06
**Scope:** Re-evaluate the "Rust trait + C vtable dual API" choice
in light of multi-language consumers / providers.

---

> **Enforcement role (agnosticism contract).** This C-ABI vtable seam is the
> ONLY way core packages + user libraries (`nros-node`, `nros-c`, `nros-cpp`,
> the `nros` API surface) reach platform/RMW behaviour — they must NOT carry
> `platform-*`/`rmw-*` features or `#[cfg(feature="platform-*")]` branches. See
> the **Agnosticism contract** in ARCHITECTURE §2; convergence tracked by issue
> #60 / phase-248.

## TL;DR

Make the **C ABI the canonical interface**. Rust trait becomes a
thin syntactic-sugar wrapper *generated from* the C-ABI types, not
a parallel definition. Every other language (C++, Python, Lua,
Go, Zig, Swift, …) integrates against the same C ABI without going
through Rust. Phase 115's `NrosTransportOps` fn-ptr vtable is the
template pattern; extend it across the whole RMW + platform surface.

This trades Rust idiom (`Result`, `Option`, generics, futures) for
**1 lingua franca instead of N². The Rust API stays ergonomic — just
no longer the source of truth.

---

## Problem

We have three first-class language surfaces today:

| Layer | Crate / Header | Provided By |
|-------|----------------|-------------|
| Rust apps | `nros-rmw` trait | core team |
| C apps | `<nros/*.h>` (cbindgen) | core team |
| C++ apps | `<nros/*.hpp>` (handwritten over C FFI) | core team |

And two provider sides:

| Layer | Crate / Header | Provided By |
|-------|----------------|-------------|
| Rust RMW backend | `impl Rmw for ...` | dust-DDS, zenoh-pico, XRCE-DDS, uORB |
| C RMW backend | `nros-rmw-cffi::NrosRmwVtable` | downstream (currently unused; placeholder) |
| Rust platform | `impl Platform*` traits | nros-platform-{posix,zephyr,…} |
| C platform | `nros-platform-cffi::NrosPlatformVtable` | downstream (placeholder) |

The current shape is "Rust trait is canonical, C vtable is
mechanically forwarded." That works for two languages. It breaks
when you add a third.

### Concrete pain — adding a new language

Hypothetical: someone wants to write an RMW backend in Zig, or
expose nano-ros to Python apps via `ctypes`.

**Today's path** (Zig backend):

1. Read `nros-rmw/src/traits.rs`. Translate trait surface mentally.
2. Look at `nros-rmw-cffi/src/lib.rs` for the C-vtable mirror.
3. Notice the two definitions are *almost* aligned but not quite —
   `Result<T, E>` vs `rmw_ret_t`, `Option<usize>` vs sentinel,
   `&[u8]` vs `(ptr, len)`, etc.
4. Pick one (the cffi vtable, since Zig can't import the trait).
5. Hope cffi stays in sync with the trait when we update it.

**Today's path** (Python app):

1. Bind `<nros/nros_generated.h>` via `ctypes` or `cffi`.
2. Discover that some entry points (typed pub/sub, action client)
   are missing from the C surface entirely — they only exist as
   Rust generics.
3. Either: write the missing entry points yourself, or do raw-bytes
   only.

Both paths converge on "the C ABI is the real interface, the
Rust trait is just the most popular consumer."

### Drift risk

Today, when we change `nros-rmw::Subscriber::take_serialized_with_info`,
we update:
1. `nros-rmw/src/traits.rs` (the trait)
2. `nros-rmw-cffi/src/lib.rs` (the vtable)
3. `nros-rmw-abi/include/nros/rmw_vtable.h` (regenerated)
4. `nros-c/src/subscription.rs` (C surface)
5. `nros-cpp/include/nros/subscription.hpp` (C++ surface)

If any one is forgotten, the API drifts silently. We caught two
such drifts in Phase 108 alone (publisher `supports_event`,
`assert_liveliness` C entry).

---

## Recommendation

### R1. Make the C ABI canonical

`#[repr(C)]` Rust structs in **`nros-rmw-cffi`** become the **single
source of truth**. cbindgen emits the C header. Rust trait
(`nros-rmw::Rmw`, `Session`, `Publisher`, etc.) is reduced to a
thin wrapper that delegates to the vtable.

Pseudo-code:

```rust
// canonical: nros-rmw-cffi/src/lib.rs
#[repr(C)]
pub struct NrosRmwSubscriberVtable {
    pub abi_version: u32,                   // R5: see Versioning
    pub _reserved: [u8; 4],
    pub take_serialized: unsafe extern "C" fn(
        sub: *mut NrosRmwSubscriber,
        buf: *mut u8,
        buf_len: usize,
        out_len: *mut usize,                // 0 ⇒ no message; >0 ⇒ byte count
    ) -> NrosRmwRet,
    pub supports_event: unsafe extern "C" fn(
        sub: *const NrosRmwSubscriber,
        kind: NrosRmwEventKind,
    ) -> bool,
    // ... etc
}

// thin wrapper: nros-rmw/src/lib.rs
pub trait Subscriber {
    fn take_serialized(&mut self, buf: &mut [u8]) -> Result<Option<usize>, TransportError> {
        // delegates to vtable; never declared in two places.
    }
}
```

Rust apps still write `sub.take_serialized(&mut buf)?`. No idiom loss.
But the *definition* lives in cffi.

### R2. Hand-written canonical C headers

cbindgen is the wrong direction for the ABI surface. The header
is the source of truth and the Rust struct mirrors it. Drift is
caught by `#[repr(C)]` + compile-time `assert!(size_of::<T>() == ...)`
checks where layout matters, and by the C-stub integration tests
that pin every function signature.

Concrete shape per layer:

- **RMW vtable** (`<nros/rmw_vtable.h>`) — hand-written canonical
  struct of fn pointers with `abi_version` first field;
  `nros_rmw_register()` rejects mismatched versions. Phase 117 pattern.
- **Platform ABI** (`<nros/platform.h>`) — hand-written canonical
  set of free `extern "C"` symbols (no vtable struct, no register
  call). Link-time resolution; one platform per binary. Diverges
  from the RMW shape because platforms never swap at runtime. The
  full contract — surface inventory, capability groups, two port
  shapes (Rust trait + macro export vs pure C body), and the
  "adding a new port" checklist — lives in
  [`book/src/internals/platform-c-abi.md`](../../book/src/internals/platform-c-abi.md).

cbindgen still produces `nros_generated.h` for message types —
that surface is data-only, machine-derivable, and a poor fit for
hand maintenance. The ABI tier headers are not.

### R3. Avoid Rust-only shapes in the public ABI

| Drop | Replace with |
|------|--------------|
| `Result<T, E>` | `NrosRmwRet` (`i32`) + out-pointer for `T` |
| `Option<T>` | sentinel-typed `T` (`OptUs(u32)` exists; generalise) |
| `Vec<T>` / `String` | `(ptr, len)` slice pair, caller-allocated |
| `&[u8]` / `&mut [u8]` | `(ptr, len)` |
| `impl FnOnce` | `unsafe extern "C" fn(...)` + `user_data` |
| `Box<dyn ...>` | fn-ptr vtable (Phase 115 already does this) |
| Generics (`M: RosMessage`) | raw bytes + type-name string |
| Associated types | none — collapse to a single `NrosRmwRet` |
| Lifetimes | document ownership in prose; `*mut T` / `*const T` only |
| `Future` / `async fn` | poll-style + waker callback (already done in `register_waker`) |

We already follow most of these in `nros-rmw-cffi`. Audit what
remains and harmonise.

### R4. Layered API ladder

Document and enforce three explicit layers:

| Layer | Purpose | Audience | Examples |
|-------|---------|----------|----------|
| **L0 — C ABI** | Lingua franca | Anyone | `<nros/rmw_vtable.h>`, `<nros/transport.h>` |
| **L1 — Idiomatic wrapper** | Per-language ergonomics | Per-language users | Rust trait, C++ classes, Python class |
| **L2 — Application API** | Typed pub/sub/service | App developers | `Publisher<M>`, `nros::Subscription<M>` |

Rules:
- **L1 and L2 are mechanical translations of L0.** Generated where
  possible; thin glue otherwise. No new design decisions.
- **All design decisions live at L0.** A new feature lands in
  `nros-rmw-cffi` first; L1/L2 wrappers follow.
- **Backward compat is tracked at L0.** L1/L2 may be reshuffled
  freely as long as L0 is preserved.

### R5. Vtable versioning

Reserve the first 8 bytes of every vtable struct for a `(version,
reserved)` pair:

```c
typedef struct {
    uint32_t abi_version;       // bumped on breaking change
    uint32_t reserved;          // 0; future flags / capabilities
    /* ... fn pointers ... */
} nros_rmw_subscriber_vtable_t;
```

- **Major bump** = remove / reorder a field. Old consumers fail
  cleanly via the version check.
- **Minor bump** = append a new fn at the end of the struct. Old
  consumers ignore the new field; new consumers detect support
  via `vtable->abi_version >= MIN_VERSION_X`.

Pattern is well-known (Vulkan, ALSA, dust-DDS itself uses
`get_extension`). Cheap to add now, expensive to add later.

### R6. Multi-language smoke tests

Add a `tests/multi_lang/` harness that exercises the C ABI from at
least two non-Rust languages:

1. **C-implemented stub RMW** that the Rust core drives. Catches
   "vtable shape that only Rust can produce" issues. ~200 LOC.
2. **Python-implemented stub RMW** via `cffi` or `ctypes`. Catches
   "vtable shape that needs Rust-specific compiler magic." ~150 LOC.

Run both as part of `just test-all`. If one breaks, we hear about
it before downstream users do.

### R7. Reduce the API surface

Every method in `Rmw`, `Session`, `Publisher`, `Subscription`,
`ServiceTrait`, `ClientTrait` is a portability cost:
each must be (a) declared in cffi, (b) wrapped in nros-c, (c)
wrapped in nros-cpp, (d) documented in the porting guide.

Audit pass:
- Mark methods as **Required** vs **Optional**.
- Optional methods have default impls (returning `Unsupported`)
  so other-language backends only implement what they need.
- The `supports_event` / `supported_qos_policies` discovery pattern
  (Phase 108) generalises: every optional capability has a
  `supports_<capability>` predicate in the vtable.

### R8. Phase 115's pattern, generalised

`NrosTransportOps` (Phase 115.A) already follows R1-R5:
- `#[repr(C)]` struct, fn pointers + `user_data`.
- Single layout for Rust + C + C++ (the three current consumers).
- Sentinel-encoded return shapes (`i32` for both fail-codes and
  byte-counts).
- Versioning achievable via "register vtable in slot, drain at
  open" indirection.

Promote this pattern as the **template for any future Rust→C
boundary**. New work that adds a Rust trait should justify why
fn-ptr-vtable doesn't fit — not the other way around.

---

## Concrete next steps

Tracked separately as a follow-up phase ("Phase 117: Portable
ABI alignment", or similar):

1. Audit current `nros-rmw-cffi` against R3 — list all remaining
   non-portable shapes.
2. Add `abi_version` to all vtable structs (`NrosRmwVtable`,
   `NrosPlatformVtable`, plus future ones). Bump from `0` →
   `1` as the rollout.
3. Add the C-implemented stub RMW smoke test (R6).
4. Update `book/src/porting/custom-rmw.md` to document L0/L1/L2
   ladder and lead with the C ABI.
5. Re-evaluate `nros-rmw::Rmw` trait: every associated type, every
   `impl Trait` parameter, every Rust-specific shape gets a
   note explaining how an other-language implementer maps it.
6. Add Python `ctypes` smoke test (R6) — proves the C surface is
   usable from a dynamic-typed language without Rust knowledge.

---

## Anti-recommendations

Things to **NOT** do:

- **Don't write IDL.** Resist `flatbuffers` / `protobuf` /
  `WIT` / similar IDL-then-codegen flows. They solve the
  wrong problem (cross-machine wire format) at the cost of build
  complexity. We have C headers; that's the IDL. cbindgen does
  the codegen.
- **Don't expose a Rust-only async surface.** `async fn` in trait
  + `impl Trait` for futures = unportable. Stick with poll-style
  + waker callback. Per-language wrappers can layer async on top.
- **Don't make the vtable struct part of the *public* C-side API.**
  Users call `nros_subscription_take_serialized` (a C entry that
  internally dispatches via the vtable). Backend authors fill in
  the vtable. Keep the two surfaces separate.
- **Don't generic-ize the vtable types.** No `NrosRmwSubscriber<T>`.
  Type info travels as a `type_name` string + size (R3, generic
  drop).

---

## See also

- [`docs/roadmap/phase-115-runtime-transport-vtable.md`](../roadmap/archived/phase-115-runtime-transport-vtable.md)
  § A.1 — the no-`dyn` rationale that motivates this whole review.
- [`docs/research/sdk-ux/SYNTHESIS.md`](../research/sdk-ux/SYNTHESIS.md)
  UX-22 — the original "users-fork-board-crates" pain that Phase
  115 addresses. R6's multi-language smoke test reduces the same
  pain for backend authors.
- `book/src/internals/rmw-api-design.md`
  — current RMW-API doc; needs an L0/L1/L2 update once Phase 117
  lands.
