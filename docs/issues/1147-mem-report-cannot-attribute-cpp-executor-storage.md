---
id: 1147
title: "`mem-report` counts the C++ executor storage but files it under the `nros`
  RUST crate, so the C/C++ half of every embedded image's largest pool is
  attributed to the wrong owner and can never join a declared pool"
status: open
type: tech-debt
area: tooling
related: [phase-392, 0815]
---

## Problem

`scripts/nros-mem-report.py` attributes a symbol to a crate lexically, on the
first `lowercase_ident::` of the DEMANGLED name:

```python
CRATE = re.compile(r"\b([a-z][a-z0-9_]*)::")
```

`nm -C` demangles Itanium C++ as well as Rust v0, so the C++ executor storage
— `Node::GlobalStorageHolder<0>::storage`, an `alignas(8) uint8_t
[NROS_CPP_EXECUTOR_STORAGE_SIZE]` in `.bss` (`packages/api/nros-cpp/include/nros/node.hpp`)
— arrives as `nros::Node::GlobalStorageHolder<0>::storage` and matches `nros`.
That is the name of a **Rust crate in this workspace** (`packages/api/nros`), so
the bytes land in its bucket, indistinguishable from Rust `nros` statics.

Pool attribution then cannot reach it either. The join key is
(crate ident, last `::` segment), and it additionally requires
`name.startswith(crate + "::")` where `crate` comes from the declaring file's
`Cargo.toml` name. A pool would have to be declared in a crate literally named
`nros` and named `storage` — and `gen-pool-inventory.py` scans tracked `*.rs`
only (`git ls-files '*.rs'`), so a `// nros-pool:` comment in the C++ header is
invisible to it regardless.

The plain C path is worse: a file-scope `static struct { … nros_executor_t
executor; … } app;` has no `::` at all, so it falls into the
`(C / asm / no path)` bucket. On the native Rust zenoh talker that bucket is
91,216 bytes; on a C or C++ image it also holds the executor storage.

## Why it matters now

phase-392 W6 made the RUST arm visible (`nros_node::executor::backing::EXECUTOR_BACKING`,
21,560 B measured on the native talker). The C and C++ arms were already in
`.bss` — they were never the invisible ones — but they are **mis-attributed**,
and the embedded images are exactly the C/C++ ones. So the campaign can now
price this pool on the platform where it matters least and not on the platforms
it was opened for.

## Not "just add a mapping"

The tempting fix — special-case `nros::Node::GlobalStorageHolder` — is the
authored-map drift class this tree already has scars from (the RMW parity map,
CLAUDE.md). Two shapes worth considering instead:

1. **Give the storage an unmangled name.** Define it once in Rust
   (`#[unsafe(no_mangle)] static mut nros_cpp_executor_storage`) and have
   `node.hpp` declare it `extern "C"`. That removes the C++ template-static
   COMDAT trick, removes the `NROS_CPP_EXECUTOR_STORAGE_SIZE` header/Rust size
   mirror, and gives one greppable symbol across every C/C++ image. It also
   inherits `NROS_EXECUTOR_BACKING_SECTION` for free. Cost: it touches
   `nros_cpp_init`'s storage parameter and the NuttX `__cxa_guard` workaround
   the template-static exists to dodge (`node.hpp` documents that empirically),
   so it needs a NuttX build to accept.
2. **Teach the report a C++ arm.** Attribute a demangled `A::B::C` with no Rust
   crate of that name to the DECLARING header's component. Cheaper, but it
   invents a second attribution rule, and the mis-attribution to a real Rust
   crate would still need a tiebreak.

Whichever is chosen, the acceptance is the phase's standing rule: a BUILT C or
C++ image showing the storage under an owner a reader would guess, confirmed
with `just mem-report`.

## Related

- phase-392 W1 (issue 0815) — the instrument this is about.
- phase-392 W6 — fixed the Rust arm and measured it; this is the arm it left.
