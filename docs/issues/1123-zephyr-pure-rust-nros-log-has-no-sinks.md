---
id: 1123
title: "A pure-Rust Zephyr image installs the `log` bridge and never publishes an `nros_log` sink list, so every framework record sits in the early ring"
status: open
area: boards, core
severity: medium
found: 2026-09-06
related: [1048, 0589, 0708, 0710]
---

# The facade that works is not the facade CLAUDE.md tells you to use

Found while sweeping the CLASS of issue 1048 (`log::set_logger` does not exist
on `riscv32imc`, so the esp32-qemu board dropped every record). The esp32 half
is fixed. This is the OTHER instance the sweep turned up, and it is the mirror
image: on Zephyr the `log` facade works fine and `nros_log` is the one that
goes nowhere.

## What is claimed, and where

CLAUDE.md's issue-0589 entry is explicit that `std::println!` kills a Zephyr
native_sim image and that the replacement is:

> Write `nros_log::nros_error!(nros_log::get_logger("<crate>"), …)`: it lands on
> `LOG_ERR`/`printk`, never fatal, and reaches `no_std` targets that every
> `cfg(feature = "std")` arm silently skipped.

That is true of the delivery MECHANISM — `nros_platform_log_write` is defined
unconditionally in `packages/platform/nros-platform-zephyr/src/platform.c:1117`
and maps severity onto `LOG_ERR`/`LOG_WRN`/`LOG_INF` (or `printk` without
`CONFIG_LOG`). It is not true of the DISPATCH, for a pure-Rust image.

## The gap

`nros_log::dispatch_to_sinks` (`packages/core/nros-log/src/lib.rs`) reads a sink
list that `nros_log::init` publishes. With none published it does not fall back
to the platform — issue 0710 deliberately removed that, because reaching for
`nros_platform_log_write` on a path every binary executes turned a pluggable
delivery into a link-time requirement. Instead it HOLDS the record in
`nros_log::early`'s bounded ring, to be drained by whatever `init` eventually
runs.

For a pure-Rust Zephyr image, no `init` ever runs:

* `nros::zephyr_component_main!` (`packages/api/nros/src/lib.rs:772`) installs
  `zephyr::set_logger()` — the `log` facade — and nothing else.
* `nros::main!`'s Zephyr `rust_main` codegen
  (`packages/core/nros-macros/src/main_macro.rs:1906`) does the same.
* `nros-board-zephyr` calls `::nros_platform_cffi::log::init_default()` at
  exactly one funnel, `entry_tiers.rs:363` (`run_tiers`) — so only a MULTI-TIER
  image is covered.
* The C/C++ path is fine and is why this has stayed invisible:
  `nros_log_emit` lazily calls `ensure_default_sinks()`
  (`packages/api/nros-c/src/log.rs`), and `nros_log_init()` exists as an explicit
  C entry. A pure-Rust image has no `libnros_c.a` (issue 0163), so neither runs.

The records that go missing are not user records — a Zephyr example body writes
`log::info!`, which works. They are the FRAMEWORK's: `nros-node`'s executor
(`arena.rs`, `monitor.rs`, `action.rs`, `node.rs`, `spin.rs`),
`nros-rmw-zenoh` (`shim/session.rs`, `zpico.rs` — including the session-pool
diagnostic that issue 0589 moved to `nros_log` specifically so it would reach
`no_std` targets), `nros-rmw-cffi`, and `nros-rmw-bridge`. On a pure-Rust Zephyr
image every one of those is constructed, dispatched, held, and never seen.

## Evidence, and what is NOT evidence

STATIC only. This was read out of the sources listed above; no Zephyr image was
built or booted for it, because the host it was found on could not afford a west
build. Specifically NOT established:

* whether any pure-Rust Zephyr fixture in the tree would visibly change output
  once a sink list is published (the early ring has a bounded depth, so some
  records may already have overflowed by the time an `init` lands);
* whether the `early-records-<N>` default in a Zephyr build is non-zero at all —
  `early-records-0` restores pre-0708 dropping.

Both need a build to answer. Anyone picking this up should start there rather
than trusting this section.

## Why this is 1048's class and not a separate bug

Issue 0708 required "every board boot funnel calls `init_default()`", and
`nros_log::early`'s own module docs already record that as a SEARCH for boot
paths that "kept losing". 1048 was that search losing on esp32-qemu: the board
published at `run_bare` and not at `BoardEntry::run`, so a fixture image printed
and a `nros::main!` image did not. This is the same search losing on Zephyr,
where the funnel is a MACRO rather than a board method and so was never in the
set anyone was grepping.

## Fix sketch (not yet attempted)

The two macros are the funnels, so they are where the call belongs — but neither
expansion can name `nros_platform_cffi` today: the Zephyr example leaves
(`examples/zephyr/rust/*/Cargo.toml`) dep `nros`, `nros-platform`, `zephyr`,
`log` and the backend, and `nros` itself deps `nros-log` but NOT
`nros-platform-cffi`. So a fix is either

1. re-export the sink from a crate the leaves already dep — `nros-platform` gains
   `pub use nros_platform_cffi::log;` under the features that already pull it in
   (`platform-zephyr` enables `dep:nros-platform-cffi`), and both macros emit
   `::nros_platform::log::init_default();`; or
2. give the leaves the dep, which is worse — it is a link-time requirement
   pushed onto every consumer, which is the shape issue 0710 rejected.

(1) keeps the requirement on the crate that already declares the platform.
Whichever is chosen, the acceptance is a BUILT native_sim image whose console
shows a framework `nros_log` record, not a green `cargo check`.
