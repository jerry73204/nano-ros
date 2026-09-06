# Phase 188 — Book front-door presentation

**Goal.** Make the deployed mdBook (`NEWSLabNTU.github.io/nano-ros-book/`)
land a strong first impression in the first 30 seconds — *show*, not just
*tell* — and remove front-door drift. The persona funnel
(`start-here/choose-your-entry.md`) is already good and stays as-is; this
phase reworks the landing page (`introduction.md`) and the architecture
entry point.

**Status. CLOSED 2026-09-06.** 188.A + 188.C DONE (2026-05-28);
`mdbook build book` clean (mermaid renders, no broken links). Workstream B
(visual identity) was scoped but deferred pending a logo decision, and in the
three and a half months since, that decision has not been made.

It is rehomed as **issue 1141** rather than kept here. What remained was two
theming files — a favicon/logo and an accent stylesheet — blocked on a DESIGN
choice, not on engineering: the `[output.html]` / `additional-css` wiring they
need already has a working precedent in the same `book.toml` (the mermaid
preprocessor). A phase held open by a favicon reads, to anyone scanning the
roadmap, as an engineering workstream in progress; it is not one.

The stale-source-comment follow-up noted under 188.A is carried into 1141 too,
so closing this loses nothing.

**Priority.** P2 — presentation / adoption. No product capability depends
on it, but the book is the public front door and the GitHub-Pages site is
where evaluators land first.

**Depends on.** Nothing in-code. Touches only `book/src/`. Mermaid is
already wired (`book/book.toml` → `[preprocessor.mermaid]` + `additional-js`).

---

## Overview

The book is content-rich (113-line `SUMMARY.md`, ~60 pages) and the
persona-based `choose-your-entry.md` funnel is strong. The weak spots are
all on the **landing page**:

- The intro **tells** (feature bullets + tables) but never **shows** a code
  snippet — bad for the "5-minute glance" persona the funnel explicitly
  targets.
- Mermaid is loaded but used in exactly one page (`concepts/two-layer-api.md`);
  the intro and `concepts/architecture.md` are wall-of-text/table with **zero
  diagrams**.
- **Drift:** intro Key Features said "**Dual** middleware — Zenoh or
  XRCE-DDS" while the RMW Backends section below it lists **three** backends
  (adds Cyclone DDS). The "dual" framing predates Cyclone becoming a
  first-class backend.
- The "Project Status" section is a vague "under active development"
  paragraph; the README already carries a crisp feature-status table that
  the maturity-signal persona actually wants.

## Architecture

All changes are confined to `book/src/`. No theme, CSS, or workflow change
in Workstream A — those are Workstream B. The hero code snippet mirrors the
canonical `examples/native/rust/talker/` (`src/lib.rs::run()`), trimmed to
the minimal idiomatic publisher so it stays copy-pasteable and drift-checkable
against a real example.

## Work Items

### 188.A — Front-door content (active)

- [x] **188.A.1 — Fix the "dual middleware" drift.** `introduction.md` Key
  Features: replace "Dual middleware — Zenoh or XRCE-DDS" with a
  three-backend "pluggable middleware" framing (Zenoh, XRCE-DDS, Cyclone
  DDS), consistent with the RMW Backends section lower on the same page.
- [x] **188.A.2 — Add a hero code snippet.** Insert a minimal Rust talker
  (~16 lines, faithful to `examples/native/rust/talker`) right after the
  opening paragraph of `introduction.md`, so an evaluator sees the API
  immediately. Link to the per-language First Node pages.
- [x] **188.A.3 — Add an architecture diagram.** A compact mermaid
  layer/flow diagram (app → core → RMW → platform, with the wire-compat
  edge) on the landing page, surfaced near the top. Lists the three real
  backends (no retired `dust-dds`).
- [x] **188.A.4 — Tighten "Project Status".** Replace the vague paragraph
  with the feature-status table (mirrors `README.md`), giving the
  maturity-signal persona a real answer.

**Files**
- `book/src/introduction.md` (all four items)

### 188.D — Accuracy follow-on + README badges

- [x] **188.D.1 — Architecture-page backend drift.**
  `concepts/architecture.md` RMW-layer block listed "Zenoh, XRCE-DDS,
  **DDS**, Cyclone DDS" — the standalone "DDS" was retired dust-dds
  (Phase 169). Dropped it → "Zenoh, XRCE-DDS, Cyclone DDS, or a custom
  backend". (Checked `concepts/no-std.md` too: its "all **four** backend
  crates" is correct — the table lists exactly 4 zenoh/xrce-family crates,
  not dust-dds. Left as-is.)
- [x] **188.D.2 — README badges.** Added a badge row under the title: CI
  (Actions `ci.yml`), Book (→ site), `no_std`, Rust edition 2024, ROS 2
  Humble | Iron. **License badge intentionally omitted** — see note.

**Files**
- `book/src/concepts/architecture.md`, `README.md`

> **License inconsistency — RESOLVED in 188.E below.**

### 188.E — License resolution (dual default + ROS carve-outs)

- [x] **188.E.1 — Policy.** Confirmed house default `MIT OR Apache-2.0`
  (already the root `[workspace.package]` value, and 153 of 181 tracked
  crates). Audited all 181: 15 declared `Apache-2.0`, 13 declared nothing.
- [x] **188.E.2 — Carve-outs kept Apache-2.0** (genuinely derived from
  Apache-2.0 ROS 2 sources): `rcl-interfaces`, `lifecycle-msgs` (generated
  from ROS msgs), `nros-c` (rclc-compatible C API).
- [x] **188.E.3 — Fixed 12 drifters → `MIT OR Apache-2.0`:** `nros`,
  `nros-cpp`, `nros-sizes-build`, `zpico-alloc`, the 7 `examples/zephyr/rust/*`,
  and `multi-package-workspace/.../pkg_rust_publisher` (original code with no
  Apache-only reason; the zephyr examples had drifted from the template while
  every other example tree was already dual).
- [x] **188.E.4 — Filled 6 license-less hand-written crates** with the dual
  SPDX (`book/rustdoc-driver`, the two `native/rust/*-async` examples,
  `nros-tests`, `nros-nuttx-ffi`, the `nros-serdes` cmake template).
  **Skipped generated crates** (`rcl-interfaces/generated/*`,
  `wake-latency-cortex-m3/generated/*`) per the don't-modify-generated rule,
  and the internal `tests/simple-workspace/*` fixtures.
- [x] **188.E.5 — Added root `LICENSE-MIT` + `LICENSE-APACHE`** (was none →
  GitHub `licenseInfo: null`) and expanded the README License section
  (file links + carve-out note + contribution clause).
- [x] **188.E.6 — README license badge** can now be added safely; deferred to
  the same follow-up as the rest of the badge row unless requested.

All 18 touched manifests pass `cargo verify-project` (the cmake template is
valid TOML; cargo only rejects its non-`Cargo.toml` filename). The 128
already-correct hardcoded-dual crates were left untouched (consistent with
the examples that must hardcode — standalone example crates can't inherit
`license.workspace`).

### 188.F — Book accuracy sweep

Swept `book/src/` for pre-Cyclone / pre-Phase-169 framing.

- [x] **188.F.1 — Stale "two backends" counts → three.**
  `porting/custom-rmw.md` ("ships with two RMW backends") and
  `getting-started/zephyr.md` ("two RMW backends on Zephyr") both predated
  Cyclone. Fixed to three; added a **Cyclone DDS** subsection to zephyr.md
  with the real `prj-cyclonedds.conf` Kconfig (CONFIG_CPP, heap/arena
  sizing, IGMP, native_sim NSOS offload). Verified the `prj-cyclonedds.conf`
  overlay actually exists.
- [x] **188.F.2 — Interop framing.** `introduction.md` "ROS 2 compatible"
  bullet said interop is via `rmw_zenoh_cpp` only; added the direct RTPS
  path via `rmw_cyclonedds_cpp`.
- [x] **188.F.3 — README license badge** added (resolved by 188.E).
- [x] No remaining `dust-dds` / standalone-"DDS" backend drift outside
  `internals/rmw-backends.md` (which correctly documents the Phase 169
  retirement). `concepts/no-std.md`'s "four backend crates" verified correct
  (4 zenoh/xrce crates, not dust-dds).

### 188.G — Retired shim crates in porting docs

Phase 129 deleted `zpico-platform-shim` + `xrce-platform-shim`; their symbols
now come from default-on C alias TUs (`zpico-sys/c/zpico/platform_aliases.c`
under the `platform-aliases` feature; `nros-rmw-xrce/src/platform_aliases.c`
always compiled into `nros-rmw-xrce-cffi`) forwarding to the canonical
`nros_platform_*` ABI. Rewritten from the archived Phase 129 doc + the actual
`zpico-sys` features and current board crates (not from memory).

- [x] **188.G.1** `internals/porting-platform/zenoh-pico.md` — symbols come
  from the `zpico-sys` alias TU; porter supplies `nros_platform_*`, not `z_*`.
- [x] **188.G.2** `internals/porting-platform/xrce-dds.md` (×4 spots incl. the
  Rust example that hand-wrote `uxr_millis`) — porter implements canonical
  `nros_platform_time_now_ms`/`clock_us`; the alias TU derives `uxr_*`.
- [x] **188.G.3** `concepts/platform-model.md` — "Shims" bullet → "Alias TUs".
- [x] **188.G.4** `porting/overview.md` — crate table rows → alias TUs.
- [x] **188.G.5** `porting/custom-board.md` — fixed the **broken dependency**
  (`path = "…/zpico-platform-shim"`, a crate that no longer exists) → just
  `zpico-sys`; fixed the diagram and the `extern crate zpico_platform_shim`
  keep-alive → `extern crate zpico_sys` (matches real boards).
- [x] **188.G.6** `porting/custom-platform.md` — rewrote the whole "Activate
  the shim(s)" section (shim-feature table + build-script contract were all
  pre-129) → ABI-marker features + default-on alias TU; fixed the
  `ConcretePlatform`-calls-the-shim line.

Verified: only historical "Phase 129 retired" notes mention the shim names
now; `mdbook build book` clean.

> **Minor follow-up (out of scope, code comments not book):** a few stale
> source comments still name the shims — `nros-board-mps2-an385/src/lib.rs:19`,
> `nros-board-orin-spe/src/node.rs`, `nros-platform-api/src/lib.rs` (×2).
> Harmless (comments), worth a cleanup pass with the next platform-crate edit.

### 188.B — Visual identity (deferred follow-up)

- [ ] **188.B.1** Favicon + logo (`book/theme/`), wired via
  `[output.html]` so the site stops looking like a stock mdBook.
- [ ] **188.B.2** Accent CSS (`book/theme/custom.css` via `additional-css`).

### 188.C — Funnel + deploy hygiene

- [x] **188.C.1** Dead-link sweep of `choose-your-entry.md` + `SUMMARY.md`
  after the Phase 168/180 renames. **Clean** — 0 broken across 29 + 81
  `.md` link targets (verified `tmp/linkcheck.sh`); no fixes needed.
- [x] **188.C.2** Set the GitHub repo `homepageUrl` to the book site so the
  repo → website funnel exists (was empty). Also filled the empty
  **description** and added **20 topics** (`ros2`, `embedded`, `rust`,
  `no-std`, `rtos`, `zephyr`, `freertos`, `nuttx`, `threadx`, `dds`,
  `zenoh`, `cyclonedds`, `micro-ros`, `cortex-m`, `esp32`, `riscv`, …).
  Homepage → `https://newslabntu.github.io/nano-ros-book/`.

---

## Acceptance

- A first-time visitor to `introduction.md` sees a working code snippet and
  an architecture diagram above the fold, plus an accurate feature-status
  table.
- No "dual middleware" / two-vs-three-backend contradiction remains on the
  landing page.
- `mdbook build book` succeeds (mermaid renders, no broken links introduced).

## Notes

- The hero snippet is intentionally a trimmed copy of a real example, not a
  bespoke sample — if the public API shifts, the example breaks first and
  this phase's snippet is the obvious next edit.
- Workstreams B and C were scoped in the same review but split out: B needs
  a logo design decision, C spans the GitHub repo surface. Neither blocks A.
