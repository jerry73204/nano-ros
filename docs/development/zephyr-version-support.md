# Zephyr version support

How nano-ros decides which Zephyr versions it supports, why, and how to add a
new one. Policy rationale + the full work plan live in
[`docs/roadmap/phase-199-zephyr-version-support-policy.md`](../roadmap/archived/phase-199-zephyr-version-support-policy.md);
this is the developer-facing reference.

## The contract: a `(zephyr × zephyr-lang-rust)` pair

nano-ros's Zephyr support is **not** bounded by our patches — it is bounded by
**`zephyr-lang-rust`**, the official in-tree Rust module our integration links
through (`rust_cargo_application()` + the `zephyr` crate; a generated Zephyr
package is a Rust staticlib). A *supported configuration* is therefore a pair:

> a Zephyr revision **plus** a `zephyr-lang-rust` revision known to build against it.

Each `west-<line>.yml` pins both halves of the pair. Bumping one without a
matching other is unsupported.

### The floor: Zephyr 3.7

`zephyr-lang-rust`'s first commit ("Initial support for Rust on Zephyr") landed
**2024-09-11**, right after the **Zephyr 3.7.0 LTS** (July 2024). It did not
exist for 3.6 (Feb 2024) or 3.5 (Oct 2023). **There is no nano-ros-on-Zephyr
below 3.7** — no amount of native_sim / NSOS patching changes that; the Rust
staticlib has nothing to link into. Downstream integrators (e.g.
autoware-safety-island) must be on **≥ 3.7**.

### The window: current LTS + at most one rolling

- **Default / CI baseline: the current Zephyr LTS** (today **3.7**, supported
  ~5 yr). This is `NROS_ZEPHYR_VERSION`'s default + the `west.yml` manifest.
- **At most one rolling line** that `zephyr-lang-rust` supports (today **4.4**,
  `west-4.4.yml`, `NROS_ZEPHYR_VERSION=4.4`).
- **Never below the `zephyr-lang-rust` floor** (3.7).

Zephyr releases on a 6-month cadence (Apr/Oct); an LTS every ~2.5–3 yr. When a
new LTS is declared, it becomes the default and the old rolling line is dropped.

### The rolling line is KEPT because consumers outside this repo use it

Decided 2026-08-20 (maintainer), and worth stating here because the record on
its own says the opposite. Issue 0651 checked the "a feature requires 4.4"
recollection against phase-199 and the commit that pinned `west-4.4.yml`, found
no feature gated on it, and concluded the rolling line was a POLICY slot — which
made "drop it rather than half-test it" a legitimate answer to the cost of
maintaining a line no local tier could reach.

That conclusion was drawn from what is visible in this repository, and it is
wrong in the way repository-only evidence usually is: **nano-ros has users
outside this tree, and they are on the rolling line.** Nothing in the code or
git history can show that. So the rolling line is not an optional policy
artifact whose upkeep is weighed against convenience — dropping it breaks real
consumers, and the cost of the 4.4 line is a cost of shipping, not of tidiness.

Practical consequence: 4.4 coverage is worth paying for. That is why tier 3
(`just ci-full`) now REQUIRES the 4.4 line rather than skipping it when
unprovisioned, and why `just zephyr kconfig-trees` exists to make the cheap
half of that check available without a west workspace.

If you are about to argue from "nothing in the tree needs 4.4", read this
paragraph first — the argument is sound and the premise is not.

## Stable surfaces vs the churn

Build only on Zephyr's **stable** surfaces (versioned ≥ 1.0.0, frozen ≥ 2
releases): kernel / `k_*`, Kconfig, devicetree, the public POSIX + BSD-socket
API — plus the `zephyr-lang-rust` contract and our manifest selector. These do
not move across versions.

The per-version **churn** is isolated to one place: the native_sim / NSOS /
CycloneDDS-on-native_sim shims (`drivers/net/nsos_sockets.c`,
`native_simulator/.../nsos_adapt.c`, `pthread_mutex`, SoC files). These edit
Zephyr *internals* (explicitly **not** stable APIs), so they re-anchor each
release. They are **simulator-only** — real targets (FVP `fvp_baser_aemv8r_smp`,
NXP S32Z) use the hardware net stack and don't apply them. The plan is to
upstream them so the carried set shrinks per release (see phase 199.4).

## How patches are applied (per-line dispatch)

`just setup zephyr` runs a **version-dispatched** patch set — no inline
`if version = …` branching in the recipe:

```
just setup zephyr           # → bash scripts/zephyr/patches/${NROS_ZEPHYR_VERSION}.sh "$WORKSPACE"
```

Each line's patch sequence lives in `scripts/zephyr/patches/<line>.sh`
(`3.7.sh`, `4.4.sh`). See
[`scripts/zephyr/patches/README.md`](../../scripts/zephyr/patches/README.md).

## Adding a new Zephyr line — checklist

1. **Confirm the pair builds.** Find a `zephyr-lang-rust` revision that builds
   against the target Zephyr. If the target is below 3.7, stop — it is not
   supportable (no Rust module).
2. **Pin the pair.** Add `west-<line>.yml` pinning **both** the zephyr revision
   and a *specific* `zephyr-lang-rust` revision (not `main` — that drifts; see
   phase 199.2).
3. **Wire the selector.** Add the `NROS_ZEPHYR_VERSION == "<line>"` arms in
   `just/zephyr.just` (the `ZEPHYR_MANIFEST` / `ZEPHYR_WORKSPACE` /
   `ZEPHYR_VENV_BIN` selectors).
4. **Drop a patch set.** Add `scripts/zephyr/patches/<line>.sh` — re-anchor only
   the native_sim / NSOS / CycloneDDS patches still un-upstreamed for that line
   (each idempotent; `cd`s to repo root; takes the workspace arg). **No edit to
   the `setup` recipe** — the dispatcher picks it up by name.
5. **Pin the SDK for the line.** Read `zephyr/SDK_VERSION` in the new tree, add
   a `[tool.zephyr-sdk-<ver>]` table to `nros-sdk-index.toml` (host-keyed URLs
   + upstream's own sha256s), and add the matching `case "$MANIFEST"` arm in
   `scripts/zephyr/setup.sh` naming that entry. **Check the distribution
   shape** — 0.16.x ships one fat tarball with every toolchain, 1.0.x ships
   `_minimal` plus per-toolchain downloads the SDK's own `setup.sh -t` fetches
   on demand, and 1.0.x has no macos-x86_64 at all.

   This step exists because it was missed. Zephyr 4.4 requires SDK **1.0.1**
   while the version was a hardcoded `0.16.8`, so
   `NROS_ZEPHYR_VERSION=4.4 just zephyr setup` exited 0 and produced a
   workspace that could not build any Cortex-M target — the failure surfacing
   only at the first `west build` as a bare `FindZephyr-sdk.cmake:160` error.
   `just zephyr doctor` now compares the registered SDKs against
   `zephyr/SDK_VERSION` and fails loudly instead.
6. **Add a CI line.** Extend the Zephyr jobs in `.github/workflows/nightly.yml`
   with the new line.
7. **Provision sources via `nros`.** `just setup zephyr` provisions
   `zenoh-pico` / `cyclonedds-src` / `px4-rs` via `nros setup --source`
   (index-driven; the canonical path — no hand `git submodule update`).

## See also

- [`docs/roadmap/phase-199-zephyr-version-support-policy.md`](../roadmap/archived/phase-199-zephyr-version-support-policy.md) — rationale + the full work plan.
- [`scripts/zephyr/patches/README.md`](../../scripts/zephyr/patches/README.md) — the per-line patch-set contract.
- [`docs/development/sdk-tiers.md`](sdk-tiers.md) — which modules install in which `just setup` tier.
