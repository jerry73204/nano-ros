---
id: 1138
title: "`rustdoc-links` fails CLOSED when a vendored source is not provisioned, so every fresh worktree reads as a documentation defect"
status: open
type: bug
area: ci, tooling, docs
severity: medium
related: [1043, 1110, 1116, 0390, 0650]
found: 2026-09-06
---

# "I cannot build the docs here" is reported as "your docs are broken"

## Symptom

On the fast line — and therefore in the `pre-push` hook — in any checkout where
`packages/rmw/zenoh/zpico-sys/zenoh-pico` is not provisioned:

```
zenoh-pico source not provisioned at ".../zpico-sys/zenoh-pico". Run: nros setup
--source zenoh-pico (or git submodule update --init
packages/rmw/zenoh/zpico-sys/zenoh-pico). — #0390
error: recipe `rustdoc-links` failed
check-fast (parallel): 1 of 235 gate(s) FAILED
pre-push: REFUSING to push — the fast gate tier is red.
```

The tree is fine. The host has not provisioned an optional vendored source.

## Measured, on four independent sessions in one day

`rustdoc-links` arrived 2026-09-06 in `6359314e5` (issue 1110, "the docs deploy
was red for three days, and no lane could say so"). Within hours it blocked four
agent sessions working in `git worktree`s, none of which had touched Rust:

| session | what it was fixing | what it did |
| --- | --- | --- |
| issue 0986 | pre-push hook side effects | pushed with `NROS_SKIP_PREPUSH_CHECKS=1`, reported it |
| issue 1015 | C array extent floors | initialised the submodule; **verified the red reproduces identically on unmodified `origin/main` by stashing** |
| issue 1029 | zephyr nightly cron | pushed with `--no-verify`, recorded the reason in the commit |
| issue 1016 | tier-2 west leaves | initialised the submodule |

Two workarounds, two bypasses, zero real defects. A gate whose ordinary outcome
is a bypass has stopped being a gate.

## Cause

`just check rustdoc-links` (`just/check/docs.just:229`) is only:

```bash
source scripts/build/rustdoc-set.sh
mapfile -t pkgs < <(nros_rustdoc_package_args)
cargo doc --no-deps --quiet --features "$NROS_RUSTDOC_FEATURES" "${pkgs[@]}"
```

It asserts nothing about provisioning. The failure comes from a DEPENDENCY's
build script: `cargo doc` runs build scripts, and
`nros-zpico-build/src/runner.rs:999` **panics** when the source is absent:

```rust
if !use_side && !zenoh_pico_src.join("include").exists() {
    panic!("zenoh-pico source not provisioned at {:?}. ... — #0390", zenoh_pico_src);
}
```

That panic is correct for a BUILD — an image cannot be produced without the
source. It is wrong as the verdict of a DOC-LINK gate, which is asking a
different question, and it is the gate that reports.

## The class, and the fix that already exists for it

This is issue **1043** one gate over: *a gate that fails when it cannot evaluate
is indistinguishable from one that found a defect.* 1043 fixed exactly this for
`check-submodule-pins`, and the shape of its fix applies here:

* three outcomes — `FAIL` (documented, links broken) / `NOT VERIFIED` (source not
  provisioned here) / `OK`;
* the narrowing REPORTED, not silent — and reported through the shared
  `nros_check_skip` ledger, because `run-gates-parallel.sh` discards the stdout
  of every gate that exits 0, so a happy-path message is invisible on the push
  lane (issue 0650's shape);
* a remedy that splits by audience — local: "`nros setup --source zenoh-pico`
  turns this skip into a verdict"; CI: name the lane that was supposed to
  provision it;
* an env override (`NROS_RUSTDOC_LINKS_STRICT=1`) restoring fail-closed, wired
  to whichever lane genuinely provisions the source.

The check must be made BEFORE `cargo doc`, since the panic comes from inside it.

## Not covered

* Whether other fast-line gates share the shape. `rustdoc-links` was found
  because four sessions hit it in one day; nobody has swept the other 234 for
  "fails closed on an unprovisioned optional source". That sweep is the real
  class and is not attempted here.
* Whether the published-crate set should depend on `zpico-sys` at all for a
  LINK check — narrowing the feature set might sidestep the build script
  entirely, which would be a better fix than reporting the skip. Not measured.
* Issue 1116 (~70 rustdoc diagnostics outside the published set) is a different
  problem: it is about WIDENING this gate, not about it failing to evaluate.
