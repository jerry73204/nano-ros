---
id: 1153
title: "`check-rust-targets-covered` reads three declaration sites and misses two more, so a Rust target the tree builds for had no row at all"
status: open
type: bug
area: tooling, ci
severity: medium
found: 2026-09-06
related: [0833, phase-418]
---

# A gate whose coverage is narrower than the rule it enforces

`scripts/check-rust-targets-covered.py` collects "every declaration" of a Rust
target with three `git ls-files` globs:

```python
for rel in tracked("packages/boards/*/nros-board.toml"):      # [target.<triple>]
for rel in tracked("cmake/toolchain/*.cmake"):                # set(Rust_CARGO_TARGET "…")
for rel in tracked("*.cargo/config.toml", ".cargo/config.toml")
```

Two more places declare a target and are not scanned:

- **`zephyr/cmake/nros_cargo_build.cmake`** — `set(NROS_RUST_TARGET …)`, a
  different variable name, at lines 82 and 84:

  ```cmake
  if(CONFIG_FPU)
      set(NROS_RUST_TARGET "armv7r-none-eabihf" PARENT_SCOPE)
  else()
      set(NROS_RUST_TARGET "armv7r-none-eabi"   PARENT_SCOPE)
  endif()
  ```

  Mirrored in `scripts/zephyr/cortex-r-rust-patch.sh`.
- **`ci/docker/ci-base/Dockerfile`** — `rustup target add` lines, which decide
  what CI can build at all.

## What it let through

`armv7r-none-eabi` was declared at `nros_cargo_build.cmake:84` and had **no row
in `config/rust-targets.txt` and no `[rust.target.*]` entry in
`nros-sdk-index.toml`**. A Zephyr Cortex-R build without `CONFIG_FPU` therefore
selected a triple the tree never listed, provisions nothing for, and no gate
noticed — for as long as that branch has existed.

Found while adding the `cortex-r5` arch profile (phase-418 item 418.3), which
needed exactly that triple and had to add the row. So the omission is fixed;
the blindness that allowed it is not.

## Why it is worth fixing rather than noting

This is issue 0196's shape — a gate whose scope is narrower than the rule it
claims — and the tree has paid for it repeatedly. The gate's own docstring says
"every declaration", which is what makes a green run misleading rather than
merely incomplete.

## Shape of a fix

Add the two sites, keying on what each actually writes: `NROS_RUST_TARGET` for
the Zephyr lane, `rustup target add` for the Dockerfile. Both are grep-able and
neither needs a build.

Worth deciding at the same time: whether the gate should assert that the three
independent spellings AGREE for a given triple — `config/rust-targets.txt`,
`nros-sdk-index.toml` `[rust.target.*]`, and `rust-toolchain.toml` `targets` —
rather than only that a declared target appears somewhere. A triple listed in
two of the three is the same defect one step later.
