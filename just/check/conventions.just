# nano-ros gates: conventions
#
# Cross-cutting source conventions that belong to no one subsystem.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

# Issue 0498 — a sync-owned file a CONCURRENT process reads must be written
# temp + `rename(2)`, never `fs::write` (which truncates to zero, then fills).
# `cmd/ws.rs` already had a private `atomic_write` whose doc called it "the
# discipline every other sync-owned file here uses"; the metadata sidecar one
# directory over had three plain `fs::write` writers and died mid-sweep on an
# empty read. Buildless — greps the guarded function bodies.
[private]
atomic-sync-writes:
    @bash scripts/check-atomic-sync-writes.sh

# Issue 0857 — a `nros::node!` class DECLARES what its cell registries hold.
# Undeclared means `NROS_RUNTIME_MAX_CELL_ENTITIES` (8) per kind, and a
# publisher slot embeds a loan arena: measured with `nm -S` on the prebuilt
# `examples/workspaces/rust` native entries, an undeclared one-service class
# cost 50,824 B of `.bss` against 272 B for a class that declares zero. Also a
# LOWER-BOUND check on the numbers — a declaration under what `register()`
# creates is a boot-time registration error, so it fails here instead. Same
# family as the `entity-slot-costs` / `infra-queryable-counts` budget gates one
# file over, but the subject is the `nros` component runtime, not an RMW pool.
[private]
component-entity-bounds:
    @python3 scripts/check-component-entity-bounds.py

# Issues 0400/0706/0712 — EVERY `export -f` list in scripts/build/ must close
# over its call graph, or a helper added to an exported function dies
# "<name>: command not found" in the make WORKER and nowhere else. Three times
# now. Supersedes the #0717 gate, which checked one entry point on the reasoning
# that build_root_derivation.sh covered the other — true, but that scenario
# EXECUTES one call path, so a helper on a branch not taken is invisible to it.
[private]
export-f-closure:
    @bash scripts/check-export-f-closure.sh

# eyre's `Context` alias is behind `#[cfg(feature = "anyhow")]` from 0.6.13, so
# code using it compiles only against a graph that resolves 0.6.12.
[private]
eyre-context-alias:
    @bash scripts/check-eyre-context-alias.sh

# Issue 0481 — a readiness grep naming an AMBIGUOUS literal waits out its whole
# timeout in silence and still passes. Buildless: reads sources only.
[private]
readiness-marker-literals:
    @bash scripts/check-readiness-marker-literals.sh

# String-convention guards (forbidden org / retired-tool refs in user surfaces).
# The `check.yml` step (SSoT).
[private]
string-conventions:
    @scripts/ci/string-conventions-check.sh

# Issue 1117 — a cross build must SAY which compiler it picked and where from.
# The pin is `[tool.arm-none-eabi-gcc] upstream = "13.2.rel1"`; a host whose SDK
# store had never been provisioned built with Ubuntu's /usr/bin 10.3.1 and said
# nothing, and the resulting compile error was diagnosed twice as a codegen bug
# (issue 1113, retracted). The system-toolchain fallback stays — activate.sh
# documents it on purpose — but it is now recorded, the way issue 0500 made
# `Corrosion <ver> via <origin>` the evidence. Self-testing: the mutants that
# matter keep the SHAPE valid (report() naming a different tool than resolve()
# configures still prints a provenance line, about the wrong compiler).
[group("check")]
cross-toolchain-provenance:
    @python3 scripts/check-cross-toolchain-provenance.py

# Issue 1118 — nothing may DECLARE a C++ standard below the one nros-cpp needs.
#
# The tree said C++14 in 95 places while `component_node.hpp` used `if
# constexpr`, which is C++17. GCC only warns on that, so the declared minimum
# was a claim the code violated and no build ever failed. The number lived in
# four vocabularies, two of which already disagreed with their own siblings
# (`arm-freertos-armcr52.cmake` at 17 among four toolchains at 14;
# `fvp_entry/prj.conf` at CPP17 beside its workspace's CPP14).
#
# The gate holds NO version number: it reads the floor out of
# `target_compile_features(nros-cpp-headers INTERFACE cxx_std_NN)`, so raising
# the floor is one edit and this follows. Covers `CMAKE_CXX_STANDARD`, the
# `cxx_std_N` feature and Zephyr's `CONFIG_STD_CPPN`; the `-std=` in a
# toolchain's `CMAKE_CXX_FLAGS_INIT` is deliberately out of scope and the
# script's header says why.
#
# A site that consumes no nros-cpp header and whose own build imposes an older
# dialect (uORB inside PX4 at `-std=gnu++14 -Werror`; the Cyclone wrapper on
# Zephyr at `-std=c++11`) carries `# nros-cxx-floor-exempt: <reason>`. The
# reason is required, and the exempt sites print on the GREEN path.
[group("check")]
cxx-standard-floor:
    @python3 scripts/check-cxx-standard-floor.py
