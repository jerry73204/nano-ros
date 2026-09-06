---
id: 1172
title: The C and C++ entry emitters derive a tier's callback groups differently
status: open
area: codegen
severity: medium
opened: 2026-09-06
---

# The C and C++ entry emitters derive a tier's callback groups differently

Both entry emitters bake a per-tier array of callback-group names, and the
runtime uses those arrays to decide which tier runs a callback. The two
emitters build the array from the same `ResolvedTierTable` and do not agree.

`emit_c` (`packages/cli/nros-cli-core/src/codegen/entry/emit_c.rs`):

```rust
// deduped ACROSS tiers, empty names dropped
let mut seen: HashSet<String> = HashSet::new();
for t in &tiers.tiers {
    for (_, grp) in &t.members {
        if !grp.is_empty() && seen.insert(grp.clone()) { g.push(grp.clone()); }
    }
}
```

`emit_cpp`:

```rust
// deduped WITHIN each tier, empty names kept
let mut seen = BTreeSet::new();
tier.members.iter().filter_map(|(_, g)| seen.insert(g.clone()).then(|| g.clone()))
```

Two consequences, both silent:

1. **A group named by two tiers.** C lists it under the FIRST tier only; C++
   lists it under BOTH. So the same plan produces a group routed to one tier in
   a C entry and present in two tier arrays in a C++ entry.
2. **A member with an empty group name.** C drops it; C++ emits `""` into the
   array and counts it in `n_groups`, so the C++ tier claims a group whose name
   is the empty string.

Neither shows up today because no golden and no fixture declares a plan with a
cross-tier group or an empty group name — which is exactly why it survived.

## Why it is open rather than fixed

Found while converting `emit_cpp` onto a template (phase-432 W2.3), which is a
byte-for-byte change: reconciling the two moves goldens, so it belongs in its
own commit with its own diff to read. The shared row type
(`codegen::entry::TierView`, added in W2.3) is what makes the divergence
visible at all — before it, the two derivations sat in two files with no
common declaration.

## What a fix has to decide

Which behaviour is correct is a RUNTIME question, not a style one: it depends
on what the tier arrays mean to `run_tiers` when a group appears twice. Answer
that first, then make it one derivation beside `tier_views`, and add a golden
row that carries a cross-tier group and an empty group name so the answer is
pinned.

## Related

- phase-432 (`docs/roadmap/phase-432-codegen-one-producer-many-packs.md`) — W2.3.
- RFC-0091 — one entry-codegen producer, many language packs.
- RFC-0047 — callback groups and tiers.
