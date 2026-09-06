#!/usr/bin/env python3
"""Generate book/src/reference/rmw-feature-matrix.md from backend sources.

The per-RMW capability story used to live as prose scattered over four
pages, and it drifted in BOTH directions (Cyclone services were
documented as unsupported a year after service.cpp landed; zenoh manual
liveliness was documented as a no-op while the shim wired it). This
generator derives the wired/NULL facts from the artifacts that cannot
lie:

  * the two C vtables (positional `/*slot*/ value` in cyclonedds's
    vtable.cpp, designated `.slot = value` in xrce's vtable.c),
  * the zenoh shim's Rust trait overrides (presence of the `fn` in the
    backend source — the trait default is the unsupported fallback),
  * the QoS masks (`supported_qos_policies` in the zenoh shim; the
    CFFI layer's blanket mask for the C-ABI backends, which is a
    runtime-side ASSUMPTION — TODO 115.K.2.x in rmw/cffi/src/lib.rs).

Node-layer rows (actions, params, lifecycle) are static data with
citations: they are built on pub/sub + services in nros-node, not on a
per-backend slot.

Run:  python3 scripts/gen-rmw-feature-matrix.py [--check]
Gate: just check rmw-feature-matrix
"""

import os
import re
import sys

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parent / "lib"))
from tracked import tracked  # issue 0721: index lookup, not a walk

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "book", "src", "reference", "rmw-feature-matrix.md")

CYCLONE_VTABLE = "packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/vtable.cpp"
XRCE_VTABLE = "packages/rmw/xrce/nros-rmw-xrce/src/vtable.c"
# The canonical slot list: EMPTY_VTABLE in the cffi crate names EVERY field
# (phase-376 W4 made that a load-bearing property). Parsed as the authority so
# a slot rename/addition surfaces here as a diff, never as a silent miss.
CFFI_EMPTY = "packages/rmw/cffi/src/lib.rs"
ZENOH_SRC_DIR = "packages/rmw/zenoh/nros-rmw-zenoh/src"
ZENOH_SESSION = "packages/rmw/zenoh/nros-rmw-zenoh/src/shim/session.rs"
CFFI_LIB = "packages/rmw/cffi/src/lib.rs"

# slot-name -> (row label, which C-ABI slot(s) must be non-NULL,
#               regex for the zenoh Rust override)
# Slot names follow the phase-376 upstream-rmw spellings.
FEATURES = [
    ("Publish / subscribe", ["create_publisher", "create_subscription"],
     r"fn (create_publisher|publish_raw)"),
    ("Services (server side)", ["create_service", "take_request", "send_response"],
     r"fn (create_service|send_response)"),
    ("Service clients", ["create_client", "send_request", "take_response"],
     r"fn (create_client|send_request)"),
    # phase-428 W13: the zenoh trait method is `service_is_ready` (rclcpp's
    # name); `server_available` was the pre-#638 spelling.
    ("Server-availability probe", ["service_server_is_available"],
     r"fn (server_available|service_is_ready)"),
    ("Status events (deadline / liveliness / lost)",
     ["subscription_event_init", "publisher_event_init"],
     r"fn register_event_callback"),
    ("Manual liveliness assert", ["publisher_assert_liveliness"],
     r"fn assert_liveliness"),
    ("Event-driven wake (`set_wake_callback`)", ["set_wake_callback"],
     r"fn set_wake_callback"),
    ("Deadline hint (`next_deadline_ms`)", ["next_deadline_ms"],
     r"fn next_deadline_ms"),
    ("Zero-copy loan API", ["borrow_loaned_message"], r"fn (pub_loan|borrow_loaned)"),
    ("Batch receive (`take_sequence`)", ["take_sequence"],
     r"fn take_sequence"),
    ("Streamed publish", ["publish_streamed"], r"fn publish_streamed"),
    ("Connectivity ping", ["ping_session"], r"fn ping_session"),
    # ---- phase-376 W4 parity surface (declared slots; wiring in flight) ----
    ("Identity / feature probe",
     ["get_implementation_identifier", "feature_supported"],
     r"fn (implementation_identifier|feature_supported)"),
    ("Publisher GID / matched counts",
     ["get_gid_for_publisher", "publisher_count_matched_subscriptions"],
     r"fn (gid_for_publisher|count_matched)"),
    ("Actual-QoS read-back", ["publisher_get_actual_qos"],
     r"fn (get_actual_qos|actual_qos)"),
    ("Wait-for-acked", ["publisher_wait_for_all_acked"],
     r"fn wait_for_all_acked"),
    ("Take-with-info", ["take_with_info"], r"fn take_with_info"),
    # "Entity new-data callbacks" (`subscription_set_on_new_message_callback`)
    # was a row here until phase-407 retired the slot (issue 0960 DECLINED the
    # trio); the generator then refused to run for four days — and nothing
    # noticed, because `check rmw-feature-matrix` is on no lane. A row names a
    # slot the canonical vtable has; a declined capability is not one.
    ("Graph introspection (names/types/counts)",
     ["get_node_names", "get_topic_names_and_types", "count_publishers"],
     r"fn (get_node_names|get_topic_names)"),
]

# Node-layer features: not a backend slot; static rows with the rule.
NODE_LAYER = [
    ("Actions", "yes", "yes", "untested",
     "Built in `nros-node` on pub/sub + services. Cyclone has both since "
     "`service.cpp`, but no action example runs on it in CI — see "
     "[known limitations](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/reference/cyclonedds-known-limitations.md)."),
    ("Parameters (+ param services)", "yes", "yes", "yes",
     "Node-layer services (RFC-0004); available wherever services are."),
    ("Lifecycle (REP-2002)", "yes", "yes", "yes",
     "Node-layer state machine + services (`lifecycle-services` feature)."),
]

QOS_ORDER = [
    "CORE", "DURABILITY_TRANSIENT_LOCAL", "DEADLINE", "LIFESPAN",
    "LIVELINESS_AUTOMATIC", "LIVELINESS_MANUAL_BY_TOPIC",
    "LIVELINESS_MANUAL_BY_NODE", "LIVELINESS_LEASE",
    "AVOID_ROS_NAMESPACE_CONVENTIONS",
]


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf8") as fh:
        return fh.read()


def canonical_slots():
    """Every vtable slot, parsed from cffi's EMPTY_VTABLE literal (which is
    required to name every field). The authority for rename detection."""
    text = read(CFFI_EMPTY)
    m = re.search(r"pub const EMPTY_VTABLE[^{]*\{(.*?)\};", text, re.S)
    if not m:
        sys.exit("gen-rmw-feature-matrix: EMPTY_VTABLE not found in cffi lib.rs")
    return set(re.findall(r"([a-z_][a-z0-9_]*)\s*:\s*None", m.group(1)))


def parse_designated(text):
    """xrce shape: `.slot = value,` -> {slot: bool(wired)}.

    A slot ABSENT from a designated initializer is implicitly NULL —
    phase-376's new parity slots are exactly that, so absence means
    unwired, never an error (the canonical-slot check catches renames).
    """
    out = {}
    for m in re.finditer(r"^\s*\.([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([A-Za-z_][A-Za-z0-9_]*)",
                         text, re.M):
        out[m.group(1)] = m.group(2) != "NULL"
    return out


def parse_positional(text):
    """cyclone shape: `/*slot*/ value,` (comment names the slot).

    A value can be an identifier that is itself a typed-nullptr constant
    (`constexpr … (*kFoo)(…) = nullptr;` — the Phase-108 deferred event
    hooks are spelled exactly that way), so resolve those to NULL too:
    treating any non-`nullptr` identifier as wired reported Cyclone's
    status events as implemented, which is the drift this generator
    exists to prevent.
    """
    null_consts = set(re.findall(
        r"constexpr[^;=]*\(\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)[^;=]*=\s*nullptr\s*;",
        text, re.S))
    out = {}
    for m in re.finditer(r"/\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*\*/\s*([A-Za-z_:][A-Za-z0-9_:]*)",
                         text):
        val = m.group(2)
        out[m.group(1)] = val != "nullptr" and val not in null_consts
    return out


def zenoh_has(pattern):
    # Tracked crate sources — index lookup, not a walk (issue 0721).
    rx = re.compile(pattern)
    for path in tracked(os.path.join(ROOT, ZENOH_SRC_DIR), suffix=".rs"):
        if rx.search(path.read_text(encoding="utf8")):
            return True
    return False


def qos_mask(text, fn_name):
    """Collect QoSPolicyMask::X terms inside fn `fn_name`'s body."""
    m = re.search(rf"fn {fn_name}[^{{]*\{{", text)
    if not m:
        return None
    depth, i, start = 1, m.end(), m.end()
    while depth and i < len(text):
        depth += {"{": 1, "}": -1}.get(text[i], 0)
        i += 1
    body = text[start:i]
    return sorted(set(re.findall(r"QoSPolicyMask::([A-Z0-9_]+)", body)))


def cell(v):
    return {True: "wired", False: "—"}[v]


def render():
    cyc = parse_positional(read(CYCLONE_VTABLE))
    xrce = parse_designated(read(XRCE_VTABLE))
    zen_qos = qos_mask(read(ZENOH_SESSION), "supported_qos_policies")
    cffi_qos = qos_mask(read(CFFI_LIB), "supported_qos_policies")

    canon = canonical_slots()
    missing = [s for _l, slots, _z in FEATURES for s in slots if s not in canon]
    if missing:
        # A renamed/removed slot must fail the generator, not silently
        # render a `—` for a feature that merely moved. Absence from a
        # BACKEND's initializer is fine (implicitly NULL); absence from the
        # canonical EMPTY_VTABLE list is a rename.
        sys.exit(f"gen-rmw-feature-matrix: slot(s) not in the canonical "
                 f"vtable (EMPTY_VTABLE): {missing}")

    lines = [
        "<!-- GENERATED by scripts/gen-rmw-feature-matrix.py — do not edit by hand.",
        "     Regenerate: python3 scripts/gen-rmw-feature-matrix.py",
        "     Gated by:   just check rmw-feature-matrix -->",
        "",
        "# Per-RMW Feature Matrix",
        "",
        "One table per question the capability pages used to answer in",
        "contradictory prose. **Derived from the backend sources** — the two",
        "C vtables (`" + os.path.basename(CYCLONE_VTABLE) + "`,",
        "`" + os.path.basename(XRCE_VTABLE) + "`) and the zenoh shim's trait",
        "overrides — so a backend gaining or losing a slot moves this page in",
        "the same commit or fails the gate.",
        "",
        "`wired` = the backend implements it. `—` = not wired: the runtime",
        "surfaces `UNSUPPORTED` or falls back where a fallback exists (the",
        "vtable comments name which). The rows below the parity marker are",
        "the phase-376 W4 surface — slots declared in the ABI whose backend",
        "wiring is the in-flight campaign; all-dash rows there mean",
        "*declared, not yet wired anywhere*, and they flip automatically as",
        "backends land implementations.",
        "",
        "## Session / entity capabilities",
        "",
        "| Capability | Zenoh | XRCE-DDS | Cyclone DDS |",
        "|---|---|---|---|",
    ]
    for label, slots, zpat in FEATURES:
        zen = zenoh_has(zpat)
        xr = all(xrce.get(s, False) for s in slots)
        cy = all(cyc.get(s, False) for s in slots)
        lines.append(f"| {label} | {cell(zen)} | {cell(xr)} | {cell(cy)} |")

    lines += [
        "",
        "## Node-layer features",
        "",
        "These live in `nros-node` on top of pub/sub + services — they are",
        "not backend slots, so the rule, not a vtable, decides the row:",
        "",
        "| Feature | Zenoh | XRCE-DDS | Cyclone DDS | Rule |",
        "|---|---|---|---|---|",
    ]
    for label, z, x, c, why in NODE_LAYER:
        lines.append(f"| {label} | {z} | {x} | {c} | {why} |")

    lines += [
        "",
        "## QoS policies",
        "",
        "A backend advertises the policies it can enforce via",
        "`supported_qos_policies()`; requesting an unadvertised policy fails",
        "entity creation loudly (`INCOMPATIBLE_QOS`) — **no silent",
        "downgrade**. Per-policy semantics: [RMW vs upstream §7](../design/rmw-vs-upstream.md).",
        "",
        "| Policy | Zenoh | XRCE-DDS / Cyclone DDS (via C ABI) ¹ |",
        "|---|---|---|",
    ]
    for pol in QOS_ORDER:
        z = "✓" if pol in (zen_qos or []) else "—"
        c = "✓" if pol in (cffi_qos or []) else "—"
        lines.append(f"| `{pol}` | {z} | {c} |")
    lines += [
        "",
        "¹ The C-ABI backends' mask is asserted by the **runtime shim**",
        "(`packages/rmw/cffi/src/lib.rs`), not reported by the backend — the",
        "vtable has no `supported_qos_policies` slot yet (TODO 115.K.2.x).",
        "Treat the column as the runtime's assumption; the backend's own",
        "enforcement happens at entity creation.",
        "",
        "## Related",
        "",
        "- [Choosing an RMW Backend](../user-guide/rmw-choosing.md) — the",
        "  decision tree",
        "- [Backend Reference](../user-guide/rmw-backends.md) — architecture,",
        "  footprint, transports per backend",
        "- [Support Status](support-status.md) — versions, pins, and CI tiers",
        "",
    ]
    return "\n".join(lines)


def main():
    content = render()
    if "--check" in sys.argv:
        on_disk = open(OUT, encoding="utf8").read() if os.path.exists(OUT) else ""
        if on_disk != content:
            sys.exit("check-rmw-feature-matrix: STALE — regenerate with "
                     "python3 scripts/gen-rmw-feature-matrix.py and commit.")
        print("rmw-feature-matrix OK")
        return
    with open(OUT, "w", encoding="utf8") as fh:
        fh.write(content)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
