"""RFC-0079 priority address plans — ONE reader for both consumers.

`check-tier-priority-plan.py` enforces plans; `dev/priority-collision-report.py`
reports on them. They had a table each for about an hour, which is the second
spelling this codebase keeps paying for — and the plan tables are precisely the
thing whose whole value is being the single place a band is written down.
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tracked import tracked  # issue 0721: index lookup, not a walk

ROOT = Path(__file__).resolve().parent.parent.parent

TIER_PLAT_RE = re.compile(r"^\[tiers\.([A-Za-z0-9_]+)\.([A-Za-z0-9_]+)\]\s*$")
TIER_RE = re.compile(r"^\[tiers\.([A-Za-z0-9_]+)\]\s*$")
PRIO_RE = re.compile(r"^priority\s*=\s*(-?\d+)")
PAIR_RE = re.compile(r"\[\s*(-?\d+)\s*,\s*(-?\d+)\s*\]")


def _parse_plan_block(text, header, source, platform=None):
    """Pull one `[<header>]` table out of `text` as a plan dict.

    Shared by the platform and board readers so the two cannot drift — the
    thing this module's own docstring says is the whole point.
    """
    # NOT `if header not in text` — that is satisfied by a COMMENT naming the
    # table, and this file's own strip-comment note documents exactly that
    # hazard for `<nano_ros_provides>` (issue 0516). Measured here: the boards
    # that MOVED their plan to the platform kept a comment saying where it went,
    # and the substring test then produced an empty plan that "conflicted" with
    # the platform's. The table must actually be ENTERED.
    if header not in text:
        return None
    in_plan = False
    seen_table = False
    plan = {"reserved": {}, "pool": {}, "source": source}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if platform is None and line.startswith("platform ="):
            platform = line.split("=", 1)[1].strip().strip('"')
        if line == header:
            in_plan = True
            seen_table = True
            continue
        if in_plan and line.startswith("["):
            in_plan = False
        if not in_plan:
            continue
        if line.startswith("derived"):
            plan["derived"] = line.split("=", 1)[1].strip().strip('"')
        elif line.startswith("resolver"):
            plan["resolver"] = line.split("=", 1)[1].strip().strip('"')
        elif line.startswith("tier_key"):
            plan["tier_key"] = line.split("=", 1)[1].strip().strip('"')
        elif line.startswith("direction"):
            plan["direction"] = line.split("=", 1)[1].strip().strip('"')
        elif line.startswith("range"):
            m = PAIR_RE.search(line)
            plan["range"] = (int(m.group(1)), int(m.group(2)))
        elif line.startswith("reserved."):
            name = line.split("=", 1)[0].strip().split(".", 1)[1]
            m = PAIR_RE.search(line)
            if m is None:
                plan.setdefault("empty_bands", []).append(name)
                continue
            plan["reserved"][name] = (int(m.group(1)), int(m.group(2)))
        elif line.startswith("pool."):
            name = line.split("=", 1)[0].strip().split(".", 1)[1]
            m = PAIR_RE.search(line)
            plan["pool"][name] = (int(m.group(1)), int(m.group(2)))
    if not seen_table:
        return None
    return plan, platform


def load_platform_plans():
    """tier_key -> plan, from `config/*/nros-platform.toml`.

    phase-375 W8 — a priority plan is a PLATFORM fact (`range = [1, 7]` is
    FreeRTOS's `configMAX_PRIORITIES`), and stating it on boards meant two
    boards of one platform held byte-identical copies. Worse than untidy:
    [`load_plans`] keys by platform, so the later board silently won and a
    divergence would have been invisible.
    """
    plans = {}
    for desc in sorted(tracked(ROOT / "config", name="nros-platform.toml")):
        text = desc.read_text(encoding="utf-8")
        got = _parse_plan_block(text, "[priority_plan]", desc.relative_to(ROOT))
        if got is None:
            continue
        plan, _ = got
        key = plan.pop("tier_key", None)
        if key:
            plans[key] = plan
    return plans


def load_plans():
    """platform -> plan: the platform's, overridden by a board that states one."""
    plans = load_platform_plans()
    for desc in sorted(tracked(ROOT / "packages/boards", name="nros-board.toml")):
        text = desc.read_text(encoding="utf-8")
        # ONE parser, shared with `load_platform_plans`. This loop carried its
        # own copy until phase-375 W8 — in the module whose docstring says the
        # two consumers "had a table each for about an hour, which is the second
        # spelling this codebase keeps paying for".
        got = _parse_plan_block(
            text, "[board.priority_plan]", desc.relative_to(ROOT)
        )
        if got is None:
            continue
        plan, platform = got
        # The key a `[tiers.<name>.<key>]` pin is written under. Usually the
        # board's own `platform`, but not always: both ThreadX boards are
        # `threadx-linux` / `threadx-riscv64` while bringups write
        # `[tiers.*.threadx]`. FreeRTOS and NuttX matched by coincidence, which
        # is not a thing to build on — so the plan states it.
        key = plan.pop("tier_key", None) or platform
        if not key:
            continue
        prev = plans.get(key)
        if prev is not None and (prev["reserved"], prev["pool"],
                                 prev.get("direction")) != (plan["reserved"],
                                                            plan["pool"],
                                                            plan.get("direction")):
            raise SystemExit(
                f"two boards claim tier_key {key!r} with DIFFERENT plans:\n"
                f"  {prev['source']}\n  {plan['source']}\n"
                "One tier key is one address space; if these ports really "
                "differ, they need different keys.")
        plans[key] = plan
    return plans



ABOVE_RE = re.compile(r'^above\s*=\s*"([A-Za-z0-9_]+)"')


def scan_pins():
    """-> [(path, tier, platform, priority, above)] over every system.toml.

    `above` is the band this tier DELIBERATELY outranks, declared on the tier
    (RFC-0079 §6) and inherited by each of its per-platform pins:

        [tiers.safety]
        above = "transport"      # states the choice once, for every port

    It sits on the tier rather than the platform table because it is a
    statement about the SYSTEM, not about one kernel's numbering — the same
    reason the timing contract lives there.
    """
    pins = []
    for f in sorted(tracked(ROOT / "examples", name="system.toml")):
        tier = plat = None
        above = {}
        for raw in f.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            m = TIER_PLAT_RE.match(line)
            if m:
                tier, plat = m.group(1), m.group(2)
                continue
            m = TIER_RE.match(line)
            if m:
                tier, plat = m.group(1), None
                continue
            if line.startswith("["):
                tier = plat = None
                continue
            m = ABOVE_RE.match(line)
            if m and tier and plat is None:
                above[tier] = m.group(1)
                continue
            m = PRIO_RE.match(line)
            if m and tier and plat:
                pins.append((f.relative_to(ROOT), tier, plat,
                             int(m.group(1)), above.get(tier)))
    return pins


# ---------------------------------------------------------------------------
# Derived bands (RFC-0079 §4.1) — Zephyr
# ---------------------------------------------------------------------------
#
# Every other port's reserved band is a LITERAL read off the port: FreeRTOS 4,
# NuttX 100, ThreadX 14. Zephyr's is computed, per image, from Kconfig — so a
# literal in the descriptor would be true for one build and quietly wrong for
# the next, which is the shape RFC-0079 exists to eliminate one level up
# (issue 0766).
#
# The chain, each link with the file it comes from:
#
#   CONFIG_NROS_ZENOH_{READ,LEASE}_PRIORITY   zephyr/cmake/nros_rmw_zenoh.cmake
#     -> ZPICO_{READ,LEASE}_TASK_PRIORITY     zpico.c (band 0..31)
#     -> POSIX priority                       zpico_posix_set_priority():
#                                             lo + (span*n*2 + 31)/62,
#                                             lo = 0, hi = NUM_PREEMPT - 1
#     -> k_thread priority                    zephyr/lib/posix/options/pthread.c
#                                             POSIX_TO_ZEPHYR_PRIORITY, SCHED_RR:
#                                             NUM_PREEMPT - prio - 1
#
# Tiers are RAW k_thread priorities, so the band has to end in k_thread units
# for `reserved.transport` to mean anything against them.

ZPICO_BAND_MAX = 31


def _dotconfig_ints(path):
    """`CONFIG_x=<int>` pairs out of a Zephyr `.config`."""
    out = {}
    for raw in pathlib_Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        v = v.strip()
        if v.lstrip("-").isdigit():
            out[k.strip()] = int(v)
    return out


NROS_PLATFORM_PRIORITY_MAX = 255


def zpico_band_to_posix(band, num_preempt):
    """The band -> POSIX half, mirroring `nros_zephyr_native_priority`.

    issue 0852 — this used to mirror `zpico_posix_set_priority`, a 0-31 map
    that wrote into a `pthread_attr_t` NOTHING ON ZEPHYR EVER READ. So this
    gate certified a band no image ever had, while `realtime-c`'s `system.toml`
    authored tier values against that fiction and the declared ordering was in
    fact inverted. A static gate that models dead code cannot see the bug it
    exists to catch; that function is now deleted and this mirrors the live one
    (`nros-platform-zephyr/src/platform.c:nros_zephyr_native_priority`):

        want = lo + band * (hi - lo) // NROS_PLATFORM_PRIORITY_MAX

    truncating, and clamped to [lo, hi] — the platform ABI's 0-255 band, not a
    private 0-31 one.
    """
    lo, hi = 0, num_preempt - 1
    if hi < lo:
        return None
    n = min(max(band, 0), NROS_PLATFORM_PRIORITY_MAX)
    want = lo + (n * (hi - lo)) // NROS_PLATFORM_PRIORITY_MAX
    return min(max(want, lo), hi)


def posix_rr_to_kthread(posix_prio, num_preempt):
    """`POSIX_TO_ZEPHYR_PRIORITY(prio, SCHED_RR)` — pthread.c:25."""
    return num_preempt - posix_prio - 1


def resolve_zephyr_plan(dotconfig):
    """Resolve a Zephyr `[board.priority_plan]` against ONE image's `.config`.

    Returns the same shape a static plan has, plus `derived_from` so a reader
    can see which image produced it, or an `unapplied` note when the image's
    Kconfig means the priority is never set at all — in which case the
    transport INHERITS its creator and the band is not a choice, the way NuttX
    read before issue 0736.
    """
    cfg = _dotconfig_ints(dotconfig)
    has = lambda k: _dotconfig_has(dotconfig, k)
    num_preempt = cfg.get("CONFIG_NUM_PREEMPT_PRIORITIES")
    num_coop = cfg.get("CONFIG_NUM_COOP_PRIORITIES", 0)
    if num_preempt is None:
        return {"error": "CONFIG_NUM_PREEMPT_PRIORITIES absent — not a Zephyr .config?"}

    # Both gates must be on, or `nros_zephyr_native_priority` returns -1 and
    # the tasks inherit their creator (platform.c + issue 0766).
    if not (has("CONFIG_POSIX_PRIORITY_SCHEDULING") and has("CONFIG_PREEMPT_ENABLED")):
        return {
            "unapplied": "CONFIG_POSIX_PRIORITY_SCHEDULING and/or "
                         "CONFIG_PREEMPT_ENABLED is off — the transport priority "
                         "is NOT applied in this image; the tasks inherit their "
                         "creator and no band can be reserved",
            "derived_from": str(dotconfig),
        }

    bands = {}
    for name, key in (("read", "CONFIG_NROS_ZENOH_READ_PRIORITY"),
                      ("lease", "CONFIG_NROS_ZENOH_LEASE_PRIORITY")):
        # issue 0852 — Kconfig's default on the 0-255 band, not the old 0-31 16.
        band = cfg.get(key, 200)
        posix = zpico_band_to_posix(band, num_preempt)
        bands[name] = {"band": band, "posix": posix,
                       "kthread": posix_rr_to_kthread(posix, num_preempt)}

    ks = sorted(b["kthread"] for b in bands.values())
    return {
        "direction": "smaller-is-urgent",
        # Zephyr's usable k_thread range: negatives are cooperative.
        "range": (-num_coop, num_preempt - 1),
        "reserved": {"transport": (ks[0], ks[-1])},
        # Tiers allocate strictly LESS urgent than the transport — numerically
        # larger here.
        "pool": {"app": (ks[-1] + 1, num_preempt - 1)},
        "detail": bands,
        "derived_from": str(dotconfig),
        "source": "derived from Kconfig (RFC-0079 §4.1)",
    }


def _dotconfig_has(path, key):
    """True when `<key>=y` is set."""
    for raw in pathlib_Path(path).read_text(encoding="utf-8").splitlines():
        if raw.strip() == f"{key}=y":
            return True
    return False


from pathlib import Path as pathlib_Path  # noqa: E402  (used above)
