#!/usr/bin/env python3
"""Correlate a nano-ros API surface against the ROS 2 client library it mirrors.

Phase 379. Consumes the records `extract_cxx.py` / `extract_rust.py` produce.

# What "correlate" means here

The campaign's claim is that nano-ros is the SAME API as rclc / rclcpp / rclrs,
diverging only where an RTOS forces it. That claim is only checkable if every
item on either side has been ACCOUNTED FOR -- matched, or classified with a
reason. So this produces four buckets and no fifth:

  same       -- names correspond and the arguments agree
  arity-only -- names correspond, the arities overlap, and no parameter position
                holds a compatible type: the agreement is in the COUNT. `init`
                is the example -- ours takes (locator, domain), rclcpp's takes
                (argc, argv), and both take two.
  systematic -- names correspond, the arguments differ, and a SIGNATURE RULE
                explains it: one platform decision applied everywhere (no
                allocator, compile-time QoS, callback bound at creation). The
                rule is stated once in `signature_rules.py`; the row inherits
                its constraint instead of needing a ledger entry of its own.
  differs    -- names correspond, the arguments differ, and NO rule explains it
                (the campaign's work list; each needs a decision, not a shrug)
  ours-only  -- we ship it and ROS 2 does not (an RTOS extension, or a name we
                invented where ROS 2 already had one)
  theirs-only-- ROS 2 ships it and we do not (a gap, or a deliberate decline)

`--check` fails on anything whose bucket is not `same` and has no row in the
ledger. A ledger row is a sentence somebody wrote; the point of the gate is that
no divergence can enter the tree without one.

# Why matching is by NORMALIZED NAME and not by an authored map

An authored map for ~700 items is a document nobody finishes and nobody
re-reads. Names already correspond by construction -- the project's stated goal
is that they do -- so the tool assumes correspondence and makes DISAGREEMENT the
thing a human has to write about. That inverts the labour onto exactly the rows
the campaign cares about.

Normalisation is per-language because each library spells the same idea its own
way:

  C++    `nros::Node::create_publisher` <-> `rclcpp::Node::create_publisher`
         -- drop the namespace, keep `Type::method`.
  C      `nros_publisher_init`          <-> `rclc_publisher_init`
         -- drop the library prefix, keep the rest verbatim.
  Rust   `nros::node::NodeCtx::create_publisher`
                                        <-> `rclrs::NodeState::create_publisher`
         -- drop the module path, and fold rclrs's `XState` naming (0.5+ made
            `Node = Arc<NodeState>`) onto the name a user actually writes.

A rename we CHOSE still needs saying out loud, so the ledger can also assert an
explicit pair; those are matched before normalisation is tried.
"""

import difflib
import re

import signature_rules


# rclrs 0.5 split every handle into `X` (an `Arc<XState>` alias) and `XState`
# (the inherent impl). A user writes `Node`; the methods live on `NodeState`.
# Folding the suffix is what makes `rclrs::NodeState::create_publisher` line up
# with `nros::NodeCtx::create_publisher` instead of reading as two gaps.
_RCLRS_STATE = re.compile(r"^(.*?)State$")

# rclcpp splits every entity into a type-erased base and a typed subclass:
# `Publisher<T>` IS-A `PublisherBase`, and half the methods a user calls
# (`get_topic_name`, `assert_liveliness`, `wait_for_service`, `cancel`) are
# declared on the base. Our `nros::Publisher` is one class, so without folding
# the suffix those methods report as an `ours-only` row and a `theirs-only` row
# that never mention each other -- inventing a divergence out of an inheritance
# split. Exactly the rclrs `XState` case one library over.
_RCLCPP_BASE = re.compile(r"^(.*?)Base$")

# Not every `*Base` is that split. These are real, separate rclcpp types whose
# names happen to end in Base, and folding them would merge two distinct APIs.
_BASE_KEEP = {"NodeBase", "MemoryStrategyBase", "AllocatorMemoryStrategyBase"}

# Our Rust node handle is `NodeCtx` for the reason RFC-0022 gives (no `Arc<Node>`;
# a short-lived borrow instead). It is the same entity as rclrs's `Node`, so the
# correlator must see through the name even though the name is deliberate.
TYPE_SYNONYMS = {
    "rust": {"NodeCtx": "Node", "NodeState": "Node"},
    "c++": {},
    "c": {},
}

# Longest first: `rclc_` must be stripped before `rcl_`, or every rclc symbol
# normalises to a stray leading `c_`.
LIB_PREFIXES = {
    # phase-428 W2 — OUR side strips the rcl/rclc prefixes too, or an adopted
    # name normalises to `rcl_node_fini` on our side and `node_fini` on
    # theirs, which can never match. Longest-first so `rclc_` wins over `rcl_`.
    "c": (
        ("nros_", "NROS_", "rclc_", "RCLC_", "rcl_", "RCL_"),
        ("rclc_", "RCLC_", "rcl_", "RCL_"),
    ),
}


def _strip_prefix(name, prefixes):
    for p in prefixes:
        if name.startswith(p):
            return name[len(p) :]
    return name


def _last_two(qual):
    """`a::b::Type::method` -> `Type::method`; `a::b::free_fn` -> `free_fn`."""
    parts = [p for p in qual.split("::") if p]
    return "::".join(parts[-2:]) if len(parts) >= 2 else (parts[-1] if parts else "")


def normalize(lang, side, qual, kind):
    """Map a qualified item name onto the language-neutral key used for matching."""
    if lang == "c":
        ours, theirs = LIB_PREFIXES["c"]
        return _strip_prefix(qual, ours if side == "ours" else theirs)

    parts = [p for p in qual.split("::") if p]
    if not parts:
        return qual
    # Drop the crate/library root; keep the tail that a user actually types.
    tail = parts[-1]
    owner = parts[-2] if len(parts) >= 2 else ""

    if lang == "c++":
        m = _RCLCPP_BASE.match(owner)
        if m and m.group(1) and owner not in _BASE_KEEP:
            owner = m.group(1)
        # The TYPE itself must fold too, not only a method's owner: `flatten`
        # builds each member key from its record's type key, so folding only
        # `owner` leaves `rclcpp::PublisherBase`'s methods keyed under
        # `PublisherBase::` and changes nothing.
        m = _RCLCPP_BASE.match(tail)
        if m and m.group(1) and tail not in _BASE_KEEP and kind in ("type", "enum", "alias"):
            tail = m.group(1)

    if lang == "rust":
        m = _RCLRS_STATE.match(owner)
        if m and m.group(1):
            owner = m.group(1)
        owner = TYPE_SYNONYMS["rust"].get(owner, owner)
        # The TYPE itself must fold too, exactly as the C++ `*Base` case does:
        # `flatten` builds member keys from the type key, so folding only the
        # method owner leaves `rclrs::PublisherState`'s members keyed under
        # `PublisherState::` and they never meet ours. Missing this understated
        # `same` across the whole Rust lane.
        if kind in ("type", "enum", "alias"):
            m = _RCLRS_STATE.match(tail)
            if m and m.group(1):
                tail = m.group(1)
            tail = TYPE_SYNONYMS["rust"].get(tail, tail)

    if kind in ("type", "enum", "alias", "const", "macro"):
        # A type's identity is its own name; its module path is not part of the
        # API the way a method's owning type is.
        return tail
    return ("%s::%s" % (owner, tail)) if owner and owner[:1].isupper() else tail


# Reachability marks a RECORD may carry, and how they combine when several
# records land on one key.
#
# `flatten` merges records: `nros::Node::create_publisher` from the base TU and
# `rclcpp::Node::create_publisher` from the compat shim are two records under
# one key. A mark that says "you can only reach this under condition X" is
# therefore an AND across every record that contributed -- if ANY record is
# reachable without X, the item is.
#
# Both marks were previously set on the record and then DROPPED here, which is
# why `std_only` had no consumer anywhere in the tree for two phases: the
# extractor tagged it (`api-parity.py:190`) and `flatten` threw it away one
# call later. `surface` would have died the same way.
REACH_MARKS = ("std_only",)

# `surface` is not a boolean, so it combines by its own rule: an item reachable
# from the NATIVE headers is native, whatever else also declares it.
NATIVE = "native"
PORTED = "ported"


def _merge_marks(slot, rec):
    """Fold one record's reachability marks into the item it merged into."""
    for mark in REACH_MARKS:
        slot[mark] = bool(slot.get(mark, True)) and bool(rec.get(mark))
    cur = rec.get("surface") or NATIVE
    prev = slot.get("surface")
    slot["surface"] = cur if prev is None else (
        NATIVE if NATIVE in (prev, cur) else cur)


def flatten(records, lang, side):
    """Records -> {key: item}, one entry per callable or type.

    A class contributes its own key AND one per public method, because a method
    is where the arguments live and arguments are the question.
    """
    out = {}
    for rec in records:
        kind = rec["kind"]
        key = normalize(lang, side, rec["qual"], kind)
        header = rec.get("header", "")
        if kind in ("type", "enum"):
            _merge_marks(
                out.setdefault(
                    key,
                    {
                        "key": key,
                        "kind": kind,
                        "qual": rec["qual"],
                        "header": header,
                        "values": rec.get("values"),
                    },
                ),
                rec,
            )
            for m in rec.get("members", []):
                if m.get("field"):
                    continue
                mkey = "%s::%s" % (key, m["name"])
                # Overloads collapse onto one key; the arity set is what the
                # report compares, so an overload difference still shows up.
                slot = out.setdefault(
                    mkey,
                    {
                        "key": mkey,
                        "kind": "method",
                        "qual": "%s::%s" % (rec["qual"], m["name"]),
                        "header": header,
                        "overloads": [],
                    },
                )
                slot["overloads"].append(
                    {
                        "params": m.get("params", []),
                        "ret": m.get("ret", ""),
                        "template": m.get("template", []),
                    }
                )
                _merge_marks(slot, rec)
        elif kind == "function":
            slot = out.setdefault(
                key,
                {"key": key, "kind": "function", "qual": rec["qual"],
                 "header": header, "overloads": []},
            )
            slot["overloads"].append(
                {
                    "params": rec.get("params", []),
                    "ret": rec.get("ret", ""),
                    "template": rec.get("template", []),
                }
            )
            _merge_marks(slot, rec)
        else:
            _merge_marks(
                out.setdefault(
                    key,
                    {"key": key, "kind": kind, "qual": rec["qual"], "header": header},
                ),
                rec,
            )
    return out


# Type spellings that mean the same thing on both sides. These are NOT
# divergences to report -- reporting them would bury the real ones under
# hundreds of rows saying `std::string` differs from `const char *`, which
# RFC-0018 already decided once and for all.
_TYPE_NOISE = [
    (re.compile(r"\bconst\s+"), ""),
    (re.compile(r"\b(struct|enum|union)\s+"), ""),
    # The library prefix is not part of the type's identity, exactly as it is
    # not part of a name's: `struct nros_client_t *` and `rcl_client_t *` are
    # the same concept spelled by two libraries. Without this, every handle
    # parameter reads as a type mismatch.
    (re.compile(r"\b(nros|rclc|rcl|rmw|rosidl)_"), ""),
    (re.compile(r"\s*&\s*"), "&"),
    (re.compile(r"\s*\*\s*"), "*"),
    (re.compile(r"\s+"), " "),
    (re.compile(r"^rclcpp::"), ""),
    (re.compile(r"^rclcpp_action::"), ""),
    (re.compile(r"^rclrs::"), ""),
    (re.compile(r"^nros::"), ""),
    (re.compile(r"^std::"), ""),
]


def canon_type(t):
    s = t or ""
    for pat, rep in _TYPE_NOISE:
        s = pat.sub(rep, s)
    return s.strip()


def arity_set(item):
    """Every parameter count this name accepts, DEFAULT ARGUMENTS INCLUDED.

    Arity, not full types, is the primary comparison: a type difference is
    usually RFC-0018's `std::string` -> `const char*` rule applied again, while
    an ARITY difference means the two APIs ask the user for different things.
    Full types are still reported alongside so a reader can judge.

    A defaulted parameter widens the range rather than fixing it. Counting only
    declared parameters reported `nros::Executor::spin(int32_t poll_ms = 10)` as
    diverging from `rclcpp::Executor::spin()` -- when `exec.spin()` compiles in
    both, which is the entire point of issue 0338's fix. A checker that flags a
    convergence someone deliberately made is worse than no checker.
    """
    out = set()
    for o in item.get("overloads", []):
        params = o["params"]
        required = sum(1 for p in params if not p.get("default"))
        for n in range(required, len(params) + 1):
            out.add(n)
    return out or {0}


def compare(ours, theirs, lang):
    """Bucket every key present on either side.

    `signature_rules` is consulted only after a plain arity comparison fails, so
    a rule can never turn an agreement into an explanation.
    """
    rows = []
    for key in sorted(set(ours) | set(theirs)):
        o = ours.get(key)
        t = theirs.get(key)
        if o and not t:
            rows.append({"key": key, "bucket": "ours-only", "ours": o, "theirs": None})
        elif t and not o:
            rows.append({"key": key, "bucket": "theirs-only", "ours": None, "theirs": t})
        else:
            oa, ta = arity_set(o), arity_set(t)
            if o["kind"] in ("method", "function") or t["kind"] in ("method", "function"):
                if oa & ta:
                    arity_only, subs = arity_verdict(o, t, key)
                    if arity_only:
                        bucket, detail = "arity-only", {
                            "ours_arity": sorted(oa),
                            "theirs_arity": sorted(ta),
                            "rules": [],
                        }
                    elif subs:
                        bucket, detail = "systematic", {
                            "ours_arity": sorted(oa),
                            "theirs_arity": sorted(ta),
                            "rules": subs,
                        }
                    else:
                        bucket, detail = "same", None
                else:
                    rules = signature_rules.explain(key, o, t)
                    bucket = "systematic" if rules else "differs"
                    detail = {
                        "ours_arity": sorted(oa),
                        "theirs_arity": sorted(ta),
                        "rules": rules,
                    }
            else:
                bucket = "same"
                detail = None
            rows.append(
                {"key": key, "bucket": bucket, "ours": o, "theirs": t, "detail": detail}
            )
    return rows


def render_params(item):
    if not item or not item.get("overloads"):
        return ""
    shown = []
    for o in item["overloads"][:3]:
        shown.append("(" + ", ".join(canon_type(p["type"]) for p in o["params"]) + ")")
    return " | ".join(sorted(set(shown)))


# A rename is a naming difference the bucket report CANNOT show: it splits into
# an `ours-only` row and a `theirs-only` row that never mention each other. Since
# a rename with no platform reason is precisely what this campaign exists to
# find, the pairs are worth surfacing -- but by SIMILARITY, which is a guess.
#
# So these are printed as SUGGESTIONS and never as findings, and they never
# satisfy `--check`. A human confirms the pair and writes the ledger row; the
# tool's job is to stop the pair being invisible.
def suggest_renames(rows, cutoff=0.72):
    """[(ours_key, theirs_key, ratio)] for unmatched names that look alike."""
    ours_only = [r["key"] for r in rows if r["bucket"] == "ours-only"]
    theirs_only = [r["key"] for r in rows if r["bucket"] == "theirs-only"]
    out = []
    for key in ours_only:
        match = difflib.get_close_matches(key, theirs_only, n=1, cutoff=cutoff)
        if not match:
            continue
        ratio = difflib.SequenceMatcher(None, key, match[0]).ratio()
        out.append((key, match[0], ratio))
    out.sort(key=lambda x: -x[2])
    return out


# A shared arity does not mean a shared meaning. `nros::init(const char* locator,
# uint8_t domain)` and `rclcpp::init(int argc, char** argv)` both take two
# parameters and have nothing to do with each other, and the bucket report calls
# that `same`.
#
# So `same` is qualified: if two signatures share an arity but NO parameter
# position holds a compatible type, the agreement is in the count only. That is
# reported rather than silently accepted -- it is a weaker claim than `same`,
# and a stage that trusts it will skip a real divergence.
#
# Type comparison runs through `canon_type`, so a library prefix, a `struct`
# keyword and a `const` do not count as a difference; what is left is a genuine
# disagreement about what the parameter IS.
def _effective_arities(params):
    """Every call arity this parameter list accepts, defaults trimmed off."""
    required = sum(1 for p in params if not p.get("default"))
    return range(required, len(params) + 1)


def _positions_agree(ours_overload, theirs_overload):
    """(agree, rules) for one overload pair.

    Compared at the SHARED arity with defaulted trailing parameters trimmed, not
    at the declared length. `spin(int32_t poll_ms = 10)` against `spin()` agree
    at arity 0 -- comparing declared lengths would call them a mismatch and
    re-report the convergence issue 0338 landed on purpose, one bucket over.

    A position may agree OUTRIGHT or through a type substitution
    (`signature_rules.TYPE_EQUIVALENCES`): `const char *` against
    `const std::string &` is RFC-0018's decision, not a disagreement about what
    the parameter is. `rules` names the substitutions that were needed, so the
    report can say WHICH decision made two spellings the same.
    """
    a, b = ours_overload["params"], theirs_overload["params"]
    shared = set(_effective_arities(a)) & set(_effective_arities(b))
    if not shared:
        return False, []
    for n in sorted(shared):
        if n == 0:
            return True, []
        rules = []
        for pa, pb in zip(a[:n], b[:n]):
            ca, cb = canon_type(pa["type"]), canon_type(pb["type"])
            if ca == cb or ca.rstrip("*&") == cb.rstrip("*&"):
                return True, []
            rule = signature_rules.substitution(ca, cb)
            if rule and rule not in rules:
                rules.append(rule)
        if rules:
            return True, rules
    return False, []


def shares_only_arity(ours_item, theirs_item):
    """True when the arities overlap and no overload pair agrees on a position."""
    return arity_verdict(ours_item, theirs_item)[0]


def _try_alignment(key, ours_item, theirs_item):
    """Retry the position comparison with an alignment rule applied."""
    rule = signature_rules.alignment_for(key)
    if rule is None:
        return None
    drop = rule.get("drop_ours", 0)
    for a in ours_item.get("overloads") or []:
        if len(a["params"]) <= drop:
            continue
        shifted = {"params": a["params"][drop:], "ret": a.get("ret", "")}
        for b in theirs_item.get("overloads") or []:
            agree, subs = _positions_agree(shifted, b)
            if agree:
                return [rule["id"]] + subs
    return None


def arity_verdict(ours_item, theirs_item, key=""):
    """(arity_only, rules) -- rules explain the spelling where they agree.

    `arity_only` means the count matches and NOTHING else does. When a type
    substitution reconciles the position, the row is not arity-only: it is a
    systematic divergence, and the rules say which one.
    """
    oo = (ours_item or {}).get("overloads") or []
    to = (theirs_item or {}).get("overloads") or []
    if not oo or not to:
        return False, []
    best = None
    for a in oo:
        for b in to:
            agree, rules = _positions_agree(a, b)
            if agree and not rules:
                return False, []
            if agree and (best is None or len(rules) < len(best)):
                best = rules
    if best is not None:
        return False, best
    aligned = _try_alignment(key, ours_item or {}, theirs_item or {})
    if aligned:
        return False, aligned
    return True, []
