#!/usr/bin/env python3
"""A ledger row may not cite a file that does not exist.

phase-428 W6. The ledger validates existence in ONE direction: `--check` fails
on an unledgered difference, and nothing ever asks whether a row still has a
subject. Measured consequences at the time this was written:

* **43 rows cite `rclcpp_compat.hpp`**, deleted in stage 6 step A;
* six `rust:nros_*` log rows are `blocked-needs-decision` on a decision made
  2026-09-04, describing forwarders removed in phase-417 W-B5;
* `rust:init` is a mis-filed row whose prose is about `nros::logging::init`,
  so `nros::init()` — which HAS an rclrs counterpart — reads as having none.

That is the campaign's own generalisation ("nothing validates a REASON") one
layer up, at the ledger. This gate closes the cheapest, most mechanical half of
it: a row whose reason names a path can be checked against the filesystem.

WHAT IS CHECKED: every `path/to/file.ext` appearing in a row's prose, where the
extension is one this tree uses for source. A cited path must exist.

WHAT IS NOT: whether the LINE number is still right, whether the reason is
still true, or whether the row's subject still exists as a symbol. Those need
the extractor, not a filesystem stat — stated so a green here is not read as
more than it is.
"""
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
LEDGER = ROOT / "docs/reference/api-parity-ledger"
EXTS = ("hpp", "h", "rs", "py", "cpp", "c", "toml", "just", "md", "jinja", "sh")
PATH_RE = re.compile(r"\b((?:[\w.-]+/)+[\w.-]+\.(?:%s))\b" % "|".join(EXTS))


def cited_paths(text):
    for m in PATH_RE.finditer(text):
        p = m.group(1)
        # `docs/...` and `packages/...` are repo-relative; anything else (an
        # upstream include path, a URL fragment) is out of scope.
        if p.startswith(("packages/", "scripts/", "docs/", "just/", "cmake/", "examples/", "book/")):
            yield p


def scan(files):
    bad = []
    for f in files:
        rows = json.loads(f.read_text())
        for key, val in rows.items():
            if not isinstance(val, dict):
                continue
            blob = " ".join(str(v) for v in val.values() if isinstance(v, str))
            for p in cited_paths(blob):
                if not (ROOT / p).exists():
                    bad.append((f.name, key, p))
    return bad


def main():
    if "--self-test" in sys.argv:
        import tempfile

        d = pathlib.Path(tempfile.mkdtemp())
        (d / "planted.json").write_text(
            json.dumps(
                {
                    "cpp:real": {"why": "see packages/api/nros-cpp/include/nros/qos.hpp for the table"},
                    "cpp:orphan": {"why": "see packages/api/nros-cpp/include/nros/rclcpp_compat.hpp:428"},
                    "cpp:upstream": {"why": "upstream rclcpp/qos.hpp says otherwise"},
                }
            )
        )
        hits = scan([d / "planted.json"])
        keys = [k for _, k, _ in hits]
        assert "cpp:orphan" in keys, "self-test: the planted dead path was not caught: %s" % keys
        assert "cpp:real" not in keys, "self-test: a live path was flagged: %s" % keys
        assert "cpp:upstream" not in keys, (
            "self-test: an UPSTREAM path was flagged; only repo-relative paths are ours: %s" % keys
        )
        print("check-ledger-orphan-refs --self-test: 3 case(s) OK")
        return 0

    files = sorted(LEDGER.glob("*.json"))
    if not files:
        print("check-ledger-orphan-refs: no ledger shards found at %s" % LEDGER, file=sys.stderr)
        return 1
    bad = scan(files)
    if bad:
        print(
            "FAIL: %d ledger row(s) cite a file that does not exist.\n"
            "      A reason naming a deleted file is a reason nobody can check.\n" % len(bad),
            file=sys.stderr,
        )
        for shard, key, p in bad[:40]:
            print("  %-16s %-44s -> %s" % (shard, key, p), file=sys.stderr)
        if len(bad) > 40:
            print("  … and %d more" % (len(bad) - 40), file=sys.stderr)
        return 1
    print(
        "check-ledger-orphan-refs: OK — %d shard(s), every cited repo path exists"
        % len(files)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
