---
id: 1079
title: "`fixture_group_collision_gate.sh` appends a bogus `[[bin]]` to two TRACKED example `Cargo.toml`s while ~20 other gates are reading them"
status: resolved
type: bug
area: testing, tooling
severity: medium
related: [1077, 0196]
found: 2026-09-05
resolved: 2026-09-05
---

# A gate that proves itself by editing the tree everything else is reading

`packages/testing/nros-tests/tests/fixture_group_collision_gate.sh` proves its
gate can fire by creating the collision for real. It picked two leaves out of the
fixture manifest — real, tracked example directories —

```sh
a="${leaves[0]}"          # from `fixtures-manifest.py list --lang rust`
b="${leaves[1]}"
…
printf '\n[[bin]]\nname = "%s"\npath = "src/main.rs"\n' "$collider" >> "$f"
```

and restored them from an `EXIT` trap. Proving a gate fires by making the defect
is the right instinct — a gate nobody watched fail is not a gate. Doing it in
the shared worktree is the problem.

## The measurement (2026-09-05) — the hazard is real, and the read race is the SMALLER half

The original filing was explicit that this was a structural hazard read off the
code and not a measured failure. It is now measured. Three experiments, all on
this host, all keyed on the literal string `tripwire_collider`, which appears
nowhere in the tracked tree and is written only by this script — so a positive
observation is attributable no matter what else is running in the checkout.
Reds that did NOT name the collider were counted separately and discarded as
unattributable; there were none.

### 1. The perturbation window is 1.5 s of a 3.1 s run

A 500 Hz poller over the two leaf manifests during one run:
`samples=3256 dirty=686 frac=0.211 window=1.503s` — two intervals (T1 and T2),
~1.5 s of tracked-file dirtiness per invocation.

### 2. "~20 other gates read those manifests" is true, but only ONE of them cares

Perturbed both leaves with a SUPERSET of what the script does (T1's extra
`[[bin]]` *and* T2's `[lib]` rewrite at once) and ran all 37 runnable gates that
mention `Cargo.toml` under `scripts/check-*`:

```sh
grep -rln "Cargo\.toml" scripts/check-*.py scripts/check-*.sh | sort
```

**36 of 37 passed unchanged and never mentioned the collider.** The lone
sensitive gate is `check-fixture-groups.py` — the very gate this script
tripwires. So the issue's framing was too broad: an extra `[[bin]]` in a leaf
manifest is invisible to the rest of the fan-out. (Excluded: two gates that take
an ELF argument, `check-no-alloc-image.py` and `check-stack-floor.py`.)

That single observer is enough, because it is on `check-fast`.

### 3. A concurrent `check-fixture-groups.py` sees it 51 % of the time

`check-fixture-groups.py` looping continuously while this script ran 10 times:

| | observer runs | runs reporting `tripwire_collider` | unattributable reds |
| --- | --- | --- | --- |
| before the fix | 43 | **22 (51 %)** | 0 |
| after the fix | 50 | **0** | 0 |

A red naming a binary nobody authored, from a gate that reads only tracked
files. Inside ONE `check-fast` the two cannot collide (the `fixture-groups`
recipe runs the gate and then the tripwire, sequentially), so this is the
two-writers case — a second agent, or a second `just check` — which is exactly
the situation the original sighting could not attribute.

### 4. The worse half: two concurrent runs CORRUPT a tracked file and both report PASS

Three pairs of concurrent invocations. Pair 3 left
`examples/qemu-arm-baremetal/rust/action-server-rtic/Cargo.toml` carrying

```toml
[[bin]]
name = "tripwire_collider"
path = "src/main.rs"
```

while **both processes printed `all checks passed` and exited 0**. No `SIGKILL`
required: run A took its backup while run B had already perturbed, so A's
"restore" wrote the perturbation back. The `EXIT`-trap argument in the original
filing understated this — the trap running is not sufficient, because the trap
restores whatever the *other* run's perturbation left in the backup.

After the fix: 3/3 pairs green, 0 dirty, both manifests byte-identical.

## The fix — a scratch overlay, not an in-place edit with a trap

The perturbation now happens in a shadow tree under `mktemp -d`:

* one symlink per repo-root entry (`ln -sT`, so a link can never be created
  *inside* an existing symlinked directory — a first draft of the overlay did
  exactly that and left eight stray symlinks in the real tree);
* every directory on the path down to the two leaves split into a real directory
  of symlinks;
* the two `Cargo.toml`s copied in as real, writable files.

`check-fixture-groups.py` derives its root as
`dirname(dirname(os.path.abspath(__file__)))` — `abspath` does NOT resolve
symlinks — and runs every subprocess with `cwd=` that root. So invoking the copy
reached through `$shadow/scripts/…` points the whole gate (manifest read, shell
group derivation, leaf manifest reads) at the overlay. No production code
changed; nothing needed a test-only hook.

Two arms were added, and both are load-bearing:

* **T0** — the UNPERTURBED overlay must produce output byte-identical to the real
  tree. Without it, "T1 reported the collision" would not distinguish a working
  gate from an overlay that quietly dropped rows; the gate's counts are
  assertions (see its docstring) and a truncated tree still reports whatever
  collision you feed it.
* **T4** — the two tracked manifests must be byte-identical to a copy taken at
  entry. This is the regression guard for this issue, and it is deliberately a
  content comparison rather than "the restore ran": experiment 4 above is a case
  where the restore ran and the file was still wrong.

That the overlay is what actually got read is self-asserting rather than
asserted: the collision exists only there, so an arm that fell back to the real
tree reports no collision and FAILS (mutant M2 below).

### Mutation testing

Each mutant applied to a throwaway overlay of the repo, so nothing under version
control was edited:

| mutant | shape | killed by |
| --- | --- | --- |
| M1 | revert phase-340 B1 — `artifacts()` drops the unhashed-LIB arm | T2 (T1 still ok) |
| M2 | wires crossed, shape still valid — arms assert against `$repo_root` while perturbing `$shadow` | T1 **and** T2 |
| M3 | `shadow_writable` leaves the symlink, so the perturbation follows it to the real file | T4 (T1/T2 still ok) |
| M4 | overlay silently drops `examples/*/listener*` leaves | T0 |

M2 is the "keeps the shape valid, crosses the wires" case: it reads as a
perfectly ordinary refactor and the gate stops testing anything. Control run
(unmutated overlay): all five arms green.

## The class

Swept `packages/testing/nros-tests/tests/` and `tests/` for a write, append,
`sed -i`, `mv` or `cp` onto a tracked path, with or without a restore trap.

**Sweep A — dynamic, and the one that actually detects the class.** It polls
`git status` *while the script runs*, so a perturbation an `EXIT` trap restores
is still caught; a before/after diff cannot see it, which is precisely why this
went unnoticed for two phases.

```sh
# dirty-probe: does <cmd> dirty a TRACKED path while it runs?
cd "$(git rev-parse --show-toplevel)"
base="$(mktemp)"; seen="$(mktemp)"
git status --porcelain > "$base"
( while :; do
    git status --porcelain | grep -vxFf "$base" | grep -E '^.?[MADRU]' >> "$seen"
    sleep 0.05
  done ) & poll=$!
"$@" >/dev/null 2>&1; rc=$?
sleep 0.2; kill $poll 2>/dev/null; wait $poll 2>/dev/null
[ -s "$seen" ] && { echo "DIRTIED (rc=$rc):"; sort -u "$seen"; } || echo "clean (rc=$rc)"
```

Verified both ways on this very script — the pre-fix version reports

```
DIRTIED TRACKED PATHS (rc=0):
     M examples/qemu-arm-baremetal/rust/action-client-rtic/Cargo.toml
     M examples/qemu-arm-baremetal/rust/action-server-rtic/Cargo.toml
```

and the post-fix version reports `clean (rc=0)`. Note the `rc=0`: the script
*passed* while dirtying the tree, so exit status is no signal here.

**Sweep B — static, for triage.** Cheap enough to run over everything at once,
but read it knowing its limit: **it would NOT have found the original.** The
offending line was `printf … >> "$f"`, whose destination is a variable, and this
arm only sees a tracked directory spelled literally. Use it to shortlist, then
run sweep A on anything it raises.

```sh
# shell + rust, write-shaped call lines naming a tracked top-level directory
python3 - <<'PY'
import re, subprocess, pathlib
root = pathlib.Path(".")
tops = {l.split("/")[0] for l in subprocess.run(
    ["git","ls-files"], capture_output=True, text=True).stdout.splitlines()}
W = re.compile(r'(fs::write|File::create|fs::copy|fs::rename|fs::remove_file|'
               r'fs::remove_dir_all|OpenOptions|create_dir_all|\bcp\b|\bmv\b|'
               r'\bln\b|\btouch\b|\brm\b|sed -i|>>?[^>])')
for f in sorted(root.glob("packages/testing/nros-tests/tests/*")) + \
         sorted(root.glob("tests/*")):
    if not f.is_file(): continue
    try: text = f.read_text()
    except (UnicodeDecodeError, OSError): continue
    for i, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if s.startswith(("#", "//", "*")) or not W.search(line): continue
        for t in sorted(tops):
            if re.search(r'["/\s(]' + re.escape(t) + r'/', line):
                print(f"{f}:{i}  {s[:140]}"); break
PY
```

**The class has exactly one member — this script.** Every other candidate
sweep B raises is a false positive, hand-classified:

* `tests/codegen-stamp-tests.sh` writes `$ROOT/packages/core/…` where
  `ROOT="$TMP/root"` (line 61) — a *synthetic* repo under `mktemp`, the same
  idea as the overlay;
* `tests/cmake-*-tests.sh`, `tests/fixture-staleness-probe-tests.sh` write only
  under `$TEST_TMPDIR` / `$work`;
* the Rust tests write into `tempfile::tempdir()` or `$project/tmp/`
  (gitignored, and what CLAUDE.md prescribes) — `emulator.rs`,
  `cmake_platform_matrix.rs`, `lane_build_covers_run.rs` are the three that
  anchor at `project_root()`, and all three join `tmp/`;
* the rest are `git grep` reads, `source` lines, or prose in comments.

**No new gate was added, deliberately.** Sweep B is what a static "no test
writes a tracked path" gate would be, and it scores 10 false positives to 1 true
positive on today's tree while missing the only real member — a gate that needs
hand-classification on every run *and* cannot see the defect it exists for is
worse than none. Sweep A is exact, but it has to RUN each script, which for most
of these means fixtures and minutes, so it belongs in a person's hands rather
than on `check-fast`. The precise guard is already in place at the one site that
has ever done this: **T4 compares bytes** and fails loudly, and mutant M3 shows
it fires. If a second member ever appears, the thing to extract is the overlay
builder (`link_entries` / `shadow_split` / `shadow_writable`), not a grep.

## Note

This is adjacent to issue 1077 but a **separate mechanism**. 1077 was
`grep -q` + `pipefail` turning a match into a failed pipeline, and it is fixed;
this is a gate writing the shared tree. They were not closed against each other.

`run-gates-parallel.sh` carries a note about reds that look like "something
transiently rewrote what that gate reads" without naming a cause. This was one
of them, and the 51 % figure above is how often it fired.
