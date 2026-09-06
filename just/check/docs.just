# nano-ros gates: docs
#
# The three numbered doc series, the book, and the api-parity ledger.
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

# Phase 379 — the user API against the ROS 2 client library it mirrors. This is
# the LEDGER gate, not the parity gate: it checks that every ledger row names a
# verdict the tool knows and gives a reason, and that the correlator's own
# matching rules still hold (a defaulted parameter must not read as a
# divergence -- that bug reported all 11 of the C++ lane's first "findings").
#
# Buildless: no clang, no ROS, no nightly. The full comparison needs all three,
# so it is `just api-parity`, not a gate. `--check` joins this recipe in W2,
# once the ledger classifies the ~2000 rows the first report found; wiring a
# gate that fails on all of them today is how a gate gets switched off.
#
# phase-428 W6 — a ledger row may not cite a file that does not exist.
#
# The ledger validates existence in ONE direction: `--check` fails on an
# unledgered difference and nothing asks whether a row still has a subject. The
# sweep found rows citing `rclcpp_compat.hpp` (deleted in step A) and two probe
# files renamed by W-B5. A reason naming a deleted file is a reason nobody can
# check — the campaign's own "nothing validates a REASON", at the ledger.
#
# Cheap and mechanical: it stats cited repo-relative paths. It does NOT check
# line numbers, whether the reason is still true, or whether the row's subject
# still exists — those need the extractor, so a green here is narrower than it
# looks and the script says so.
[private]
ledger-orphan-refs:
    @python3 scripts/check-ledger-orphan-refs.py

# Check the API-parity ledger and the correlator's matching rules.
[group("check")]
api-parity-ledger:
    @python3 scripts/api-parity.py --self-test
    @python3 scripts/gen-api-comparison.py --self-test

# Every `nros…` identifier the book quotes must exist in the tree. The
# 2026-08-21 persona review's dominant failure class was prose naming renamed
# or never-existing symbols (max_callbacks_per_spin, nros_board_init_clocks,
# try_recv_safe…) — plausible spellings nothing failed on until a reader typed
# them. First run of this gate caught 8 more.
[private]
book-identifiers:
    @python3 scripts/check-book-identifiers.py

# Every relative link in the book must resolve. `check-doc-refs` answers a
# different question — "does issue 0123 exist?", by ID, and deliberately
# resolves through `archived/` — so a book link naming the pre-archival PATH
# keeps it green while the rendered page 404s. Nine links were dead this way;
# seven were plain depth errors (`../../docs/…` from `book/src/<dir>/` is
# `book/docs/…`). Buildless: reads tracked files, resolves paths, no mdbook.
[private]
book-links:
    # Two chapters are GENERATED and no longer tracked, so on a fresh clone they
    # do not exist and every link to them reads as dead. `just book` and the
    # docs workflow already generate them before mdbook; this gate has to as
    # well, and it cannot rely on `check-{pool-inventory,config-surface}`
    # running first — `check-fast` runs gates in PARALLEL, so that would be a
    # race that passes locally and fails in CI, or vice versa.
    @python3 scripts/gen-pool-inventory.py >/dev/null
    @python3 scripts/gen-config-surface.py >/dev/null
    @python3 scripts/check-book-links.py

# The book's USER track (getting-started/, user-guide/, start-here/,
# platform-guides/) teaches no `just` — it is a contributor dependency a user
# does not have (phase-368 W12). An invocation is allowed only inside an
# explicitly contributor-marked block ("**Contributors (…):**" within 8
# lines). Buildless: tracked files + regex, no mdbook.
[private]
book-no-just:
    @python3 scripts/check-book-no-just.py

# phase-429 W3 (RFC-0090 D1/D5) — `NROS_CODEGEN_VERSION` is an AUTHORED integer,
# and this tree's authored mirrors drift: `check-rmw-api-parity`'s map read green
# while 25 symbols disagreed with the vtable. Drift here is quieter and worse —
# every generated tree on disk keeps stamping the old number, so a runtime that
# can no longer run them accepts them all.
#
# So the surface generated code names is EXTRACTED (from the codegen packs AND
# the emitter's own format strings) and stamped beside the integer. The hash is
# evidence; the token stays an integer, because a reader must be able to order
# two of them. Buildless — pure source analysis, ~0.3 s.
[private]
codegen-version-surface:
    @python3 scripts/check-codegen-version-surface.py

# phase-400 — the config census, and the ratchet on it. The ladder knob count
# is the number every wave gate is stated against; it was unmeasurable until
# this existed, because the phase doc recorded a table and no method.
config-knob-census:
    @python3 scripts/check/config-knob-census.py --check

# Issue 0934 — one fact, up to nine spellings. Most are CARRIERS (how a decision
# crosses a layer), not decisions; nothing recorded which was which, so a reader
# saw nine ways to set a locator. This enumerates every Kconfig symbol and
# demands each be declared public, carrier or dead. RATCHETED rather than
# absolute: 55 of 74 are unclassified today, and a gate that demanded zero would
# be switched off on day one.
[private]
config-surface:
    # WRITES the page rather than diffing a committed copy: it is generated
    # and no longer tracked, so `--check` would report "stale" on every fresh
    # clone. The assertions that matter (the knob scan and its self-test) run
    # either way; the artifact the book needs falls out.
    @python3 scripts/gen-config-surface.py

# issue 0936 — a `just <recipe>` printed in a DOCUMENT must name a recipe that
# exists. `check-just-recipe-refs` covers justfiles and workflows and hands
# documents to `check-doc-refs`, which checks links and paths, not recipe names —
# so a document was the one place an invocation could rot unwatched, while being
# exactly where a reader copies one from.
#
# A RATCHET over `.config/doc-recipe-refs-baseline.txt` (102 known dead
# references), because fixing them is a research task per line and guessing
# would replace a dead command with a wrong one. Buildless: markdown + the
# justfile grammar.
[private]
doc-recipe-refs:
    @python3 scripts/check-doc-recipe-refs.py

# Every `docs/{design,issues}/NNNN-*.md` path written anywhere — prose, issue
# frontmatter, or a cmake error message — must resolve. Renumbering on an id
# collision is what breaks these.
doc-refs:
    @bash scripts/check-doc-refs.sh

# Issue-id integrity: ids unique across docs/issues/ + archived/, and each
# file's `id:` frontmatter matching its filename. Parallel sessions kept
# picking the same "next free" id — six were duplicated across thirteen files
# before this gate existed, which made every `See 0051-*` pointer ambiguous.
[private]
issue-ids:
    @scripts/ci/issue-ids-check.sh

# The "Open issues" list in docs/issues/README.md must name EXACTLY the files in
# docs/issues/. The rule is already written in that file's Conventions #3 —
# nothing enforced it, and it drifted twice in two consecutive pulls (#0465,
# #0474): each was archived with a `git mv` while its README row stayed in the
# OPEN spelling, so the index advertised an open issue whose file was gone.
[private]
issue-index:
    @python3 scripts/gen-issue-index.py --self-test
    # `docs/issues/open.md` is a gitignored ARTIFACT since 2026-09-03, so there
    # is no committed copy to compare against and `--check` has nothing to be
    # stale against. REGENERATE, then verify — which is the stronger order
    # anyway: `check-issue-index.sh` derives the file list INDEPENDENTLY and
    # compares it to the rows, so the two computations still have to agree. What
    # is gone is only the drift class that existed because the list was
    # committed and hand-editable.
    @python3 scripts/gen-issue-index.py
    @bash scripts/check-issue-index.sh

# The api-parity ledger's schema has ONE home (issue 1095). It used to be copied
# into all 17 shards, so a schema change touched 17 files and concurrent PRs
# conflicted in up to 17 paths without disagreeing about anything — the class
# issues 0883/0884 record, one surface over.
[group("checks")]
ledger-doc-single-home:
    @python3 scripts/check-ledger-doc-single-home.py

# issue 1085 — a markdown LINK must resolve EXACTLY, where `doc-refs` above is
# deliberately loose.
#
# `doc-refs` accepts a reference whose basename is at the named path OR in that
# series' `archived/` dir, because archiving MOVES a resolved issue and
# retargeting every prose mention would be churn. Right for prose and for a
# cmake error string; wrong for a link, which a reader CLICKS. Swept when this
# landed: 146 dead links in live documents, 123 of them a live doc still
# pointing at where an issue, phase or RFC used to be — with `doc-refs` green
# the whole time.
#
# `book/` has its own gate; `**/archived/**` is a historical record and is
# COUNTED rather than fixed, as a ratchet that may only fall. Code spans are
# stripped first: `T operator[](i)` is C++, not a link.
[private]
markdown-links:
    @python3 scripts/check-markdown-links.py

# issue 1103 — a phase SECTION whose declared `**Files:**` all exist while its
# boxes stay open. `check-roadmap-claims` beside this compares a document against
# ITSELF (status text vs box counts, an `**Owns:**` link vs an issue's
# frontmatter); nothing asked whether the work described had ARRIVED. phase-215
# read 20 of 38 boxes while 27 were done — a P1 phase advertising seven finished
# items as available.
#
# SECTION level, not box level, and that was measured rather than assumed: the
# per-box rule ("an unticked box naming a path that exists") catches 2 of those
# 7 and fires on 2 false positives tree-wide, because most boxes name a variable,
# a function, a recipe, or prose. The section rule catches 3 of the 4 relevant
# sections with zero findings on main. Ratcheted; `--write-baseline` to accept.
[private]
roadmap-boxes:
    @python3 scripts/check-roadmap-boxes.py

# phase-419 W2 — an active phase must not contradict its own body. Sibling of
# `roadmap-status`, which asserts a status EXISTS and deliberately stops there
# ("staleness of the CONTENT is a human call"). That is right for supersession
# and wrong for a header that says "nothing landed" over ticked work items —
# the document contradicts ITSELF, and no judgment is involved. Ratcheted;
# the judgment half is phase-419 W3, a report and never a gate.
[private]
roadmap-claims:
    @python3 scripts/check-roadmap-claims.py

# Every ACTIVE roadmap phase carries a findable status line; a finished one
# belongs in `docs/roadmap/archived/`. A one-off pass does not hold — four
# phases lost theirs within days of `ecc195ed6` doing exactly that.
[private]
roadmap-status:
    @bash scripts/check-roadmap-status.sh

# Issue 1110 — the PUBLISHED API docs must still build, on a lane a PR runs.
#
# rustdoc's broken-link lint is deny-level here, and NOTHING on a merge-gating
# path ran rustdoc. `3941b569a` deleted `ClientTrait::is_server_ready` while the
# paragraph beside it still linked to it, so `just book` and every docs deploy
# went red and stayed red for three days — `docs.yml`'s path filter includes
# that crate, so the deploy was failing, not skipping. A SECOND break sat behind
# it (`nros-rmw-zenoh` naming a `record_alloc_ceilings` that never existed),
# invisible because rustdoc stops at the first crate that fails.
#
# Same class as issues 0319 and 0896: a correct check on a path nothing
# traverses. The fix is the path, not the check.
#
# The crate + feature set is `scripts/build/rustdoc-set.sh`, which `just book`
# reads too — one spelling, because a gate documenting a DIFFERENT set from the
# one that deploys is a gate that can be green while the deploy is red.
#
# Not on the fast line: it compiles. That is stated in
# `.config/gate-lane-exempt.txt` and not only here, which is the difference
# between an intent and a mechanism — fast-lane membership is DERIVED (issue
# 1072: a recipe is a fast gate unless it is exempt), so a comment saying "not
# on the fast line" did not make it so. It WAS on the fast line, and on `push`
# to main — where gate.yml does not provision `-sys` sources — `cargo doc`
# reached `zpico-sys`'s build script and died on the absent zenoh-pico:
#
#   zenoh-pico source not provisioned at ".../zpico-sys/zenoh-pico"
#   error: recipe `rustdoc-links` failed with exit code 101
#
# It runs from its OWN pull-request step in gate.yml, which sits after the
# provisioning step — `check-lane-contracts`'s rule is that a lane may only
# resolve artifacts the job itself builds, and that job does.
[group("ci")]
rustdoc-links:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/rustdoc-set.sh
    mapfile -t pkgs < <(nros_rustdoc_package_args)
    cargo doc --no-deps --quiet \
        --features "$NROS_RUSTDOC_FEATURES" \
        "${pkgs[@]}"
    echo "rustdoc-links OK — the ${#NROS_RUSTDOC_CRATES[@]} published crates document cleanly."

# Issue 0937's shape — an issue in `archived/` must SAY it is resolved.
#
# `4f1305788` archived 0937 with an `R100` rename: a 100% similarity move, zero
# content change. The correction its own commit message described had been
# clobbered by a `git checkout origin/main -- <file>`, so the file sat in
# `archived/` for a day reading `status: open` and blaming a cause that had
# been disproved. `c7f2b09a0` restored it, +81/-72.
#
# The rule was CHOSEN by measurement, not taste: "an archive move must not
# delete text" fires on 127 of 668 archive renames (19%), most of them
# legitimate; "the archived content equals an older committed version" fires on
# zero, including 0937, and costs 27 s. A frontmatter that still reads `open`
# is what an R100 rename necessarily leaves behind, and it is one cheap read.
[private]
archived-issue-status:
    @python3 scripts/check-archived-issue-status.py

# phase-359 W0 — the `std` census ratchet. Buildless (reads sources), so it
# belongs in the fast tier next to the other convention gates.
#
# The campaign to drop `std` from the core crates spans 181 cfg sites and 425
# `std::` paths over nine crates. This freezes those counts per crate and FAILS
# when one goes up — including a crate entering scope that the baseline has
# never seen. Counts going DOWN also "fail", loudly and on purpose: the fix is
# to lower the baseline in the same commit, so progress lands in the diff
# instead of a claim in a message.
[group("ci")]
std-census:
    @python3 scripts/check-std-census.py
