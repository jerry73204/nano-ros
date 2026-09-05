# Contributing to nano-ros

nano-ros is a `no_std` ROS 2 client for embedded RTOS targets. Most of it cannot
be built on the machine you are reading this on: the Zephyr SDK alone is 9.2 GB,
the full fixture manifest has 314 rows, and one C++ Cyclone fixture leaf takes
~2m42s on a 32-thread machine. A full sweep is 40–90 minutes on a provisioned
maintainer box and is not something an outside contributor is expected to own.

So this page exists mainly to answer one question honestly: **what are you
actually on the hook for verifying, and what are you not?**

The answer is `just ci-l1`. Roughly six minutes, no SDK, no QEMU, no cross
toolchain, and it builds no fixtures. That is the whole obligation. Everything
below is context around that one line.

Related reading, in the order it becomes useful:

- [`AGENTS.md`](AGENTS.md) — the machine-readable contract for how work is done
  here (build verbs, test placement, naming). Read it before your first PR.
- [`CLAUDE.md`](CLAUDE.md) — the pitfall index. Terse, but it is the list of
  things that have actually bitten people in this tree.
- [`docs/development/multi-agent-ci-workflow.md`](docs/development/multi-agent-ci-workflow.md)
  — why the lanes are cut the way they are. Only needed if you want to argue
  with the split.

---

## 0. Find something to work on

Issues labelled [`good first issue`][gfi] and [`help wanted`][hw] are the
entry points. Both labels exist and are used.

[gfi]: https://github.com/NEWSLabNTU/nano-ros/labels/good%20first%20issue
[hw]: https://github.com/NEWSLabNTU/nano-ros/labels/help%20wanted

**Claim by commenting on the issue, and wait to be assigned.** Do not skip this.

Contributors with push access claim work by pushing a ref under `refs/claims/…`
— the same atomic origin-side reservation `just issue-new` uses for issue ids,
where the server rejects a second push to an existing ref and that rejection
*is* the lock. You cannot write that ref from a fork, so for you the
coordination mechanism is **GitHub issue assignment**, which is the visible half
of the same system. An agent or maintainer starting work is required to check
both the claim refs and the issue's assignee, so an assignment genuinely
protects you from someone landing the same fix while you are working.

The same applies to filing: `just issue-new <slug>` pushes to origin and will
fail for you. Open a GitHub issue instead and a maintainer will reserve the id.
This is not bureaucracy — id reservation by "read the highest number and add
one" has collided seven times in this repo, twice in the same afternoon.

## 1. Set up

```sh
git clone https://github.com/NEWSLabNTU/nano-ros.git
cd nano-ros
./scripts/bootstrap.sh        # installs rustup if needed, builds the nros CLI
source ./activate.sh          # or: direnv allow / source ./activate.fish
just doctor
```

**Build the CLI here, do not install a release.** `scripts/install.sh` exists
for users and puts a released `nros` in `~/.nros/bin`; in a checkout that binary
is refused. It emits *its own* generated code, and this tree's runtime is what
has to compile it — a release at the same codegen version is different, not
compatible (RFC-0090, phase-431 W1). `just doctor` FAILS when a `nros` on PATH
is not this checkout's, because `nros_cli_bin` resolves PATH first, so a shadow
is the binary every recipe here would run.

**`source ./activate.sh` is not optional, and skipping it fails in a confusing
place.** It puts the per-checkout `nros` binary on PATH and exports the SDK
environment the build scripts read; without it `zpico-sys/build.rs` panics with
`"FREERTOS_PORT not set"` from somewhere deep in a dependency graph you did not
touch. Every new shell needs it. It is the single most common cause of a
first-build failure here.

You do **not** need to run `nros setup` for any board, install a Zephyr SDK,
install QEMU, or install ROS 2 in order to contribute. Those provision the lanes
you are not being asked to run.

## 2. Do the work

Read [`AGENTS.md`](AGENTS.md) first. The rules most likely to bounce a PR:

- **One issue per PR.** See also the one-open-PR rule below.
- **Linear history.** `git pull --rebase` or `git fetch` + `git rebase`. Never
  merge unless asked.
- **Never `git add -A` or `git add .`.** Stage the paths you changed. A blanket
  add scoops up build output, gitignored generated trees, and — twice in one
  session in this repo — a submodule directory as an embedded git repo, which
  clones as an empty directory nobody can populate. git prints a warning; a
  blanket add buries it in noise.
- **Run `just format` before a broad change**, not after. It formats Rust
  (nightly rustfmt — `rustfmt.toml` uses nightly-only options, so stable
  produces different output), C/C++ and Python.
- **Tests must fail on unmet preconditions.** `assert!`, `bail!`, or
  `nros_tests::skip!`. A test that prints "environment not available" and
  returns reports PASS, which means it passes on exactly the host it was written
  to warn about. Seventeen such tests existed here before the gate that now
  forbids them.
- **Test names describe behaviour, not roadmap phases.** `zephyr_xrce_service_e2e`,
  not `phase212_n9_service`. Phase numbers go stale and tell a future reader
  nothing about what broke.
- **Don't hand-edit generated or vendored trees** — `third-party/`,
  `packages/interfaces/*/generated/`, any `generated/` directory in an example.
  If your change requires regenerating one, say so in the PR.

Keep the branch short-lived. This is not a style preference: post-submit lanes
can auto-revert a landed commit, and a branch cut hours earlier still *contains*
the reverted change, so enqueueing it silently re-introduces the defect.

## 3. Verify — this is the part people get wrong

```sh
just ci-l1
```

That is it. It is `check-cli-fresh check-fast check-build check-api-parity`
followed by `test-unit`. **Measured at 354 s (5.9 min)** on a developer machine;
expect longer on a cold `target/` or a laptop.

Why this specific verb and not `just ci`:

- **It builds no fixtures.** Only `just test` and `just test-all` depend on
  `_require-fixtures-ready`. That was always true of this tree and was simply
  never exposed as a verb — 74 of 163 test files never call a fixture resolver
  at all, and were gated behind a precondition they did not need.
- **It needs no SDK, no QEMU and no cross toolchain.** Which is precisely what
  makes it runnable on a fresh clone by someone who is not provisioned for
  embedded work.
- **It is not a formality.** In its first two runs it caught two reds already
  sitting on `main`: three clippy `-D warnings` errors and an unregenerated pool
  inventory. Neither was caught by anything else that runs on a push.

If `check-cli-fresh` fails immediately, your `nros` CLI is out of date relative
to its source closure. Run `just setup-cli` and try again. Any pull, rebase or
`git stash push`/`pop` re-stales it, because those rewrite tracked file mtimes.

### The ceiling, stated plainly

The lanes are numbered L0 through L6. `just ci-l1` covers L0 (the buildless
gates) and L1 (compile, lint, unit tests). The five above it — L2
host-executable platforms, L3 cross builds, L4 QEMU runs, L5 live ROS 2
interop, L6 real hardware — **you cannot run, and must not claim to have.**

This matters more for AI agents than for humans, and it is the reason this
section is blunt. An agent instructed to "make sure it works on Zephyr" will,
absent this paragraph, run the lane it *can* run, see green, and report that
Zephyr works. Nothing in that sequence built anything for Zephyr. The green was
real and the conclusion was fabricated.

> **"`just ci-l1` passed" is a complete and respectable answer.**
> **"CI passed" is not**, when the five lanes above it never ran.

State the verb you ran, verbatim, and stop there.

### Some of L1 will skip on your machine, and that is a thing to report

`ci-l1` degrades gracefully rather than failing on a host that lacks optional
pieces, which is what makes it runnable at all — but it means a green on your
laptop covers less than a green on a maintainer's box:

- `check-rmw-cyclonedds` prints `Cyclone DDS skip: submodule not initialised`
  and exits 0 when `third-party/dds/cyclonedds` is absent.
- `check-required-features-tests` reports `[SKIPPED:capability]` for its targets
  on a host with no zenoh router, and the runner tolerates that marker
  deliberately — a bare `cargo nextest` would count those skips as failures.

Skips are legitimate. **Silently letting a reader think they were passes is
not.** That is exactly what question 2 of the pull request template is for.

## 4. Submit

Fork, branch, push, open a PR.

```sh
git commit -s          # DCO sign-off — see below
git push -u origin fix/<short-slug>
gh pr create --fill
```

- **Sign off your commits** with `git commit -s`, which appends a
  `Signed-off-by:` trailer. This is the [Developer Certificate of
  Origin](https://developercertificate.org/) — a statement that you wrote the
  patch or otherwise have the right to submit it under the project's licence.
  It is deliberately lighter than a CLA: nothing to sign, nothing to
  administer. No bot enforces it today; a maintainer will ask if it is missing.
- **Fill in the pull request template.** All three questions, in prose. They are
  written so that a wrong answer is possible, which is the point — a checkbox
  that everyone ticks carries no information.
- **Contributions are dual-licensed MIT / Apache-2.0**, matching
  [`LICENSE-MIT`](LICENSE-MIT) and [`LICENSE-APACHE`](LICENSE-APACHE).

## 5. Review, and 6. Land

A maintainer reads the diff, approves CI for the run, and merges. Beyond
correctness they check one specific thing: **does the diff touch `build.rs`,
`justfile`, `just/**`, `.github/**`, or any script CI executes?** Those are the
shapes that can attack a build machine rather than merely being wrong, so a
change there gets read differently. If your PR legitimately needs one, say why
in the description; it will not be held against you, but an unexplained one
stalls.

Fork pull requests run on GitHub-hosted runners with a read-only token and no
secrets. No workflow in this repo uses `pull_request_target` — the trigger that
would run fork code with the base repository's token — and that is a property
maintained on purpose, not an accident.

If a lane you could not run fails on your PR, **that is a maintainer's problem
to triage, not yours.** You have no SDK, no runner access and no way to
reproduce it. Handing someone a red they cannot reproduce, on hardware they
cannot see, is how outside contribution stops. Ask for the log; do not guess.

---

## Rules that exist because contributors may be agents

This project is itself largely agent-built, so these are not suspicion — they
are the failure modes actually observed here, from both agents and humans.

- **One open PR until your first one lands.** An agent generates PRs faster than
  humans review them, and an unreviewed backlog is worse than no contribution:
  it converts a coordination cost into a queue nobody has time to drain.

- **Incidental fixes go in their own PR.** If you hit a broken gate or a stale
  lockfile while doing something unrelated, fix it as a separate one-line PR
  rather than folding it into yours. Two agents in this project independently
  fixed the *same* red while doing unrelated work, and separately both added the
  same issue-index row — neither was anyone's assigned issue, so no claim
  mechanism would ever have caught it. A tiny standalone PR lands in one cycle
  and everyone else rebases onto it instead of re-solving it. Before fixing a
  red you stumbled into, check whether an open PR already touches that file.

- **Do not re-push a change that was reverted, without a new diagnosis.** If a
  post-submit lane reverts your commit, the revert is a first-class signal: the
  defect is in a lane you cannot run, so "it worked locally" is not new
  information. Reclaim the issue, get the log, form a different explanation, and
  say what changed in your understanding. A re-push carrying the same reasoning
  gets reverted again.

- **Report what ran, not what you believe.** Three retractions in one session in
  this repo — a wrong root cause, an over-broad staleness rule, and a `const`
  assertion whose only claim was `min(a, b) <= b` — all passed every gate they
  were subject to. Green is evidence about the gates that ran. It is not
  evidence about the reasoning, and the reasoning is what review is for.
