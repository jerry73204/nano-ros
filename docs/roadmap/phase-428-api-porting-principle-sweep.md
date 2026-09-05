# Phase 428 — sweep the C, C++ and Rust APIs against the porting principle

**Status (2026-09-05). Planned.** Audits all three user APIs against RFC-0089's
governing principle. Not a rename pass — a conformance review that produces
findings, each of which becomes a work item somewhere else.

## The question being asked

For every public item in `nros-c`, `nros-cpp` and the `nros` Rust facade:

1. **Does an upstream counterpart exist** in rclc / rclcpp / rclrs? (The
   recorded surfaces already answer this — `docs/reference/api-surface/`.)
2. **If yes, do we use its name?** If not, why not — a constraint (clause 1), or
   a preference? RFC-0036 forbids recording a preference as a divergence, so a
   preference is a finding.
3. **If we use its name, is the difference MECHANICAL?** Does the compiler point
   at every difference, and is the fix local? A changed signature the compiler
   does not force — a widened return type at a discarding call site, behaviour
   behind an identical signature — is a finding.
4. **If no counterpart exists, is the invention necessary,** and does it carry a
   ledger row plus collision-gate coverage?

## Why it is worth doing as a sweep

The three known instances of question 3 were each found by accident, not by
looking: `rclcpp::init` widening `void` to `Result` at a discarding call site;
`ParametersQoS()` returning `QoS(10)` where upstream is `KEEP_LAST, 1000`; a
`NodeOptions` setter that stores its argument and is never read. All three
correlate `same` in the parity report, because correlation compares names and
shapes and neither differs. **The instrument cannot see this class**, which is
what makes it a sweep rather than a gate.

## Work items

* **W1 [tooling] — enumerate the candidates.** Every row where our name matches
  upstream's AND the shapes correlate `same`. That set is where a silent
  behavioural difference can hide; it is also the set the parity report is
  least likely to flag.
* **W2 [c] / W3 [cpp] / W4 [rust]** — one pass per language over its
  candidates, answering the four questions. Findings, not fixes.
* **W5 [ledger]** — every finding gets a row or amends one, with a disposition.
* **W6 [gate]** — where a finding is mechanically checkable, add the check
  rather than the note. The `[[nodiscard]]`-on-`Result` item from phase-427 is
  the model: a class of "the compiler does not force the edit" becomes a lint.

## Scope notes

* The C API's shape follows rcl/rclc, the C++ follows rclcpp, Rust follows
  rclrs — settled, each language takes its own upstream.
* This sweep does NOT rename. A finding that says "we should use upstream's
  name here" becomes a work item in the phase that owns that surface.
* Expect the Rust facade to have the most findings and the fewest of them
  actionable: rclrs is the youngest of the three upstreams and moves fastest, so
  "no counterpart" there often means "not yet" rather than "never".
