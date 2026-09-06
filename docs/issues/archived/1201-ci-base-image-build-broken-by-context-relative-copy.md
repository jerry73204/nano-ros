---
id: 1201
title: "The ci-base image build failed on every push since 2026-09-04 — the Dockerfile COPYs repo paths from a context that is its own directory — and the floating `humble` tag hid it for three days"
status: resolved
type: bug
area: ci
severity: high
related: [0833, 1040, 1059, 1163, 0996]
found: 2026-09-07
---

# A floating tag that stays put on a failed build looks exactly like one nobody asked to move

## Measured

`gh run list --workflow images.yml`, 2026-09-07:

```
failure   2026-09-06T22:19Z   47s   fix(#1146) …            <- the push that added clang (#557)
failure   2026-09-04T11:31Z   40s   fix(ci): host-tests …   <- the push that added the COPYs
success   2026-09-03T19:06Z   30m   (zephyr image only; ci-base SKIPPED by the paths filter)
success   2026-09-03T04:30Z   16m   (the last ci-base build that produced an image)
```

Both failures, one minute in:

```
COPY scripts/lib/rust-targets.sh /opt/nros-ssot/scripts/lib/rust-targets.sh
COPY config/rust-targets.txt     /opt/nros-ssot/config/rust-targets.txt
ERROR: failed to calculate checksum of ref …: "/config/rust-targets.txt": not found
```

`436035894` (2026-09-04, issue 0833's follow-up) made the Dockerfile read the
Rust target list from the tree's SSoT instead of a third hand-written copy —
correct — but `images.yml` still handed buildx `context: ci/docker/ci-base`,
and neither path exists under that directory. The Dockerfile was written
against the repo root; the workflow was written against the Dockerfile's
directory; nothing checked they agreed.

And the registry says what the consumers actually ran on:

```
$ gh api /orgs/NEWSLabNTU/packages/container/nano-ros-ci/versions
2026-08-29T06:35:51Z  ["humble-04ec76585e7b", "humble"]
```

`ghcr.io/newslabntu/nano-ros-ci:humble` — the `container:` of every `check`,
`host-tests` and post-submit job — was 2026-08-29 content. It never gained the
targets `436035894` baked (so `check-tier-preconditions` kept failing the way
0833 described, on an image whose Dockerfile said it was fixed), and it never
gained the `clang` that `0c73d061b` (#557, 2026-09-06) added so that
`check-api-parity` could run in the container at all. Issue 1163 item 2 — the
merge-gating `api-parity` step — was waiting on that image, and the image was
never going to arrive.

## Why nobody noticed

Nothing consumes an `images.yml` failure. It has no `coverage` job, it is not
a required check, and its output is a floating tag whose whole property is that
consumers do not have to know when it moved. So a failed build is a tag that
does not move, which is the same observable as a tag that had no reason to —
the phase-253 header's own complaint about the floating alias, seen from the
other side. `check-default-gates-run-somewhere` and `check-lane-contracts`
audit which gates RUN; neither can see that the image a lane runs IN is stale
against its Dockerfile.

The zephyr image already paid for this class once (its `r3`/`r4` note in
`images.yml`: an image build 404'd and the registry kept serving one-SDK
content under a tag the tree said held two).

## Fix

- `images.yml` builds ci-base with `context: .` and
  `file: ci/docker/ci-base/Dockerfile`, so the repo-relative COPYs resolve.
  `ci/docker/ci-base/Dockerfile.dockerignore` (buildx reads
  `<Dockerfile>.dockerignore` in preference to a root one) keeps the context
  to the files the Dockerfile names — measured 16 kB transferred.
- The two COPY'd files join the `push` paths filter, the `changes` filter and
  the CONTENT tag's hash: they are image inputs exactly as the Dockerfile is,
  and a `rust-targets.txt` edit with a byte-identical Dockerfile used to
  change the published content under an unchanged content tag.
- `workflow_dispatch` takes an `image` input (`ci-base` default / `zephyr` /
  `both`); a dispatch used to rebuild both, and the zephyr one is 30 minutes.

Verified by dispatching the fixed workflow from the fix branch (the Dockerfile
is main's, byte for byte) and probing the published image; the evidence is in
the resolving PR.

## Not fixed here

A build-failure that leaves the tag in place is still silent. The cheap
visibility would be a step in the CONSUMING jobs that compares the running
image's content tag against the one the tree's inputs hash to — "this job is
running on an image the tree does not describe" — which is the `cli-fresh`
question asked of the container. Left open as a follow-up rather than
invented here.
