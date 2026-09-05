---
id: 1083
title: "A service whose server lives outside the image could not be declared --
  topics have an `external:` mark and services did not, so a client-only image
  either lied or refused"
status: resolved
resolved_in: ros-launch-manifest v0.1.23 (c66c534) + play_launch db4af878
related: [phase-412, issue-0965, issue-1084]
---

# 1083 — a client-only service was unwritable

The contract could say a topic's other side is external
(`external: pub | sub | both`, Issue #51). A service could not: `ServiceDecl`
carried only `if`/`unless`/`type`/`server`/`client`, and `external_topics` is,
as its name says, topics only. `dangling-entity` then refused:

```
error: [dangling-entity] service '/add_two_ints' has 0 servers across the
       manifest tree (declared in 1 scope(s))
Error: refusing to emit a SystemModel: 2 contract error(s)
```

An ERROR with no escape, where the equivalent topic case is a WARNING with one.
Three entity families gave three different answers -- actions were checked
per-manifest but not across the tree, so a client-only ACTION already resolved.

It surfaced when phase-412 retired `ENTITIES`. Nine of the ten declarations in
this tree and the safety island moved to contract sidecars;
`examples/workspaces/cpp/src/service_client_pkg` could not, because it is half
of a deliberately-split pair (`add_client` and `add_server` are two images,
issue 0096). Its options were a false claim, an under-count that halts at boot,
or no contract at all.

## Fixed

`external: server | client | both` on `ServiceDecl` and `ActionDecl`, honoured
by play_launch's cross-scope check.

* A SEPARATE `ExternalEndpointSide`, not a reuse of `ExternalSide`. A topic's
  sides are `pub`/`sub`, a service's are `server`/`client`; one enum would
  accept `external: pub` on a service, where it names nothing. That spelling is
  now a parse error naming the three legal sides, with a test, because it is
  the mistake an author who just wrote a `topics:` block will make.
* The rule keys on WHICH side: `external: client` does not excuse a missing
  server. Negative controls in both places -- the checker, and the resolver
  end-to-end.
* `ManifestIndex::endpoint_externals` is its own map. A service and a topic can
  share an FQN, so folding it into `externals` would let them collide.

`service_client_pkg` now derives `service_client` 1 + `timer` 1 = 2 entities,
2 callback slots, which is exactly what its retired `ENTITIES` line said.

## Left open on purpose

Whether a dangling service should be an ERROR at all when the equivalent topic
is a WARNING. This makes the answer expressible; it does not change the
default. The manifest crate's own per-manifest rule also still warns about
dangling TOPICS regardless of `external:` -- play_launch drops those and
honours topic externals itself.
