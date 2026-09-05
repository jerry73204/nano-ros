---
id: 1083
title: "A service whose server lives outside the image cannot be declared -- topics
  have an `external:` mark and services do not, so a client-only image either
  lies or refuses"
status: open
type: bug
area: cli, codegen
severity: medium
related: [phase-412, issue-0965, issue-1084]
---

## What happens

The contract sidecar can say that a topic's other side lives outside this
image:

```yaml
/control/command/control_cmd:
  type: autoware_control_msgs/msg/Control
  external: pub          # the publisher is the vehicle, not us
  sub: [mrm_emergency_stop_operator/control_cmd]
```

A service has no such mark. `ServiceDecl` carries `if` / `unless` / `type` /
`server` / `client` and nothing else, and `external_topics` is, as its name
says, topics only. The `dangling-entity` rule then fires:

```
error: [dangling-entity] service '/add_two_ints' has 0 servers across the
       manifest tree (declared in 1 scope(s)) (at services./add_two_ints)
Error: refusing to emit a SystemModel: 2 contract error(s)
```

Not a warning -- the resolver refuses to emit the model at all. The asymmetry
is deliberate on the topic side and looks accidental here: a dangling topic is
a `Severity::Warning` with an `ExternalSide` escape, a dangling service is a
`Severity::Error` with none (`manifest_loader.rs`, "Dangling service: 0 servers
across the merged tree"). Actions are checked per-manifest but not across the
tree, so an action client with no server resolves fine -- three entity families,
three different answers.

## Why it matters now

phase-412 retired `nano_ros_node_register(ENTITIES ...)`: what a component
creates is stated in a contract sidecar beside the launch file, and the pools
size themselves from the resolved model. Nine of the ten declarations in this
tree and the safety island moved over cleanly.

`examples/workspaces/cpp/src/service_client_pkg` is the tenth and cannot. It is
half of a deliberately-split pair -- `add_client` and `add_server` are two
IMAGES, run as two processes (issue 0096) -- so its launch file has a client
and no server, which is exactly the shape that refuses. The options were:

* declare a server the image does not create (a false claim that also
  over-counts a queryable slot),
* declare only the timer and lose the service client (an UNDER-count, which is
  the direction that halts entity creation at boot), or
* author no contract, which is what was done.

So that one image keeps its configured `NROS_EXECUTOR_MAX_CBS` rather than a
derived one -- the state every image was in before phase-403. Nothing is wrong
in it; it simply does not benefit.

## The fix

Give `ServiceDecl` and `ActionDecl` the `external:` mark topics already have,
and have `dangling-entity` skip the marked side, in
`NEWSLabNTU/ros-launch-manifest`. It reaches nano-ros through a tag bump there,
a bump in `play_launch`'s pin, and a bump in ours -- which is why it was not
done inside phase-412.

Worth settling at the same time: whether a dangling service should be an ERROR
at all when the equivalent topic is a warning. A launch tree is routinely a
SUBSET of a running system, and a rule that cannot express that refuses valid
systems rather than catching broken ones.

## Not this

Do not "fix" it by declaring the server on the client's node. The inventory
would then size a queryable table for a server that does not exist, and the
next person reading the contract would believe it.
