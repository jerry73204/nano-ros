# Zephyr C++ talker — typed component (RFC-0043)

The node is a stateful **component** rather than a hand-written imperative
`main.cpp`: `Talker::configure(node)` binds a `std_msgs/String` publisher on
`/chatter` plus a 500 ms `bind_timer` member callback that publishes
`Hello World: N` (the official ROS 2 demo chatter). The **Zephyr typed
carrier** generates the entry:

- `nano_ros_add_node(TYPED … DEPLOY zephyr)` (with
  `NANO_ROS_PLATFORM=zephyr`) `configure_file`s
  the shared entry pack (`nros codegen entry-node`) — a plain `int main(void)`
  that constructs the component, calls `configure(node)`, and runs
  `::nros::board::ZephyrBoard::run_components(&setup)`.
- The carrier branch composes the generated entry + the component lib into the
  global `find_package(Zephyr)` `app` target
  (`target_sources`/`target_link_libraries`) — no second executable.

## Build

From the repo root (Zephyr workspace provisioned via `just setup zephyr`):

```sh
just zephyr build-one cpp/talker zenoh          # native_sim/native/64 default
just zephyr build-one cpp/talker xrce
just zephyr build-one cpp/talker cyclonedds
```

The RMW is selected by conf overlay: `CONF_FILE = prj.conf;prj-<rmw>.conf`
(this dir carries `prj-zenoh.conf`, `prj-xrce.conf`, `prj-cyclonedds.conf`).

## History

This dir absorbed the former `examples/zephyr/cpp/talker-typed` (phase-240.8
typed-carrier draft): both were the same TYPED component modulo namespace
after phase-244.C2 converted this example to the component shape, so the
duplicate was removed in phase-277 W7.
