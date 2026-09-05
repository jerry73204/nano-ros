# ws-realtime-c — two-node, two-tier realtime demo (C)

## What it shows

The C projection of the scheduling-tiers differentiator (RFC-0015 §4.2):
`ctrl_node` (`ctrl_pkg::Ctrl`) publishes `/ctrl` every 10 ms on the `high`
tier (posix prio 80); `telem_node` (`telem_pkg::Telem`) publishes `/telem`
every 100 ms on the `low` tier (prio 10). Tier definitions and the
group → tier bindings (`[[component]].group_tiers`) live entirely in
`src/demo_bringup/system.toml` — the node code carries no priorities. The
generated entry emits `nros_cpp_create_sched_context` +
`nros_cpp_bind_group_sched`.

## Run

```sh
source ./activate.sh
nros build native
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &
./build/src/native_entry/native_entry
```

## Expected output

```
[ctrl] tick=N         # ~10 lines per one
[telem] tick=N
```

(e2e asserts the ~10× cadence ratio.)

## Copy-out notes

Standard workspace copy-out. Fixture id `workspace-c-native-realtime`.
C++ siblings: `ws-realtime-cpp` and its variants.
