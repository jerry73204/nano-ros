# #0267 live verification — the domain_bridge interop harness

**What:** a lightweight, scriptable stand-in for the Autoware safety-island demo
that surfaced issue [#0267](../issues/archived/0267-cyclone-control-msg-corrupts-through-domain-bridge.md).
Instead of running Autoware on ROS 2 Jazzy, it wires **simple ROS 2 test nodes**
into the SAME topology that triggered the bug — a publisher on one DDS domain, a
`domain_bridge` serialized-passthrough republish into another domain, and a
downstream echo that must decode the payload intact.

```
 ROS_DOMAIN_ID=2                              ROS_DOMAIN_ID=1
 ┌──────────────┐   publish   ┌───────────────┐   republish  ┌──────────────┐
 │  publisher   │────────────▶│ domain_bridge │─────────────▶│ echo (check) │
 └──────────────┘             └───────────────┘              └──────────────┘
```

Script: [`scripts/ros/domain-bridge-repro.sh`](../../scripts/ros/domain-bridge-repro.sh).
Message: any nested-struct type (default `geometry_msgs/msg/PoseStamped` — a
`Header{Time}` + a nested `Pose`, the appendable-nested-struct shape that under
XCDR2 carries a DHEADER). Pass `--type autoware_control_msgs/msg/Control` (two
nested `Time`-bearing structs — the exact #0267 shape) if that package is
installed in the image.

## Why this reproduces the mechanism

#0267 is a **representation mismatch across the bridge**: a publisher emits
XCDR1-FINAL bytes; `domain_bridge`'s `GenericSubscription`/`GenericPublisher`
re-publishes them; a downstream reading the (appendable-by-default) type under
XCDR2 consumes a phantom 4-byte DHEADER and mis-walks every nested-struct member.
The harness puts a real Jazzy `domain_bridge` in the loop with a real downstream
reader, over a nested-struct type — the minimum needed to exercise the
DHEADER-vs-no-DHEADER boundary.

## Baseline (established 2026-07-26)

`--publisher stock` (a stock `ros2 topic pub`) → **PASS**: all values survive.
This matches the wire-oracle finding
(`nros_serdes::cdr::tests::xcdr1_header_matches_live_jazzy_wire_bytes`): modern
Jazzy defaults to **XCDR1** on the wire, so a pure-Jazzy pipeline never crosses
the XCDR2 boundary and never corrupts. The corruption needs an element that reads
XCDR2/appendable — which is exactly what the original demo's downstream did.

## Verifying the nano-ros fix (`--publisher external`)

The fix under test is nano-ros's cyclone appendable descriptor (phase-303 **W1c**)
plus the XCDR2 serdes stack (**W2/W3/W4**). To verify end-to-end, nano-ros is the
domain-2 publisher and a Jazzy node is the downstream:

1. Build a nano-ros talker with the **jazzy** edition + the **cyclone** RMW, e.g.
   an example under `examples/native/{c,cpp,rust}/talker` configured with
   `[system].ros_edition = "jazzy"` and `rmw = "cyclonedds"` (RFC-0056 lowers the
   `ros-jazzy` feature → the appendable descriptor + the edition-gated writer).
   Publish `geometry_msgs/msg/PoseStamped` (or `autoware_control_msgs/msg/Control`)
   on topic `pose`, `ROS_DOMAIN_ID=2`, with the known values in the script's
   `VALUES`.
2. Run the harness in external mode (host networking so the container's DDS sees
   the host's domain 2):

   ```
   scripts/ros/domain-bridge-repro.sh --publisher external --timeout 30
   ```

3. **PASS** ⇒ nano-ros's appendable/XCDR2 output survives the `domain_bridge`
   republish + a Jazzy downstream decode — #0267 cleared. **FAIL** ⇒ the script
   reports the exact missing/corrupted value.

## Notes on the design

- Cyclone **negotiates** the wire representation at runtime, so a nano-ros
  jazzy+cyclone publisher talking to an XCDR1-only Jazzy peer still interoperates
  (both fall to XCDR1); the appendable descriptor (W1c) is what makes the XCDR2
  case correct when the peer negotiates it. The zenoh-pico/XRCE paths are not DDS
  and select the format by edition (matched endpoints agree).
- The harness is Docker-only and installs `ros-<distro>-domain-bridge` into a
  throwaway `ros:<distro>-ros-base` container; nothing is left running.
- `--publisher stock` is a permanent regression/self-test of the harness itself
  (and a standing proof that pure-Jazzy is XCDR1-clean).
