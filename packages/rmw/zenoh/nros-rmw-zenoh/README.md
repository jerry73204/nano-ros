# nros-rmw-zenoh

Zenoh-pico RMW backend for nano-ros. **Default backend** — used by every
example unless explicitly set otherwise.

## Role

Implements the [`nros-rmw`](../../../core/nros-rmw) trait surface
(`Rmw`, `Session`, `Publisher`, `Subscriber`, `ServiceServerTrait`,
`ServiceClientTrait`, plus the lending traits) over the C zenoh-pico
client. Acts as the **canonical reference port**: every other RMW
backend follows the same trait-implementation pattern, so when writing a
new backend, read this crate first.

## Source layout

| File | Role |
|------|------|
| `src/lib.rs` | Public re-exports + `ZenohRmw` factory. |
| `src/zpico.rs` | `Session` / `Publisher` / `Subscriber` / service / lending impls calling into zenoh-pico. |
| `src/keyexpr.rs` | ROS-name → zenoh key expression mapping (rmw_zenoh-compatible). |
| `src/config.rs` | Locator + mode + scouting config helpers. |
| `src/shim/` | C-side interop helpers (write filter, etc.). |

## When to use

- **Default for every supported platform.** Single-stack peer-to-peer or
  client-router topologies, any combination of TCP / UDP / Serial /
  smoltcp transports.
- ROS 2 interoperability (with `rmw_zenoh_cpp`) — see
  [docs/reference/rmw_zenoh_interop.md](../../../../docs/reference/rmw_zenoh_interop.md).

Pick a different backend (XRCE-DDS / dust-DDS / uORB) only when the
deployment specifically requires it.

## Lending (zero-copy)

`PublishLoan` is implemented via `z_bytes_from_static_buf` so the
publisher can hand the user a slot in zenoh-pico's outbound buffer
without an extra copy. See the
Zero-copy raw API design.

## See also

- [Custom RMW Backend porting guide](../../../../book/src/porting/custom-rmw.md)
- [`nros-rmw` trait surface](../../../core/nros-rmw)
- Source on GitHub: <https://github.com/NEWSLabNTU/nano-ros/tree/main/packages/rmw/zenoh/nros-rmw-zenoh>
