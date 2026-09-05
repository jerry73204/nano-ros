# `nvidia-ivc`

NVIDIA Tegra IVC (Inter-VM Communication) driver. Self-contained — no
nano-ros-internal deps. Two compile-time backends:

| Feature      | Backend                                | Target      |
|--------------|----------------------------------------|-------------|
| `unix-mock`  | `UnixDatagram::pair()` (SOCK_DGRAM)    | Linux host  |
| `fsp`        | `tegra_aon_fsp.a` (`NV_SPE_FSP_DIR`)   | `armv7r-none-eabihf` (Orin SPE) |

Default build (no features) is an inert stub — every entry point fails
closed. Build with exactly one backend feature; the two are mutually
exclusive.

## Why two backends

NVIDIA's `tegra_aon_fsp.a` is closed-source and ships under SDK Manager
EULA. Anyone without an Orin DevKit account cannot build the `fsp`
backend. The `unix-mock` backend reproduces the IVC frame semantics
(fixed 64-byte SOCK_DGRAM datagrams, message boundaries preserved)
on plain Linux, which is enough for:

- CI integration tests (this crate's `tests/loopback.rs`)
- The `autoware_sentinel` Stage 1 POSIX dev path
  (FreeRTOS POSIX sentinel ↔ Linux IVC bridge daemon over a
  Unix-socket pair)

The same Rust API ([`Channel`](src/lib.rs)) and the same C ABI
(`nvidia_ivc_channel_*`) is exposed by both — callers don't branch.

## Quick test

```sh
cargo test -p nvidia-ivc --features unix-mock
```

The `loopback` test wires two channel IDs to one socketpair via
`unix_mock::register_pair`, exchanges single-frame and multi-frame
payloads, and asserts byte-perfect reassembly.

## Building for SPE

```sh
NV_SPE_FSP_DIR=$HOME/nvidia/spe-fsp \
  cargo +nightly build -p nvidia-ivc --features fsp \
  --target armv7r-none-eabihf -Zbuild-std=core
```

Drives consumed by:

- `nros-platform-orin-spe` —
  Phase 100.5; implements `PlatformIvc` by delegating here.
- `zpico-platform-shim::ivc_helpers` —
  Phase 100.4; re-exports the C ABI under the `_z_*_ivc` symbol names
  zenoh-pico's `Z_FEATURE_LINK_IVC` C code expects.
