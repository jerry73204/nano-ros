# ARM FVP (`FVP_BaseR_AEMv8R`)

Run nano-ros on Arm's `Base_RevC AEMv8-R` Fast Models — the
Cortex-A SMP profile under Zephyr 3.7 (board id
`fvp_baser_aemv8r/fvp_aemv8r_aarch64/smp`). The FVP is the
canonical local proxy for the safety-island reference platforms
that follow the same `hwv2` Zephyr shape; pair this chapter with
the [Zephyr (west module)](./integration-zephyr.md) starter for
the build half.

> **Status.** The run half — invoking `FVP_BaseR_AEMv8R` and piping
> UART 0–3 to stdout — is landed.
>
> **The two standalone talker lanes were retired** (issue 0537):
> their runners had been deleted, so they built images nothing booted. What
> remains is the part with consumers — the board-crate import surface and the
> workspace Entry. FVP support is intended to grow back from those. See
> [`docs/roadmap/phase-217-arm-fvp-local-runtime.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/phase-217-arm-fvp-local-runtime.md).

## When to use

- You need to exercise a Cortex-A SMP Zephyr image on a developer
  laptop without dedicated silicon.
- You're bringing up a new `hwv2` safety-island target — the FVP
  is the closest in-tree reference for board.cmake + SMP boot +
  Cyclone DDS shape.
- You're validating stock-Cyclone-DDS RMW interop locally before
  promoting to a hardware bench.

The FVP is **not** a replacement for QEMU on Cortex-M (`mps2-an385`,
`mps3-an547`); those targets are covered by [FreeRTOS (QEMU)](./freertos.md)
and the Zephyr starter. The FVP is also **not** the same surface
as Corellium's AVH cloud FVP — AVH packages firmware as `.coreimg`
and provisions via a remote API; local FVP loads a raw ELF via
`-a cluster0.cpu*=<elf>`. AVH is out of scope for this chapter.

## Prereqs

- A working Zephyr 3.7 workspace — run `nros setup zephyr` once if
  you haven't (see the [Zephyr starter](./integration-zephyr.md)).
- The Arm `Base_RevC AEMv8R` Fast Models binary. **nano-ros fetches it:**

  ```sh
  nros setup --tool arm-fvp
  ```

  `nros setup board fvp-aemv8r-smp` does it as one of its steps, so most
  people never run the line above directly. It is a pinned Arm CDN permalink
  with a checked SHA256 (`[tool.arm-fvp]` in `nros-sdk-index.toml`), installed
  to `~/.nros/sdk/arm-fvp/<version>`.

  **x86_64 Linux only** — Arm publishes no other host build of this model. On
  another architecture you can still build the image; you cannot run it here.

  > Until 2026-09-06 this page said the model was license-gated and that
  > fetching it was your responsibility. It is not: the permalink answers 200
  > with no authentication, which is how `autoware-safety-island` had been
  > downloading it in CI the whole time. The mistaken policy cost every reader
  > of this page a manual install.

After installing, export one of the discovery env vars:

```bash
# Preferred — Zephyr's canonical name; takes highest priority.
export ARMFVP_BIN_PATH=/opt/Arm/FastModels/Base_RevC_AEMv8R/models/Linux64_GCC-9.3

# Alternative — directory layout from the gated installer; the
# resolver scans `models/Linux64_GCC-*/` underneath it.
export ARM_FVP_DIR=/opt/Arm/FastModels/Base_RevC_AEMv8R
```

If neither is set, `FVP_BaseR_AEMv8R` is discovered via `PATH` as
a last-ditch fallback.

### Installer surface

After extracting the Arm FVP tarball, run the discovery script:

```bash
ARM_FVP_DIR=/path/to/extracted/fvp \
    scripts/installers/arm-fvp-installer.sh
```

The installer locates `FVP_BaseR_AEMv8R` under `$ARM_FVP_DIR`,
symlinks the containing directory to
`~/.nros/sdks/arm-fvp/current/` (atomic via `ln -sfn`), and
prints the `export ARMFVP_BIN_PATH=…` line for your shell rc.
Run `scripts/installers/arm-fvp-installer.sh --print-env` later
to re-emit the export. It never downloads anything — gated-tool
policy.

### Doctor check

nano-ros checks that the model resolves via `ARMFVP_BIN_PATH`, `ARM_FVP_DIR`,
`PATH`, or the SDK store (`nros sdk-path arm-fvp`), and warns — never
hard-fails — when it can't. A warning rather than an error because a
contributor who is not working on the FVP should not be blocked by its absence;
`nros setup --tool arm-fvp` is the fix when they are.

> **Contributors:** the in-tree doctor cross-check and FVP run recipes are in
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#arm-fvp).

## Build

> **Contributors:** the in-tree FVP build lanes (and the `zephyr.elf`
> paths they produce) are in
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#arm-fvp).

> Two standalone talker lanes (`build-fvp-aemv8r-cyclonedds{,-rust}`, over
> `examples/zephyr/{cpp,rust}/talker-aemv8r`) used to live here. Their
> tests were deleted and the lanes later retired, so the chapter no
> longer documents a lane whose output nobody consumes.

## Run

Once the build artifacts and `ARM_FVP_DIR` / `ARMFVP_BIN_PATH` are in
place:

> **Contributors:** the in-tree FVP run lanes (and what they do under
> the hood) are in
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#arm-fvp).

## Expected output

The Zephyr 3.7 boot banner appears on UART0 first, followed by
the talker. The exact line counts depend on `CONFIG_BOOT_BANNER`
and your locator config, but the markers to look for are:

```text
*** Booting Zephyr OS build v3.7.0 ***
[00:00:00.xxx,000] <inf> nros: session up (domain 0)
Publishing: 'Hello World: 1'
Publishing: 'Hello World: 2'
...
```

For the cpp/cyclonedds recipe, the same banner is followed by
Cyclone DDS reader-match logs and `std_msgs/String` publish lines.
Verify ROS 2 interop by running a sibling listener in another
terminal:

```bash
# stock ROS 2 — reads the FVP's Cyclone DDS publisher
ros2 topic echo /chatter std_msgs/msg/String
```

The same `std_msgs/String` payload (`Hello World: N`) + byte-equal
CDR framing must appear on both sides — that's the stock-RMW
interop contract (byte-compatible with `rmw_cyclonedds_cpp`).

## Cross-references

- [`docs/roadmap/phase-217-arm-fvp-local-runtime.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/phase-217-arm-fvp-local-runtime.md)
  — the roadmap doc for the FVP runtime slice (contributor detail
  on what has landed vs what is ongoing).
- [`docs/roadmap/archived/phase-117-cyclonedds-rmw.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/archived/phase-117-cyclonedds-rmw.md)
  — the Zephyr FVP build smoke + Cyclone DDS port the runtime
  exercises.
- [Environment Variables — `ARM_FVP_DIR` / `ARMFVP_BIN_PATH`](../reference/environment-variables.md)
  — the discovery contract.
- [Zephyr (west module)](./integration-zephyr.md) — the parent
  Zephyr starter; the FVP is a board-target slice of it.
- [`examples/workspaces/realtime-cpp/src/fvp_entry/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/workspaces/realtime-cpp/src/fvp_entry)
  — the workspace Entry this chapter builds and boots.
