# Environment Variables Reference

## Configuration File

All environment variables can be set in a `.env` file at the project root:

    cp .env.example .env
    # Edit .env — uncomment and adjust values as needed

- **justfile** — `.env` is auto-loaded. Missing file silently ignored.
- **direnv** — `.envrc` sources `.env` if present.
- **Manual** — `set -a; source .env; set +a` before `cargo build`.

Variables in `.env` take precedence over justfile defaults but are overridden by explicit shell exports.

## Runtime Configuration

Examples use `ExecutorConfig::from_env()` for configuration:

| Variable                               | Description                                                     | Default              |
|----------------------------------------|-----------------------------------------------------------------|----------------------|
| `ROS_DOMAIN_ID`                        | ROS 2 domain ID                                                 | `0`                  |
| `NROS_LOCATOR`                         | RMW locator (`tcp/…`, `udp/…`, `serial/…`, or `tls/…`)          | `tcp/127.0.0.1:7447` |
| `NROS_SESSION_MODE`                    | Session mode: `client` or `peer`                                | `client`             |
| `ZENOH_TLS_ROOT_CA_CERTIFICATE`        | Path to CA certificate (PEM) for TLS                            | (none)               |
| `ZENOH_TLS_ROOT_CA_CERTIFICATE_BASE64` | Base64-encoded CA certificate for TLS                           | (none)               |
| `ZENOH_TLS_VERIFY_NAME_ON_CONNECT`     | Verify server hostname in TLS (`true`/`false`)                  | (none)               |

> **Deprecated legacy names**: `ZENOH_LOCATOR` and `ZENOH_MODE` are still
> accepted (they fall back to `NROS_LOCATOR` / `NROS_SESSION_MODE`) but
> will print a one-line deprecation warning to stderr. Migrate to the
> `NROS_*` names. `ZENOH_TLS_*` names are kept because TLS is currently
> zenoh-specific.

### TLS Notes

- **POSIX**: requires `libmbedtls-dev` (`just setup base` checks it). File-path and base64 cert loading are both supported.
- **Bare-metal**: only `ZENOH_TLS_ROOT_CA_CERTIFICATE_BASE64` is supported (no filesystem). The certificate is embedded at build time.
- The `link-tls` Cargo feature must be enabled on both the example and the `nros` crate.

## Build-Time Configuration

| Variable         | Description                                                                                        | Required                            |
|------------------|----------------------------------------------------------------------------------------------------|-------------------------------------|
| `ZENOH_PICO_DIR` | CMake install prefix for pre-built zenoh-pico (use with `system-zenohpico` feature on `zpico-sys`) | Only with `system-zenohpico`        |
| `SSID`           | WiFi network name for ESP32 examples                                                               | Required for `build-examples-esp32` |
| `PASSWORD`       | WiFi password for ESP32 examples                                                                   | Required for `build-examples-esp32` |
| `NROS_EXTRA_BOARD_PATH` | Extra board-search roots (PATH-style `:` separated). Each entry is a directory shaped like `packages/boards/` — subdirs carrying `nros-board.toml` crates or `*/boards/<name>/board.cmake` bundles. Read by the `nros` CLI's board catalog AND `nano_ros_use_board()` (also settable as a CMake cache var there). Board keys stay global: a name found under two roots is an error, never shadowed. | (unset) |

### ARM FVP (`FVP_BaseR_AEMv8R`)

License-gated — nano-ros does not download the binary. Set one of
the discovery vars after accepting the Arm EULA and installing
locally. See the [ARM FVP getting-started chapter](../getting-started/arm-fvp.md)
for the end-to-end build+run walk-through.

| Variable          | Description                                                                          | Default |
|-------------------|--------------------------------------------------------------------------------------|---------|
| `ARMFVP_BIN_PATH` | Directory containing `FVP_BaseR_AEMv8R` (Zephyr-canonical, highest priority).        | (unset) |
| `ARM_FVP_DIR`     | Install root; resolver scans `models/Linux64_GCC-*/` underneath. Matches sdk-index.  | (unset) |

If neither is set, `scripts/zephyr/resolve-fvp-bin.sh` falls back
to `dirname $(command -v FVP_BaseR_AEMv8R)`. Phase 217.A —
`just zephyr run-fvp-ws-entry` / `run-fvp-board-import` skip gracefully when
the binary can't be resolved.

After extracting the Arm-provided tarball, run
`scripts/installers/arm-fvp-installer.sh` with `ARM_FVP_DIR` set
to the extraction root — it locates `FVP_BaseR_AEMv8R`, symlinks
the directory to `~/.nros/sdks/arm-fvp/current/`, and prints the
`export ARMFVP_BIN_PATH=…` line for your shell rc. Verify with
`nros doctor --board fvp-aemv8r-smp`, which cross-checks the
`[gated.arm-fvp]` entry in `nros-sdk-index.toml` and warns (never
hard-fails — license-gated) when the binary is missing.

### FreeRTOS / NuttX / ThreadX SDK Paths

These are auto-resolved by justfile recipes (defaulting to `external/` paths from `just setup freertos` / `just setup nuttx` / `just setup threadx_linux`). Override via env vars if sources are elsewhere.

| Variable              | Default                      | Description                        |
|-----------------------|------------------------------|------------------------------------|
| `FREERTOS_DIR`        | `third-party/freertos/kernel`   | FreeRTOS kernel source             |
| `FREERTOS_PORT`       | `GCC/ARM_CM3`                | FreeRTOS portable layer            |
| `LWIP_DIR`            | `third-party/freertos/lwip`              | lwIP source                        |
| `FREERTOS_CONFIG_DIR` | Board crate's `config/`      | `FreeRTOSConfig.h` + `lwipopts.h` |
| `NUTTX_DIR`           | `third-party/nuttx/nuttx`             | NuttX RTOS source                  |
| `NUTTX_APPS_DIR`      | `third-party/nuttx/nuttx-apps`        | NuttX apps source                  |
| `THREADX_DIR`         | `third-party/threadx/kernel`           | ThreadX kernel source              |
| `THREADX_CONFIG_DIR`  | Board crate's `config/`      | ThreadX config (`tx_user.h`)       |
| `NETX_DIR`            | `third-party/threadx/netxduo`           | NetX Duo source                    |
| `NETX_CONFIG_DIR`     | Board crate's `config/`      | NetX Duo config (`nx_user.h`)      |

## Buffer Tuning

All optional -- platform-appropriate defaults apply if unset. See
[Configuration](../user-guide/configuration.md) for deployment-scenario
guidance and platform guides for target-specific sizing.

### Zenoh-pico (`ZPICO_*`)

| Variable                           | Description                                            | Default          | Crate          |
|------------------------------------|--------------------------------------------------------|------------------|----------------|
| `ZPICO_FRAG_MAX_SIZE`              | Max reassembled message size after defragmentation     | `65536` / `2048` | zpico-sys      |
| `ZPICO_BATCH_UNICAST_SIZE`         | Max unicast batch size before fragmentation            | `65536` / `1024` | zpico-sys      |
| `ZPICO_BATCH_MULTICAST_SIZE`       | Max multicast batch size                               | `8192` / `1024`  | zpico-sys      |
| `ZPICO_MAX_PUBLISHERS`             | Max concurrent publishers in zenoh shim                | `8`              | zpico-sys      |
| `ZPICO_MAX_SUBSCRIBERS`            | Max concurrent subscribers in zenoh shim               | `8`              | zpico-sys      |
| `ZPICO_MAX_QUERYABLES`             | Max concurrent queryables in zenoh shim                | `8`              | zpico-sys      |
| `ZPICO_MAX_LIVELINESS`             | Max concurrent liveliness tokens in zenoh shim         | `16`             | zpico-sys      |
| `ZPICO_MAX_PENDING_GETS`          | Max concurrent in-flight service calls                 | `4`              | zpico-sys      |
| `ZPICO_SUBSCRIBER_BUFFER_SIZE`     | Per-subscriber static buffer in zenoh shim             | `1024`           | nros-rmw-zenoh |
| `ZPICO_SUBSCRIBER_LARGE_SIZE`      | `large` size-class slot, for subscriptions whose type does not fit the small block | `16384` | nros-rmw-zenoh |
| `ZPICO_MAX_LARGE_SUBSCRIBERS`      | How many `large`-class blocks this image reserves. **`0` is legal** (phase-403 W4): an image whose subscribed types all fit `ZPICO_SUBSCRIBER_BUFFER_SIZE` declares 0 and stops paying `RING_DEPTH x LARGE_SIZE` — 65,536 B at the defaults — for a class it never routes into. A hint no class can hold then fails `create_subscription` rather than dropping every sample. | `2` | nros-rmw-zenoh |
| `ZPICO_SERVICE_BUFFER_SIZE`        | Per-service-server static buffer in zenoh shim         | `1024`           | nros-rmw-zenoh |
| `ZPICO_GET_REPLY_BUF_SIZE`         | Stack buffer for service client replies                | `4096`           | zpico-sys      |
| `ZPICO_GET_POLL_INTERVAL_MS`       | Single-threaded polling interval in `zenoh_shim_get()` | `10`             | zpico-sys      |
| `NROS_SMOLTCP_MAX_SOCKETS`        | Max concurrent TCP sockets (smoltcp); brokered default since Phase 204.2. Legacy alias: `ZPICO_SMOLTCP_MAX_SOCKETS`. | `1` (brokered) | nros-smoltcp |
| `NROS_SMOLTCP_MAX_UDP_SOCKETS`    | Max concurrent UDP sockets (smoltcp); 1 by default, 4 with the `nros-smoltcp/rtps` feature (Phase 204.2). Legacy alias: `ZPICO_SMOLTCP_MAX_UDP_SOCKETS`. | `1` (brokered) | nros-smoltcp |
| `NROS_SMOLTCP_BUFFER_SIZE`        | Per-socket staging buffer (smoltcp). Legacy alias: `ZPICO_SMOLTCP_BUFFER_SIZE`.                          | `2048`           | nros-smoltcp  |
| `NROS_SMOLTCP_CONNECT_TIMEOUT_MS` | TCP connection timeout (smoltcp). Legacy alias: `ZPICO_SMOLTCP_CONNECT_TIMEOUT_MS`.                      | `30000`          | nros-smoltcp  |
| `NROS_SMOLTCP_SOCKET_TIMEOUT_MS`  | TCP read/write timeout (smoltcp). Legacy alias: `ZPICO_SMOLTCP_SOCKET_TIMEOUT_MS`.                       | `10000`          | nros-smoltcp  |

### XRCE-DDS (`NROS_XRCE_*`)

These are read by `nros-rmw-xrce-cffi`'s build script, either from the
environment or — on Zephyr — from `CONFIG_<name>` in `.config`. They set C
`#define`s of the same name **without** the `NROS_` prefix; the define is not
itself an environment variable, so exporting `XRCE_BUFFER_SIZE` has no effect
and reports no error.

| Variable                           | Description                                                                 | Default | Min | Crate         |
|------------------------------------|-----------------------------------------------------------------------------|---------|-----|---------------|
| `NROS_XRCE_BUFFER_SIZE`            | Per-slot receive buffer. See the note below — this is the receive ceiling.   | `1024`  | 64  | nros-rmw-xrce |
| `NROS_XRCE_SUBSCRIBER_RING_DEPTH`  | Queued samples per subscriber                                               | `32`    | 1   | nros-rmw-xrce |
| `NROS_XRCE_MAX_SUBSCRIBERS`        | Max concurrent subscribers. On Zephyr, DERIVED — see below.                 | `8`     | 0   | nros-rmw-xrce |
| `NROS_XRCE_MAX_SERVICE_SERVERS`    | Max concurrent service servers. On Zephyr, DERIVED — see below.             | `4`     | 0   | nros-rmw-xrce |
| `NROS_XRCE_MAX_SERVICE_CLIENTS`    | Max concurrent service clients                                              | `4`     | 0   | nros-rmw-xrce |
| `NROS_XRCE_STREAM_HISTORY`         | Reliable stream history depth; sizes the per-session output buffer          | `16`    | 4   | nros-rmw-xrce |
| `NROS_XRCE_CUSTOM_TRANSPORT_MTU`   | Custom transport MTU; stream buffers are `MTU x STREAM_HISTORY`             | `4096`  | 128 | nros-rmw-xrce |

Dropping `MAX_SUBSCRIBERS`, `MAX_SERVICE_*` and `SUBSCRIBER_RING_DEPTH` to 1,
`BUFFER_SIZE` to 256, `STREAM_HISTORY` to 4 and `CUSTOM_TRANSPORT_MTU` to 512
takes the session struct from ~390 KB to ~10-20 KB.

#### On Zephyr the three entity caps DERIVE, and the minimum is 0

`CONFIG_NROS_XRCE_MAX_SUBSCRIBERS` and `CONFIG_NROS_XRCE_MAX_SERVICE_SERVERS`
default to `-1`, which means *take the number from the entities this image
declares* (`nano_ros_node_register(... ENTITIES ...)`). A listener that declares
one subscription and no service is built with one subscriber slot and zero
service-server slots, which is what took the session struct from 427,968 bytes
to 59,088 on the zephyr cpp listener ([issue
1033](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/issues/archived/1033-xrce-subscriber-slots-budget-eight-for-one.md)).
Stating a number in Kconfig or the environment still wins over the derivation,
in both directions.

Zero is a legal, honoured value for all three caps — it is not rounded up to 1.
So creating an entity the image never declared does not truncate silently: the
create fails with `InvalidConfig` and the backend logs which cap it hit, which
knob moves it, and that the count came from the declaration.

`NROS_XRCE_SUBSCRIBER_RING_DEPTH` is deliberately NOT derivable — no inventory
knows how deep a burst a subscriber must survive — but it does now have a
Kconfig symbol. It did not before issue 1033, which made it unsettable on
Zephyr however it was spelled: a Zephyr cargo build sees only the knobs
`nros_cargo_build.cmake` forwards, and shell exports do not survive that.

#### `NROS_XRCE_BUFFER_SIZE` is the XRCE receive ceiling

A subscriber's receive ring entry is a fixed `uint8_t data[XRCE_BUFFER_SIZE]`,
so **a subscription's own buffer size cannot raise it**: asking for
`create_subscription_sized::<M, 16384>` still stops at 1024, because the 16384
buffer is the destination of a copy that never happens. A sample that does not
fit is refused on take with `MessageTooLarge`.

The static cost is `XRCE_SUBSCRIBER_RING_DEPTH x XRCE_BUFFER_SIZE` per
subscriber, which is why the default is small — raise it deliberately, and
consider lowering the ring depth at the same time.

Raising it stops helping at the transport MTU: at 4096 samples arrive
*corrupted* rather than refused, with no error and no counter. That is
[issue 0819](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/issues/0819-xrce-payloads-near-the-mtu-arrive-corrupted.md),
open at time of writing. Keep payloads comfortably below the MTU.

#### Compile-time only (not environment variables)

These are `#define`s in `packages/rmw/xrce/nros-rmw-xrce/src/internal.h` with no
env or Kconfig knob. Changing them means editing that header.

| Define                            | Description                             | Default |
|-----------------------------------|-----------------------------------------|---------|
| `XRCE_ENTITY_CREATION_TIMEOUT_MS` | Timeout for entity creation             | `1000`  |
| `XRCE_SERVICE_REPLY_TIMEOUT_MS`   | Per-attempt service reply timeout       | `50`    |
| `XRCE_SERVICE_REPLY_TOTAL_MS`     | Total service reply budget              | `5000`  |
| `XRCE_SESSION_FLUSH_TIMEOUT_MS`   | Session flush timeout                   | `100`   |
| `XRCE_SESSION_CREATION_RETRIES`   | Session creation retries                | `3`     |
| `XRCE_MAX_PENDING_REPLIES`        | In-flight service replies               | `4`     |
| `XRCE_SERVICE_REQUEST_RING_DEPTH` | Queued service requests per server      | `4`     |
| `XRCE_DEFAULT_AGENT_PORT`         | Default agent port                      | `2018`  |

### Core (`NROS_*`)

| Variable                        | Description                                                                              | Default | Crate       |
|---------------------------------|------------------------------------------------------------------------------------------|---------|-------------|
| `NROS_EXECUTOR_MAX_CBS`         | Max executor callback slots (compile-time fixed array size)                              | `4`     | nros-node   |
| `NROS_EXECUTOR_ARENA_SIZE`      | Executor arena size in bytes (compile-time fixed array size)                             | `4096`  | nros-node   |
| `NROS_SUBSCRIPTION_BUFFER_SIZE` | Default subscription/service buffer size (bytes)                                         | `1024`  | nros-node   |
| `NROS_EXECUTOR_MAX_HANDLES`     | Max handles in a C API executor                                                          | `16`    | nros-c      |
| `NROS_MAX_SUBSCRIPTIONS`        | Max subscriptions in a C API executor                                                    | `8`     | nros-c      |
| `NROS_MAX_TIMERS`               | Max timers in a C API executor                                                           | `8`     | nros-c      |
| `NROS_MAX_SERVICES`             | Max services in a C API executor                                                         | `4`     | nros-c      |
| `NROS_LET_BUFFER_SIZE`          | Buffer size for LET semantics per handle                                                 | `512`   | nros-c      |
| `NROS_MESSAGE_BUFFER_SIZE`      | Max buffer size for subscription/service data                                            | `4096`  | nros-c      |
| `NROS_MAX_CONCURRENT_GOALS`     | Max concurrent goals per action server (compile-time constant, not env-var configurable) | `4`     | nros-c      |
| `NROS_MAX_PARAMETERS`           | Max parameters in parameter server                                                       | `32`    | nros-params |
| `NROS_MAX_PARAM_NAME_LEN`       | Max parameter name length                                                                | `64`    | nros-params |
| `NROS_MAX_STRING_VALUE_LEN`     | Max string parameter value length                                                        | `256`   | nros-params |
| `NROS_MAX_ARRAY_LEN`            | Max parameter array length                                                               | `32`    | nros-params |
| `NROS_MAX_BYTE_ARRAY_LEN`       | Max byte array parameter length                                                          | `256`   | nros-params |
