# Logging

nano-ros ships a ROS 2 style leveled logging facade
([`nros-log`](https://github.com/NEWSLabNTU/nano-ros/tree/main/packages/core/nros-log))
with the same `Logger` + severity surface as `rclcpp::Logger` /
`rcutils_logging`. Records flow through a single per-platform sink
(`PlatformSink`) that delegates to whichever
`nros-platform-<rtos>` is linked into the binary.

## Severity ladder

REP-2012 style, matching `rcutils_log_severity_t`:

| Severity | u8 | When to use |
|----------|----|-------------|
| `Trace`  | 0  | Per-instruction granularity; off by default. |
| `Debug`  | 1  | Diagnostic information useful while developing. |
| `Info`   | 2  | Normal operation events worth surfacing once. |
| `Warn`   | 3  | Unexpected but recoverable conditions. |
| `Error`  | 4  | Errors the caller should surface; system continues. |
| `Fatal`  | 5  | Unrecoverable; system is about to abort. |

The numeric representation is stable and used by the
`nros_platform_log_write` ABI.

## Quick start

### Rust

```rust
use nros_log::{Logger, Severity};
use nros_log::{log_info, log_warn};

static LOGGER: Logger = Logger::new("my_node");

fn main() {
    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_log::sinks::default());

    log_info!(&LOGGER, "started; domain = {}", 42);
    log_warn!(&LOGGER, "queue depth {} exceeds soft limit", 5);
}
```

Inside a node, prefer the `Node::logger()` accessor — it resolves
to the same intern'd entry when the name matches:

```rust
let mut node = executor.create_node("my_node")?;
log_info!(node.logger(), "subscribed to {}", topic);
```

### C

```c
#include <nros/init.h>
#include <nros/node.h>
#include <nros/log.h>

int main(void) {
    nros_support_t support = nros_support_get_zero_initialized();
    nros_support_init(&support, "tcp/127.0.0.1:7447", 0);

    nros_node_t node = rcl_get_zero_initialized_node();
    rclc_node_init_default(&node, "my_node", "/", &support);

    nros_logger_t logger = nros_node_get_logger(&node);
    NROS_LOG_INFO(logger, "started; domain=%u", 42);
    NROS_LOG_WARN(logger, "queue depth %u exceeds soft limit", 5);

    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return 0;
}
```

### C++

```cpp
#include <nros/nros.hpp>
#include <nros/log.hpp>

int main() {
    nros::init("tcp/127.0.0.1:7447", 0);

    nros::Node node;
    nros::create_node(node, "my_node");

    auto logger = node.get_logger();
    NROS_LOG_INFO(logger, "started; domain=%u", 42);
    NROS_LOG_WARN(logger, "queue depth %u exceeds soft limit", 5);

    nros::shutdown();
    return 0;
}
```

The first `NROS_LOG_*` emit auto-installs `PlatformSink` so C/C++
call sites work without an explicit init step.

## Per-platform delivery

| Target | Backend |
|--------|---------|
| POSIX | `fprintf(stderr, "[<LEVEL>] <name>: <msg>\n")` |
| Zephyr | `LOG_INF` / `LOG_WRN` / etc. (or `printk` if `CONFIG_LOG=n`); module `nros` |
| ESP-IDF | `esp_log_write` with logger-name = ESP TAG |
| NuttX | `syslog(priority, "%s", buf)` |
| FreeRTOS | board-registered UART writer fn-ptr |
| ThreadX | board-registered UART writer fn-ptr |
| Bare-metal (mps2-an385) | QEMU semihosting via `cortex_m_semihosting::hio::hstderr` |
| Bare-metal (stm32f4) | `defmt::info!("{=str}", msg)` |
| Bare-metal (esp32 / esp32-qemu) | board-registered fn-ptr (esp-println / RTT / serial-jtag) |

Boards on platforms with no native logger (FreeRTOS / ThreadX /
bare-metal Rust ESP32) supply their writer fn-ptr at `run()` time:

```rust
// from nros-board-mps2-an385-freertos/src/lib.rs
fn register_log_writer() {
    unsafe extern "C" fn writer(severity: u8, name_ptr: *const u8, name_len: usize,
                                 msg_ptr: *const u8, msg_len: usize) { … }
    unsafe { nros_platform_register_log_writer(Some(writer), None); }
}
```

## Filtering

### Compile-time ceiling

Cargo features on `nros-log` (pick at most one; default
`max-level-trace`):

| Feature | Macros above ceiling that emit |
|---------|--------------------------------|
| `max-level-trace` | trace / debug / info / warn / error / fatal |
| `max-level-debug` | debug / info / warn / error / fatal |
| `max-level-info` | info / warn / error / fatal |
| `max-level-warn` | warn / error / fatal |
| `max-level-error` | error / fatal |
| `max-level-off` | (none) |

Below-ceiling macros expand to `()`; the format call is
dead-code-eliminated.

### Runtime per-logger threshold

```rust
let logger = nros_log::get_logger("my_node");
logger.set_level(nros_log::Severity::Warn);   // silences Trace/Debug/Info
```

Default = `Severity::Info` for any logger constructed via
`Logger::new(name)`.

## Buffer size

`nros-log` formats each record into a stack-resident
`heapless::String<N>`. `N` is picked at compile time by the
`buffer-size-<N>` feature family (default 256). Overflow truncates
and appends `…`; the macro never panics on a long format string.

| Feature | Per-call-site stack frame |
|---------|---------------------------|
| `buffer-size-128` | 128 B |
| `buffer-size-256` | 256 B (default) |
| `buffer-size-512` | 512 B |
| `buffer-size-1024` | 1024 B |

## Interop with the `log` crate

Enable `nros-log/log-compat`:

```rust
nros_log::log_compat::install_log_crate_bridge()
    .expect("log crate already initialized");
// Now `log::info!(...)` calls from ecosystem crates flow through
// nros-log's dispatcher.

// Or fan out the other direction — re-emit nros records via `log`:
static SINKS: &[&dyn nros_log::LogSink] = &[
    &nros_log::sinks::PlatformSink,
    &nros_log::log_compat::LogCrateSink,
];
nros_log::init(SINKS);
```

Severity maps Trace↔Trace, Debug↔Debug, Info↔Info, Warn↔Warn,
Error↔Error round-trip. Fatal folds into Error one-way (`log` has
no Fatal).

## Working examples

- Rust: `examples/native/rust/logging/`
- C: `examples/native/c/logging/`
- C++: `examples/native/cpp/logging/`

Each demonstrates per-severity macros + the runtime threshold
filter.

## Reference

- `packages/core/nros-log/` — facade crate.
- `packages/platform/nros-platform-cffi/include/nros/platform.h` —
  `nros_platform_log_write` / `nros_platform_log_flush` /
  `nros_platform_register_log_writer` ABI.
- `packages/api/nros-c/include/nros/log.h` — C API surface
  (`NROS_LOG_*` macros + `nros_log_emit_fmt`).
- `packages/api/nros-cpp/include/nros/log.hpp` — C++ macros (the
  legacy `NROS_INFO` / etc. file:line printf surface stays
  alongside the new `NROS_LOG_*` macros).

`/rosout` publication is explicitly out of scope today; the
dispatcher's `&'static [&dyn LogSink]` shape leaves room for an
add-later `RosoutSink` that consumes records alongside
`PlatformSink`.
