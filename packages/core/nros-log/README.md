# nros-log

Phase 88 — ROS 2 style leveled logging facade. `no_std` + optional `alloc`
+ optional `std`. Zero target deps; per-platform log delivery flows
through the `nros_platform_log_*` ABI (see `nros-platform-cffi`).

## Quick start

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

`sinks::default()` returns a `&'static [&dyn LogSink]` containing one
`PlatformSink` — that's the only sink that calls
`nros_platform_log_write`. Boards / apps can add their own sinks
(e.g. a future `RosoutSink`) by composing a `&'static` slice.

## Macros

- `log_debug!` / `log_info!` / `log_warn!` / `log_error!` / `log_fatal!` —
  rclrs's spellings. Rust follows rclrs because the three ROS 2 client
  libraries disagree with each other (`log_info!` / `RCLCPP_INFO` /
  `RCUTILS_LOG_INFO_NAMED`) and rclrs is the one a ported Rust node is read
  beside (RFC-0089, settled 2026-09-04).
- `nros_trace!` — kept under OUR prefix: rclrs stops at `debug`, so TRACE has
  no upstream twin and must not borrow a name that implies one.
- All take `(logger, fmt, args…)`.
- Below-ceiling macros expand to `()` (the format call is
  dead-code-eliminated).

### Throttled variants (phase-417 W4.d)

- `nros_info_throttle!(logger, interval_ms, fmt, args…)` — reads the platform
  clock. **Needs the `platform-clock` feature**, and says so with a
  `compile_error!` rather than compiling into a window with no time base:
  without a clock every timestamp is a constant `0`, so the window can never
  elapse and the site would emit every record while reading exactly like a
  working throttle.
- `nros_info_throttle_at!(logger, now_ns, interval_ms, fmt, args…)` — you supply
  the time. This is `rclcpp`'s shape (`RCLCPP_INFO_THROTTLE(logger, clock, …)`
  names its clock) and the only throttled form available with no platform port.
- One window per CALL SITE, not per logger — each expansion declares its own
  `static ThrottleState`.
- The FIRST record at a site always emits, as in `rclcpp`. The severity
  threshold is tested BEFORE the window, so a record the level filters does not
  consume it.
- The rule itself is `nros_log::throttle_admits`, a pure function; the C API's
  `nros_log_throttle_admit` calls the same one over caller-owned storage.

There is no `*_once` / `*_skip_first` family here. `nros_core::logger::Logger`
has one, on the logger that forwards to the `log` crate rather than to
`nros_platform_log_write`; porting it is not part of W4.d.

## Named loggers and per-logger levels

- `get_logger(name)` — LOOKUP. Answers `DEFAULT_LOGGER` for a name no `'static`
  `Logger` was `register_logger`ed under.
- `get_or_create_logger(name)` — `rclcpp::get_logger`'s shape, and what the C
  and C++ wrappers need. Creates from a bounded static arena sized by the
  `dynamic-loggers-<N>` feature (default 16, `dynamic-loggers-0` declines it).
  Returns `None` rather than a logger under the wrong name: aliasing onto
  `DEFAULT_LOGGER` would make `set_level` on the result move the threshold of
  every other unregistered name in the image.
- `Logger::set_level` / `level` / `is_enabled` — per-logger runtime threshold,
  checked before any sink sees the record.

## Sinks

- `init(&'static [&'static dyn LogSink])` — REPLACES the list. The board's verb:
  it names, once, the whole delivery it chose.
- `add_sink(&'static dyn LogSink)` — APPENDS, up to `MAX_ADDED_SINKS`. The
  consumer's verb: a library teeing records to /rosout must not discard what the
  board installed. Returns `false` when the registry is full — check it.

Records raised before any sink existed are replayed into whichever of the two
calls installs the first one (see the `early` module).

## Compile-time level ceiling

Pick at most one Cargo feature:

| Feature           | Macros above ceiling that emit                  |
|-------------------|-------------------------------------------------|
| `max-level-trace` | trace, debug, info, warn, error, fatal (default)|
| `max-level-debug` | debug, info, warn, error, fatal                 |
| `max-level-info`  | info, warn, error, fatal                        |
| `max-level-warn`  | warn, error, fatal                              |
| `max-level-error` | error, fatal                                    |
| `max-level-off`   | (none)                                          |

## Buffer size

Pick at most one Cargo feature. Default 256.

| Feature              | Per-call-site stack frame for formatting |
|----------------------|------------------------------------------|
| `buffer-size-128`    | 128 B                                    |
| `buffer-size-256`    | 256 B (default)                          |
| `buffer-size-512`    | 512 B                                    |
| `buffer-size-1024`   | 1024 B                                   |

Overflow truncates + appends `…`; `log()` never fails.

## Backend delivery (per platform)

See `docs/roadmap/archived/phase-88-nros-log.md` for the per-platform
impl table. Summary: POSIX → stderr; Zephyr → `LOG_*`; ESP-IDF →
`ESP_LOG_*`; NuttX → `syslog`; FreeRTOS / ThreadX / bare-metal →
board-registered UART / semihosting / defmt writer fn-ptr.

Each impl lives in its `nros-platform-<rtos>` crate, behind the
ABI. To change behavior on a target, change the platform impl, not
this crate.

## Phase status

See `docs/roadmap/archived/phase-88-nros-log.md`. v1 = facade + macros +
ABI + POSIX impl + PlatformSink + the Rust API. C/C++ bindings,
per-RTOS impls, examples, and tests land incrementally.
