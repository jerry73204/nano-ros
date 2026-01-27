# WCET Analysis for nano-ros

This document describes how to perform Worst-Case Execution Time (WCET) analysis for nano-ros in real-time embedded systems, with specific focus on RTIC applications.

## Overview

WCET analysis is critical for hard real-time systems where timing guarantees must be met. nano-ros is designed with WCET analysis in mind:

- **No heap allocation** in the critical path (with RTIC feature)
- **Bounded loops** where possible
- **Static buffer sizes** via const generics
- **Predictable serialization** with CDR

## Analysis Methods

### Method 1: RTIC-Scope (Recommended)

RTIC-Scope provides non-intrusive hardware tracing via ARM's ITM/DWT:

**Installation:**
```bash
# Install the host-side tools
cargo install cargo-rtic-scope

# Add target-side tracing to your Cargo.toml
[dependencies]
cortex-m-rtic-trace = "0.1"
```

**Target Setup (STM32F4):**
```rust
#![no_std]
#![no_main]

use cortex_m_rtic_trace::{self, trace};
use rtic::app;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Configure ITM/DWT for tracing
        cortex_m_rtic_trace::configure(
            &mut cx.core.DCB,
            &mut cx.core.DWT,
            &mut cx.core.ITM,
            168_000_000,  // CPU frequency
        );

        // ... rest of init
    }

    #[task(priority = 2)]
    async fn zenoh_poll(_cx: zenoh_poll::Context) {
        loop {
            trace::task_enter!();  // Mark task entry

            // ... poll zenoh

            trace::task_exit!();   // Mark task exit
            Mono::delay(10.millis()).await;
        }
    }
}
```

**Host-Side Recording:**
```bash
# Start trace recording (requires debug probe with SWO support)
cargo rtic-scope --chip STM32F429ZI --release

# Output shows task execution timeline with nanosecond precision
```

**Analysis Output:**
```
Task Timeline (us):
=====================================
Time        Task            Duration
-------------------------------------
0           zenoh_poll      45.2
10000       zenoh_poll      43.8
10050       publisher_task  127.5
20000       zenoh_poll      44.1
...

Task Statistics:
=====================================
Task            Min     Avg     Max     Count
-------------------------------------
zenoh_poll      42.1    44.3    52.7    1000
publisher_task  118.2   125.8   142.3   100
zenoh_keepalive 28.4    31.2    38.9    10
```

### Method 2: DWT Cycle Counter (Manual)

For targets without SWO or for quick measurements:

```rust
use cortex_m::peripheral::DWT;

/// WCET measurement wrapper
pub struct WcetMeasure {
    start: u32,
    max_cycles: u32,
    min_cycles: u32,
    samples: u32,
    cpu_mhz: u32,
}

impl WcetMeasure {
    pub fn new(cpu_mhz: u32) -> Self {
        Self {
            start: 0,
            max_cycles: 0,
            min_cycles: u32::MAX,
            samples: 0,
            cpu_mhz,
        }
    }

    #[inline(always)]
    pub fn start(&mut self) {
        self.start = DWT::cycle_count();
    }

    #[inline(always)]
    pub fn stop(&mut self) {
        let end = DWT::cycle_count();
        let elapsed = end.wrapping_sub(self.start);

        self.max_cycles = self.max_cycles.max(elapsed);
        self.min_cycles = self.min_cycles.min(elapsed);
        self.samples += 1;
    }

    pub fn report(&self) -> WcetReport {
        WcetReport {
            min_us: (self.min_cycles as f32) / (self.cpu_mhz as f32),
            max_us: (self.max_cycles as f32) / (self.cpu_mhz as f32),
            samples: self.samples,
        }
    }
}

pub struct WcetReport {
    pub min_us: f32,
    pub max_us: f32,
    pub samples: u32,
}
```

**Usage in RTIC task:**
```rust
#[task(priority = 2, local = [wcet: WcetMeasure = WcetMeasure::new(168)])]
async fn zenoh_poll(cx: zenoh_poll::Context) {
    loop {
        cx.local.wcet.start();

        // ... actual work

        cx.local.wcet.stop();

        // Report every 1000 samples
        if cx.local.wcet.samples % 1000 == 0 {
            let report = cx.local.wcet.report();
            defmt::info!("zenoh_poll WCET: min={} max={} us",
                report.min_us, report.max_us);
        }

        Mono::delay(10.millis()).await;
    }
}
```

### Method 3: defmt Timestamped Logging

For development and quick analysis:

```rust
use defmt_rtt as _;

#[task(priority = 1)]
async fn publisher_task(cx: publisher_task::Context) {
    loop {
        let start = Mono::now();

        // Serialize message
        defmt::trace!("serialize start");
        let msg = Int32 { data: 42 };
        let bytes = msg.serialize_cdr();
        defmt::trace!("serialize end");

        // Publish
        defmt::trace!("publish start");
        publisher.publish_raw(&bytes).ok();
        defmt::trace!("publish end");

        let elapsed = Mono::now() - start;
        defmt::info!("publish total: {} us", elapsed.to_micros());

        Mono::delay(100.millis()).await;
    }
}
```

## nano-ros Task WCET Characteristics

> **Note:** The WCET values in this section are **illustrative examples** to demonstrate the analysis methodology. Actual values must be measured on real hardware with your specific configuration. The examples assume a 168 MHz STM32F4 with zenoh-pico connectivity.

### Task: zenoh_poll

**Purpose:** Poll zenoh for incoming messages

**Components:**
1. zenoh-pico `z_recv()` - Check for incoming data
2. CDR deserialization (if data received)
3. Subscriber callback invocation

**WCET Factors:**
| Factor | Impact | Mitigation |
|--------|--------|------------|
| Network data available | Variable | Bounded by RX buffer |
| Message size | O(n) deserialize | Use const generic buffers |
| Callback complexity | User-defined | Keep callbacks short |

**Typical WCET (168MHz STM32F4):**
- No data: 30-50 µs
- With 256-byte message: 80-150 µs
- With 1KB message: 200-400 µs

### Task: zenoh_keepalive

**Purpose:** Send keepalive to maintain session

**Components:**
1. zenoh-pico lease renewal

**WCET Factors:**
| Factor | Impact | Mitigation |
|--------|--------|------------|
| Network latency | Variable | Use timeout |

**Typical WCET (168MHz STM32F4):**
- Normal: 25-40 µs

### Task: publisher_task

**Purpose:** Serialize and publish ROS 2 messages

**Components:**
1. CDR serialization
2. RMW attachment preparation
3. zenoh-pico `z_put()`

**WCET Factors:**
| Factor | Impact | Mitigation |
|--------|--------|------------|
| Message size | O(n) serialize | Bound message size |
| Network congestion | Variable | Use timeout |

**Typical WCET (168MHz STM32F4):**
- 64-byte message: 80-120 µs
- 256-byte message: 120-180 µs
- 1KB message: 300-500 µs

## Buffer Size Impact on WCET

The const generic buffer sizes directly affect WCET:

```rust
// Smaller buffer = lower WCET for serialization
let subscriber: ConnectedSubscriber<MyMsg, 256> =
    node.create_subscriber_sized("/topic")?;

// Larger buffer = higher WCET but supports bigger messages
let subscriber: ConnectedSubscriber<MyMsg, 4096> =
    node.create_subscriber_sized("/topic")?;
```

**CDR Serialization Complexity:**

| Operation | Complexity | Notes |
|-----------|------------|-------|
| Primitive (i32, f64) | O(1) | 4-8 byte copy + alignment |
| Fixed array [T; N] | O(N) | N × element size |
| String | O(len) | Length prefix + copy |
| Sequence (Vec) | O(len) | Length prefix + elements |

## WCET Analysis Workflow for nano-ros RTIC Examples

### Step 1: Identify Tasks and Periods

From `examples/stm32f4-rs-rtic/src/main.rs`:

| Task | Period (T) | Priority |
|------|------------|----------|
| zenoh_poll | 10 ms | 2 (high) |
| zenoh_keepalive | 1000 ms | 1 (low) |
| publisher_task | 100 ms | 1 (low) |

### Step 2: Measure WCET

Add instrumentation to each task (see Method 2 above).

Run for extended period to capture worst-case:
```bash
# Flash and run
cargo run --release

# Collect at least 10,000 samples per task
# Run under various conditions:
# - Network congestion
# - Maximum message sizes
# - Concurrent task execution
```

### Step 3: Document Results

Create a WCET budget table:

| Task | Measured Max | Safety Margin (20%) | WCET Budget |
|------|--------------|---------------------|-------------|
| zenoh_poll | 52.7 µs | 10.5 µs | 63.2 µs |
| zenoh_keepalive | 38.9 µs | 7.8 µs | 46.7 µs |
| publisher_task | 142.3 µs | 28.5 µs | 170.8 µs |

### Step 4: Perform Schedulability Analysis

See `docs/schedulability-analysis.md` for RMA/response time analysis.

## Static Analysis Tools

### cargo-call-stack

Analyze stack usage for WCET-critical paths:

```bash
# Install
cargo install cargo-call-stack

# Analyze (requires nightly)
RUSTFLAGS="-Z emit-stack-sizes" cargo +nightly call-stack \
    --target thumbv7em-none-eabihf \
    --bin stm32f4-rs-rtic-example
```

### Clippy for Real-Time Code

```bash
# Check for patterns that harm WCET predictability
cargo clippy -- \
    -W clippy::large_stack_arrays \
    -W clippy::large_types_passed_by_value \
    -W clippy::inefficient_to_string
```

### Assembly Inspection

For critical paths, inspect generated assembly:

```bash
# Generate assembly
cargo objdump --release -- -d > output.asm

# Or use cargo-show-asm
cargo install cargo-show-asm
cargo asm --release nano_ros_serdes::cdr::encode_i32
```

## Best Practices

### 1. Bound All Loops

```rust
// BAD: Unbounded loop
while let Some(msg) = queue.pop() {
    process(msg);
}

// GOOD: Bounded loop
for _ in 0..MAX_MESSAGES_PER_POLL {
    if let Some(msg) = queue.pop() {
        process(msg);
    } else {
        break;
    }
}
```

### 2. Use Timeouts for Network Operations

```rust
// Configure with timeout
let config = Config::client("tcp/192.168.1.1:7447")
    .with_timeout_ms(100);
```

### 3. Minimize Critical Section Duration

```rust
// BAD: Long critical section
counter.lock(|c| {
    let msg = serialize(c);  // Serialization inside lock!
    publish(&msg);
    *c += 1;
});

// GOOD: Short critical section
let count = counter.lock(|c| {
    *c += 1;
    *c
});
let msg = serialize(&count);  // Serialization outside lock
publish(&msg);
```

### 4. Separate Time-Critical and Non-Critical Code

```rust
// High priority: Only time-critical operations
#[task(priority = 3)]
async fn fast_control_loop(cx: fast_control_loop::Context) {
    // Minimal work, bounded WCET
}

// Low priority: Can be preempted
#[task(priority = 1)]
async fn logging_task(cx: logging_task::Context) {
    // Non-time-critical work
}
```

## Memory Layout Analysis

### Static Memory Sizes

| Component | Size Formula |
|-----------|--------------|
| ConnectedNode | ~200 bytes + MAX_TOKENS × 48 |
| ConnectedSubscriber | ~64 bytes + RX_BUF |
| ConnectedPublisher | ~128 bytes + TX_BUF |
| ConnectedServiceServer | ~96 bytes + REQ_BUF + REPLY_BUF |

### Stack Budget per Task

```rust
// Example stack allocation in RTIC
#[app(device = ..., dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    // Default stack size per task: 256 words (1KB on 32-bit)
    // Increase if needed via RTIC configuration
}
```

## References

- [RTIC Book](https://rtic.rs/)
- [RTIC-Scope Documentation](https://rtic-scope.github.io/)
- [ARM DWT Documentation](https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit)
- [WCET Analysis Survey](https://www.sciencedirect.com/science/article/pii/S1574013708000185)
- [Stack Resource Policy (Baker, 1991)](https://www.math.unipd.it/~tullio/RTS/2009/Baker-1991.pdf)
