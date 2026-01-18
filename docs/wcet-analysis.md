# WCET Analysis for nano-ros

This document describes how to perform Worst-Case Execution Time (WCET) analysis for nano-ros in real-time embedded systems.

## Overview

WCET analysis is critical for hard real-time systems where timing guarantees must be met. nano-ros is designed with WCET analysis in mind:

- **No heap allocation** in the critical path (with RTIC feature)
- **Bounded loops** where possible
- **Static buffer sizes** via const generics
- **Predictable serialization** with CDR

## Key Operations and Their WCET Characteristics

### 1. Message Publishing

```rust
publisher.publish(&message)?;
```

**Components:**
1. CDR serialization (bounded by message size)
2. Zenoh put operation (network dependent)
3. Buffer copy (bounded by buffer size)

**WCET Factors:**
- Message struct size determines serialization time
- Network latency is typically unbounded (use timeouts)
- Static buffer size caps memory operations

**Measurement Points:**
```rust
// Instrument with cycle counter
let start = DWT::cycle_count();
publisher.publish(&msg)?;
let end = DWT::cycle_count();
defmt::info!("Publish cycles: {}", end.wrapping_sub(start));
```

### 2. Message Reception (Polling)

```rust
node.poll_read()?;
```

**Components:**
1. Zenoh receive (may return immediately if no data)
2. CDR deserialization (bounded by buffer size)
3. Callback invocation (user-defined)

**WCET Factors:**
- Polling is bounded: returns immediately if no data
- Deserialization bounded by RX buffer size
- User callback WCET must be analyzed separately

### 3. Keepalive

```rust
node.send_keepalive()?;
```

**Components:**
1. Zenoh lease renewal (simple network operation)

**WCET Factors:**
- Relatively constant time
- Network latency considerations apply

## Static Analysis Tools

### 1. RAUK (Rust WCET Analysis)

For Cortex-M targets, use RAUK for static WCET analysis:

```bash
# Install RAUK
cargo install rauk

# Analyze a function
rauk analyze --target thumbv7em-none-eabihf \
    --function nano_ros_serdes::cdr::encode_i32
```

### 2. Manual Cycle Counting

For embedded targets, use the DWT cycle counter:

```rust
use cortex_m::peripheral::DWT;

fn measure_wcet<F: FnOnce() -> R, R>(f: F) -> (R, u32) {
    let start = DWT::cycle_count();
    let result = f();
    let end = DWT::cycle_count();
    (result, end.wrapping_sub(start))
}

// Usage
let (result, cycles) = measure_wcet(|| {
    publisher.publish(&msg)
});
defmt::info!("WCET: {} cycles @ 168MHz = {} us",
    cycles, cycles / 168);
```

### 3. Using Tracing with defmt

Enable trace-level logging to see timing:

```toml
[env]
DEFMT_LOG = "trace"
```

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

**Calculation:**
- CDR serialization: ~O(n) where n = message size
- Buffer copy: ~O(buffer_size)
- Network operations: bounded by timeout

## RTIC Priority Configuration

For RTIC applications, configure priorities based on WCET:

```rust
// Higher priority = shorter WCET required
#[task(priority = 3)]  // Highest - zenoh poll (short WCET)
async fn zenoh_poll(_cx: zenoh_poll::Context) { ... }

#[task(priority = 2)]  // Medium - keepalive
async fn zenoh_keepalive(_cx: zenoh_keepalive::Context) { ... }

#[task(priority = 1)]  // Lower - publishing (longer WCET)
async fn publisher_task(cx: publisher_task::Context) { ... }
```

## Timeout Configuration

For operations with unbounded network WCET, use timeouts:

```rust
// Configure zenoh with timeout
let config = Config::client("tcp/192.168.1.1:7447")
    .with_timeout_ms(100);  // 100ms timeout

// In application code, handle timeouts
match node.poll_read_timeout(Duration::from_millis(10)) {
    Ok(()) => { /* process */ }
    Err(Error::Timeout) => { /* continue */ }
    Err(e) => { /* handle error */ }
}
```

## Memory Allocation Analysis

### Stack Usage

Analyze stack usage for WCET-critical paths:

```bash
# Build with stack usage info
RUSTFLAGS="-Z emit-stack-sizes" cargo build --release

# Analyze with cargo-call-stack
cargo install cargo-call-stack
cargo call-stack --target thumbv7em-none-eabihf
```

### Static Memory

All static allocations are bounded:

| Component              | Size Formula                    |
|------------------------|---------------------------------|
| ConnectedNode          | ~200 bytes + MAX_TOKENS * 48    |
| ConnectedSubscriber    | ~64 bytes + RX_BUF              |
| ConnectedPublisher     | ~128 bytes + TX_BUF             |
| ConnectedServiceServer | ~96 bytes + REQ_BUF + REPLY_BUF |

## Example WCET Budget

For a 168 MHz STM32F4 with 1ms task period:

| Operation                  | Budget (cycles) | Budget (µs) |
|----------------------------|-----------------|-------------|
| zenoh_poll                 | 16,800          | 100         |
| Message deserialize (256B) | 5,040           | 30          |
| User callback              | 8,400           | 50          |
| **Total poll task**        | **30,240**      | **180**     |
|                            |                 |             |
| Message serialize (256B)   | 5,040           | 30          |
| zenoh_put                  | 16,800          | 100         |
| **Total publish**          | **21,840**      | **130**     |
|                            |                 |             |
| keepalive                  | 8,400           | 50          |

**Margin:** 1000µs - 180µs - 130µs - 50µs = 640µs (64% margin)

## Best Practices

1. **Measure, don't guess**: Always measure on real hardware
2. **Add margin**: Real-time budgets should have 30-50% margin
3. **Bound all loops**: Avoid unbounded iterations
4. **Use timeouts**: Network operations must have timeouts
5. **Static allocation**: Use const generics for deterministic memory
6. **Profile regularly**: WCET can change with code changes

## Tools Summary

| Tool              | Purpose                  |
|-------------------|--------------------------|
| DWT cycle counter | Runtime measurement      |
| RAUK              | Static WCET analysis     |
| cargo-call-stack  | Stack usage analysis     |
| defmt tracing     | Execution logging        |
| Keil µVision      | Commercial WCET analysis |

## References

- [RTIC Book - Real-Time For The Masses](https://rtic.rs/)
- [Cortex-M DWT](https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit)
- [WCET Analysis Survey](https://www.sciencedirect.com/science/article/pii/S1574013708000185)
