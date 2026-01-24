# smoltcp Integration Tests

Tests for the smoltcp platform layer of zenoh-pico-shim-sys.

## Overview

The smoltcp platform layer provides a Rust FFI interface for zenoh-pico to work
with the [smoltcp](https://github.com/smoltcp-rs/smoltcp) TCP/IP stack on bare-metal
and RTOS targets. These tests verify the platform layer functions work correctly.

## Test Files

| Script | Description |
|--------|-------------|
| `run.sh` | Test orchestrator (runs all smoltcp tests) |
| `allocator.sh` | Bump allocator tests (alloc, realloc, free) |
| `socket-buffers.sh` | Socket operations and buffer management |
| `clock-sync.sh` | Clock/timer functions |
| `poll-callback.sh` | Poll callback registration and invocation |

## Requirements

- Rust toolchain (stable)
- No external dependencies (no zenohd required)

## Running Tests

```bash
# Run all smoltcp tests
./tests/smoltcp/run.sh

# Run individual test suites
./tests/smoltcp/allocator.sh
./tests/smoltcp/socket-buffers.sh
./tests/smoltcp/clock-sync.sh
./tests/smoltcp/poll-callback.sh

# With verbose output
./tests/smoltcp/run.sh --verbose
```

## Test Categories

### Allocator Tests

Tests the bump allocator implementation:
- Basic allocation (alignment, null pointer handling)
- Multiple allocations (non-overlapping)
- Reallocation (grow, shrink, null input)
- Free operation (no-op for bump allocator)

### Socket Buffer Tests

Tests socket operations:
- Socket open/close lifecycle
- Multiple socket allocation
- Invalid handle error handling
- Connect and address storage
- RX buffer: push data, read data
- TX buffer: write data, pop data
- Connected flag management

### Clock Tests

Tests the monotonic clock:
- Initial value retrieval
- Set and get operations
- Large value handling (near u64::MAX)

### Poll Callback Tests

Tests the poll callback mechanism:
- Callback registration
- Callback invocation on poll()
- No-callback safety (null function pointer)

## Implementation Details

The smoltcp platform layer uses global static state:
- **Heap**: 16KB bump allocator
- **Sockets**: Up to 4 TCP sockets with 1KB RX/TX buffers each
- **Clock**: 64-bit millisecond counter (externally updated)
- **RNG**: XorShift32 PRNG

### Thread Safety

The platform layer is designed for **single-threaded operation only**.
Tests must be run with `--test-threads=1`:

```bash
cargo test -p zenoh-pico-shim-sys --features smoltcp -- --test-threads=1
```

### No External Dependencies

Unlike POSIX platform tests, smoltcp tests do not require:
- zenohd router
- Network connectivity
- Root privileges

This makes them suitable for CI environments and quick feedback.

## Troubleshooting

### Tests fail randomly
Run with single thread to avoid race conditions in global state:
```bash
cargo test -p zenoh-pico-shim-sys --features smoltcp -- --test-threads=1
```

### Allocation failures
The bump allocator has limited heap (16KB). Running many tests may exhaust it.
Tests are designed to be independent but share the global heap.

### Socket table full
Previous test failures may leave sockets open. The test handles this gracefully
by checking available slots.
