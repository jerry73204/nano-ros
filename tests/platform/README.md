# Platform Backend Tests

Tests that verify zenoh-pico-shim works correctly on each platform backend.

## Test Suites

| Test | Description | Requirements |
|------|-------------|--------------|
| `posix.sh` | POSIX platform (reference baseline) | zenohd |
| `smoltcp-sim.sh` | smoltcp platform (x86_64 simulation) | None (compile + unit tests) |
| `generic.sh` | Generic platform (compile-only) | None |

## Running Tests

```bash
# Run all platform tests
./tests/platform/run.sh

# Run with verbose output
./tests/platform/run.sh --verbose

# Run individual tests
./tests/platform/posix.sh
./tests/platform/smoltcp-sim.sh
./tests/platform/generic.sh
```

## Test Details

### POSIX Platform (`posix.sh`)

Tests the POSIX backend which uses zenoh-pico's native POSIX platform layer.
This is the reference implementation used on desktop/Linux systems.

**Tests:**
- Session lifecycle (open, close, reconnect)
- Publisher declaration and publishing
- Subscriber declaration and callback invocation
- Liveliness token declaration
- ZenohId retrieval and formatting
- Max publishers limit (8)
- Polling API (no-op on POSIX)

**Requirements:**
- zenohd router running on `tcp/127.0.0.1:7447`

### smoltcp Platform (`smoltcp-sim.sh`)

Tests the smoltcp backend which provides a custom platform layer for bare-metal
embedded systems using the smoltcp TCP/IP stack.

**Tests:**
- Compile verification for x86_64-unknown-linux-gnu
- Bump allocator functionality
- Clock synchronization functions
- Socket buffer management
- Random number generation

**Note:** Full network testing requires hardware (see Phase 8.9).

### Generic Platform (`generic.sh`)

Verifies that zenoh-pico-shim compiles without any platform backend.
This is useful for CI to catch API breakage.

**Tests:**
- Compile without any platform feature
- Compile with only `std` feature
- Compile with only `alloc` feature

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ZENOH_LOCATOR` | `tcp/127.0.0.1:7447` | Zenoh router address (POSIX tests) |
| `TEST_TIMEOUT` | `30` | Test timeout in seconds |

## Adding New Platform Tests

1. Create a new script in `tests/platform/`
2. Source common utilities:
   ```bash
   source "$(dirname "$0")/../common/utils.sh"
   ```
3. Use `setup_cleanup` trap for resource cleanup
4. Return 0 on success, non-zero on failure
5. Add the test to `run.sh` orchestrator
