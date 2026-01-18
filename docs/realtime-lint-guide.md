# Real-Time Anti-Pattern Detection for nano-ros

This guide describes static analysis methods and tools for detecting anti-patterns that violate real-time guarantees in Rust embedded applications.

## Overview

Real-time systems require deterministic execution times. Common anti-patterns that break real-time guarantees include:

| Anti-Pattern | Problem | Detection Method |
|--------------|---------|------------------|
| Unbounded loops | Infinite execution time | Clippy + custom lints |
| Recursion | Stack overflow, unbounded depth | cargo-call-stack |
| Heap allocation | Non-deterministic timing, fragmentation | no_std + forbid patterns |
| Blocking I/O | Unbounded wait times | Custom lints |
| Missing timeouts | Operations can hang forever | Custom lints |
| Large stack frames | Stack overflow | cargo-call-stack |

## Built-in Clippy Lints

### Loop and Iteration Lints

```bash
# Enable all loop-related lints
cargo clippy -- \
    -D clippy::infinite_iter \
    -D clippy::while_immutable_condition \
    -D clippy::never_loop \
    -D clippy::empty_loop
```

| Lint | Detects |
|------|---------|
| `infinite_iter` | Iterator chains guaranteed to be infinite |
| `while_immutable_condition` | Loop conditions that can never change |
| `never_loop` | Loops that exit on first iteration |
| `empty_loop` | `loop { }` without body (use `loop { hint::spin_loop() }`) |

### Memory and Performance Lints

```bash
cargo clippy -- \
    -W clippy::large_stack_arrays \
    -W clippy::large_types_passed_by_value \
    -W clippy::box_collection \
    -W clippy::rc_buffer
```

### Recommended Clippy Configuration

Create `clippy.toml` in your project root:

```toml
# Maximum size for stack-allocated arrays (bytes)
array-size-threshold = 512

# Warn on types larger than this passed by value
trivial-copy-size-limit = 16

# Cognitive complexity threshold
cognitive-complexity-threshold = 15
```

### Running Clippy for Real-Time Code

```bash
# Strict mode for real-time code
cargo clippy --all-targets -- \
    -D warnings \
    -D clippy::all \
    -W clippy::pedantic \
    -D clippy::infinite_iter \
    -D clippy::while_immutable_condition \
    -A clippy::module_name_repetitions
```

## Stack Analysis with cargo-call-stack

### Installation and Usage

```bash
# Install (requires nightly)
cargo +nightly install cargo-call-stack

# Build with stack size info
cd examples/rtic-stm32f4
RUSTFLAGS="-Z emit-stack-sizes" cargo +nightly build --release

# Generate call graph with stack sizes
cargo +nightly call-stack --release > call_graph.dot

# Visualize (requires graphviz)
dot -Tsvg call_graph.dot -o call_graph.svg
```

### Interpreting Results

The output shows:
- Each function's stack frame size
- Call graph relationships
- Maximum stack depth through any path
- Cycles (recursion) in the call graph

**Example output:**
```
digraph {
    "main" [label="main\n256 bytes"]
    "zenoh_poll" [label="zenoh_poll\n128 bytes"]
    "publisher_task" [label="publisher_task\n512 bytes"]

    "main" -> "zenoh_poll"
    "main" -> "publisher_task"
}
```

### Limitations

- Requires fat LTO (`lto = "fat"` in Cargo.toml)
- Limited support for programs linking `std`
- Indirect calls (function pointers, trait objects) may not be analyzed
- Best for embedded `no_std` programs

## Preventing Heap Allocation

### Method 1: no_std Without Allocator

The simplest approach - don't provide a global allocator:

```rust
#![no_std]
#![no_main]

// No #[global_allocator] defined
// Attempting to use Box, Vec, String will fail to compile
```

### Method 2: Compile-Time Enforcement

```rust
#![no_std]
#![forbid(unsafe_code)]  // Also prevents custom allocators

use heapless::{Vec, String};  // Static-sized alternatives

// This will NOT compile:
// let v = alloc::vec::Vec::new();  // Error: no allocator

// This works:
let v: heapless::Vec<u8, 256> = heapless::Vec::new();
```

### Method 3: Custom Lint (Dylint)

For projects that need `alloc` but want to restrict usage in certain modules:

```rust
// In real-time critical code, add:
#![deny(clippy::disallowed_methods)]
```

With `clippy.toml`:
```toml
disallowed-methods = [
    { path = "alloc::vec::Vec::push", reason = "Use heapless::Vec in RT code" },
    { path = "alloc::boxed::Box::new", reason = "No heap in RT code" },
    { path = "alloc::string::String::new", reason = "Use heapless::String" },
]
```

## Detecting Missing Timeouts

### Pattern: I/O Operations Without Timeout

Anti-pattern:
```rust
// BAD: No timeout - can block forever
let data = socket.read(&mut buf)?;
```

Correct pattern:
```rust
// GOOD: Explicit timeout
socket.set_read_timeout(Some(Duration::from_millis(100)))?;
let data = socket.read(&mut buf)?;
```

### Custom Lint Approach

Create a Dylint lint to detect I/O calls without preceding timeout configuration:

```rust
// Conceptual lint logic (simplified)
fn check_io_without_timeout(expr: &Expr) {
    if is_io_operation(expr) && !has_timeout_in_scope(expr) {
        emit_warning("I/O operation without timeout may block indefinitely");
    }
}
```

### Manual Audit Checklist

For code review, check that these operations have timeouts:

- [ ] `TcpStream::connect()` - use `connect_timeout()`
- [ ] `socket.read()` / `socket.write()` - set socket timeout
- [ ] `channel.recv()` - use `recv_timeout()`
- [ ] zenoh operations - configure session timeout
- [ ] Any blocking syscall

## Detecting Unbounded Loops

### Clippy Detection

```bash
cargo clippy -- -D clippy::infinite_iter -D clippy::while_immutable_condition
```

### Manual Patterns to Audit

**Potentially unbounded:**
```rust
// BAD: No termination guarantee
loop {
    if let Some(msg) = queue.pop() {
        process(msg);
    }
}

// BAD: Condition may never be false
while !flag.load(Ordering::Relaxed) {
    do_work();
}
```

**Bounded alternatives:**
```rust
// GOOD: Bounded iteration count
for _ in 0..MAX_ITERATIONS {
    if let Some(msg) = queue.pop() {
        process(msg);
    } else {
        break;
    }
}

// GOOD: Timeout-based termination
let deadline = Instant::now() + Duration::from_millis(10);
while Instant::now() < deadline {
    if let Some(msg) = queue.pop() {
        process(msg);
    } else {
        break;
    }
}
```

### Custom Lint for Loop Bounds

A Dylint lint could check:
1. All `loop` blocks have a `break` or `return` that's reachable
2. All `while` conditions involve a variable that's modified in the loop
3. Flag loops without explicit iteration limits in `#[task]` functions

## Detecting Recursion

### cargo-call-stack Cycle Detection

```bash
# Will report cycles in call graph
cargo +nightly call-stack --release 2>&1 | grep -i "cycle"
```

### Clippy Recursion Lint

```bash
# Warn on unconditional recursion
cargo clippy -- -D clippy::unconditional_recursion
```

### Manual Pattern Detection

```rust
// BAD: Direct recursion
fn process(node: &Node) {
    process(&node.child);  // May overflow stack
}

// BAD: Mutual recursion
fn a() { b(); }
fn b() { a(); }

// GOOD: Iterative with explicit stack
fn process(root: &Node) {
    let mut stack: heapless::Vec<&Node, 32> = heapless::Vec::new();
    stack.push(root).ok();

    while let Some(node) = stack.pop() {
        // Process node
        if stack.push(&node.child).is_err() {
            // Handle stack full - bounded!
            break;
        }
    }
}
```

## Advanced: Custom Dylint Lints

### Setting Up Dylint

```bash
# Install dylint
cargo install cargo-dylint dylint-link

# Create a new lint library
cargo dylint new realtime_lints
cd realtime_lints
```

### Example: Detect Blocking in Async

```rust
// src/lib.rs
use clippy_utils::diagnostics::span_lint;
use rustc_lint::{LateContext, LateLintPass};
use rustc_session::declare_lint_pass;

declare_lint! {
    pub BLOCKING_IN_ASYNC,
    Warn,
    "blocking operations in async context"
}

declare_lint_pass!(BlockingInAsync => [BLOCKING_IN_ASYNC]);

impl<'tcx> LateLintPass<'tcx> for BlockingInAsync {
    fn check_expr(&mut self, cx: &LateContext<'tcx>, expr: &'tcx Expr<'tcx>) {
        // Check if we're in an async function
        // Check if expr is a blocking call (std::thread::sleep, std::fs::*, etc.)
        // Emit lint if both conditions are true
    }
}
```

### Recommended Custom Lints for Real-Time

| Lint | Purpose |
|------|---------|
| `blocking_in_async` | Detect std::thread::sleep, blocking I/O in async |
| `unbounded_loop_in_task` | Flag loops without bounds in RTIC tasks |
| `heap_in_interrupt` | Detect allocation in `#[interrupt]` handlers |
| `mutex_in_interrupt` | Flag std::sync::Mutex in interrupt context |
| `float_in_isr` | Warn about FP operations in ISRs (soft-float targets) |
| `missing_timeout` | I/O operations without timeout |

## CI Integration

### GitHub Actions Workflow

```yaml
name: Real-Time Static Analysis

on: [push, pull_request]

jobs:
  clippy-realtime:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with:
          components: clippy
      - name: Clippy with RT lints
        run: |
          cargo clippy --all-targets -- \
            -D warnings \
            -D clippy::infinite_iter \
            -D clippy::while_immutable_condition \
            -D clippy::unconditional_recursion \
            -W clippy::large_stack_arrays

  stack-analysis:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@nightly
      - name: Install cargo-call-stack
        run: cargo +nightly install cargo-call-stack
      - name: Analyze embedded examples
        run: |
          cd examples/rtic-stm32f4
          RUSTFLAGS="-Z emit-stack-sizes" cargo +nightly build --release
          cargo +nightly call-stack --release > stack_analysis.txt
          cat stack_analysis.txt
      - name: Check for recursion
        run: |
          if grep -q "cycle" stack_analysis.txt; then
            echo "ERROR: Recursion detected in call graph"
            exit 1
          fi

  miri-safety:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@nightly
        with:
          components: miri
      - name: Run Miri
        run: cargo +nightly miri test -p nano-ros-serdes -p nano-ros-core
```

### Just Recipe for Local Checks

Add to `justfile`:

```just
# Run real-time static analysis
check-realtime:
    @echo "Running Clippy with real-time lints..."
    cargo clippy --all-targets -- \
        -D warnings \
        -D clippy::infinite_iter \
        -D clippy::while_immutable_condition \
        -D clippy::unconditional_recursion \
        -W clippy::large_stack_arrays
    @echo "Checking embedded examples for stack usage..."
    cd examples/rtic-stm32f4 && cargo +nightly build --release 2>&1 | head -50
    @echo "Real-time checks complete!"

# Full static analysis suite
static-analysis: check-realtime
    @echo "Running Miri..."
    cargo +nightly miri test -p nano-ros-serdes -p nano-ros-core
    @echo "All static analysis passed!"
```

## Tool Summary

| Tool | Detects | Effort | CI-Ready |
|------|---------|--------|----------|
| **Clippy** | Infinite iters, recursion, large arrays | Low | Yes |
| **cargo-call-stack** | Stack usage, recursion cycles | Medium | Yes |
| **Miri** | Undefined behavior, memory errors | Low | Yes |
| **Dylint** | Custom patterns (blocking, heap, etc.) | High | Yes |
| **MIRAI** | Abstract interpretation, all paths | High | Experimental |
| **no_std** | Prevents std heap allocation | Low | Automatic |
| **heapless** | Static collections | Low | N/A |

## Quick Reference: Lint Flags

```bash
# Copy-paste for strict real-time checking
cargo clippy -- \
    -D warnings \
    -D clippy::all \
    -D clippy::infinite_iter \
    -D clippy::while_immutable_condition \
    -D clippy::never_loop \
    -D clippy::empty_loop \
    -D clippy::unconditional_recursion \
    -W clippy::large_stack_arrays \
    -W clippy::large_types_passed_by_value \
    -W clippy::cognitive_complexity
```

## References

- [Clippy Lints Reference](https://rust-lang.github.io/rust-clippy/master/index.html)
- [cargo-call-stack](https://github.com/japaric/cargo-call-stack)
- [Dylint - Custom Lints](https://github.com/trailofbits/dylint)
- [MIRAI Abstract Interpreter](https://github.com/facebookexperimental/MIRAI)
- [heapless Crate](https://docs.rs/heapless)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [RTIC Book](https://rtic.rs/)
