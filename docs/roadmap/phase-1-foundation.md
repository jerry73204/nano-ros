# Phase 1: Foundation

**Status: COMPLETE** ✓

Establish core infrastructure for nano-ros: serialization, type system, and basic abstractions.

## Goals

- Implement CDR serialization compatible with ROS 2 / rmw_zenoh
- Create proc macros for message type generation
- Define core traits and error types
- Ensure full `no_std` compatibility

## Work Items

### 1. CDR Serialization (`nano-ros-serdes`)

#### 1.1 Core CDR Implementation
- [x] Implement `CdrWriter` with alignment handling
- [x] Implement `CdrReader` with alignment handling
- [x] Add 4-byte encapsulation header support (`0x00 0x01 0x00 0x00`)
- [x] Handle endianness (little-endian default)

#### 1.2 Primitive Type Serialization
- [x] `bool`, `char`, `i8`, `u8`
- [x] `i16`, `u16`, `i32`, `u32`
- [x] `i64`, `u64`
- [x] `f32`, `f64`
- [x] Strings (4-byte length + data + null terminator)

#### 1.3 Compound Type Support
- [x] Fixed-size arrays
- [x] Sequences (variable-length arrays with 4-byte count prefix)
- [x] Nested structs

#### 1.4 `no_std` Variants
- [x] `heapless::String<N>` serialization
- [x] `heapless::Vec<T, N>` serialization
- [ ] `alloc::String` / `alloc::Vec` (behind `alloc` feature) - deferred

### 2. Proc Macros (`nano-ros-macros`)

#### 2.1 `#[derive(RosMessage)]`
- [x] Parse struct fields and types
- [x] Generate `Serialize` impl
- [x] Generate `Deserialize` impl
- [x] Generate `TYPE_NAME` constant
- [x] Generate `TYPE_HASH` constant (via attribute)

#### 2.2 Attributes
- [x] `#[ros(type_name = "...")]` - full ROS type name
- [x] `#[ros(hash = "...")]` - type hash
- [ ] `#[ros(skip)]` - skip field (if needed) - deferred

#### 2.3 `#[derive(RosService)]`
- [x] Generate request/reply type associations
- [x] Service type name and hash

### 3. Core Types (`nano-ros-core`)

#### 3.1 Traits
- [x] `RosMessage` trait with `TYPE_NAME`, `TYPE_HASH`
- [x] `RosService` trait linking request/reply types
- [x] Re-export `Serialize` / `Deserialize` from serdes

#### 3.2 Error Types
- [x] `SerError` - serialization errors
- [x] `DeserError` - deserialization errors

#### 3.3 Time Types
- [x] `Time` struct (sec: i32, nanosec: u32)
- [x] `Duration` struct (sec: i32, nanosec: u32)

### 4. Standard Message Types (`nano-ros-types`)

#### 4.1 `builtin_interfaces`
- [x] `Time`
- [x] `Duration`

#### 4.2 `std_msgs`
- [x] `Header`
- [x] `String`
- [x] `Bool`, `Byte`, `Char`
- [x] `Int8`, `Int16`, `Int32`, `Int64`
- [x] `UInt8`, `UInt16`, `UInt32`, `UInt64`
- [x] `Float32`, `Float64`
- [x] `Empty`

#### 4.3 `geometry_msgs` (minimal set)
- [x] `Point`, `Point32`
- [x] `Vector3`
- [x] `Quaternion`
- [x] `Pose`, `PoseStamped`
- [x] `Twist`, `TwistStamped`

### 5. Testing & Validation

#### 5.1 Unit Tests (`nano-ros-serdes`)
- [x] Round-trip serialization for all primitive types
- [x] Round-trip for `heapless::String<N>`
- [x] Round-trip for `heapless::Vec<T, N>`
- [x] Round-trip for fixed-size arrays
- [x] Alignment verification (4 byte boundaries)
- [x] Encapsulation header parsing
- [ ] Buffer overflow returns error (not panic) - partial
- [ ] Partial read returns error - partial
- [ ] Empty sequence/string handling - deferred

#### 5.2 Unit Tests (`nano-ros-macros`)
- [x] Derive on struct with multiple fields
- [x] Derive on struct with const generics
- [x] Derive on nested structs
- [x] `#[ros(type_name)]` attribute parsing
- [x] `#[ros(hash)]` attribute parsing
- [ ] Compile-fail tests for invalid usage - deferred

#### 5.3 Unit Tests (`nano-ros-types`)
- [x] `Time` serialization roundtrip
- [x] `Header` serialization roundtrip
- [x] `std_msgs::String` roundtrip
- [x] `Int32` roundtrip
- [x] `Point`, `Quaternion`, `Twist` roundtrip

#### 5.4 Compatibility Tests
- [ ] Serialize with nano-ros, deserialize with Micro-CDR (C) - deferred
- [ ] Serialize with Micro-CDR (C), deserialize with nano-ros - deferred
- [ ] Capture ROS 2 message bytes via `ros2 topic echo --raw` - deferred
- [ ] Verify nano-ros output matches captured bytes - deferred
- [ ] Test against rmw_zenoh message format - deferred

#### 5.5 `no_std` Build Checks
- [x] `cargo check --no-default-features` (all crates)
- [x] `cargo check --no-default-features --target thumbv7em-none-eabihf`
- [ ] `cargo check --no-default-features --features alloc` - deferred
- [x] Verify no `std::` imports without feature gate

#### 5.6 CI Checks
- [x] `cargo fmt --check`
- [x] `cargo clippy -- -D warnings`
- [x] `cargo test`
- [ ] `cargo doc --no-deps` - deferred
- [ ] MSRV check (rust 1.75) - deferred
- [ ] Miri for undefined behavior - deferred

## Acceptance Criteria

1. ✓ All primitive types serialize/deserialize correctly
2. ✓ Proc macros generate correct code for message structs
3. ✓ Standard message types implemented
4. ✓ All crates compile with `#![no_std]`
5. ✓ Unit tests pass (22 tests)

## Test Results

```
22 tests run: 22 passed, 0 skipped
- nano-ros-serdes: 10 tests
- nano-ros-core: 2 tests
- nano-ros-types: 10 tests
```

## Dependencies

- `heapless` - static collections
- `syn`, `quote`, `proc-macro2` - proc macros

## Existing CDR Crates (Research)

No existing Rust CDR crate supports `no_std`:

| Crate                                                 | `no_std` | Notes                           |
|-------------------------------------------------------|----------|---------------------------------|
| [cdr](https://crates.io/crates/cdr)                   | No       | Uses `thiserror` (requires std) |
| [cdr-encoding](https://crates.io/crates/cdr-encoding) | No       | std dependencies                |
| [RustDDS](https://crates.io/crates/rustdds)           | No       | Full DDS, not embedded-focused  |

**Decision**: Implement CDR from scratch using Micro-CDR (C) as reference.

**Useful references**:
- [postcard](https://crates.io/crates/postcard) - Example of `no_std` serde serializer
- [serde no_std docs](https://serde.rs/no-std.html) - Patterns for `no_std` serde

## Deferred Items (Future Work)

- `alloc` feature for dynamic String/Vec support
- Compatibility tests against real ROS 2 messages
- Compile-fail tests for proc macros
- Full error handling tests
- CI workflow setup
- Documentation generation

## Non-Goals (Deferred to Later Phases)

- Transport layer (zenoh integration)
- Node/Publisher/Subscriber implementation
- Parameter server
- Service client/server
- Async support

## References

- Pico-ROS picoserdes: `external/Pico-ROS-software/src/picoserdes.h`
- Micro-CDR: `external/Pico-ROS-software/thirdparty/Micro-CDR/`
- CDR spec: OMG DDS-XTypes 1.3
