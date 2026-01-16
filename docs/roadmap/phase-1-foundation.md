# Phase 1: Foundation

Establish core infrastructure for nano-ros: serialization, type system, and basic abstractions.

## Goals

- Implement CDR serialization compatible with ROS 2 / rmw_zenoh
- Create proc macros for message type generation
- Define core traits and error types
- Ensure full `no_std` compatibility

## Work Items

### 1. CDR Serialization (`nano-ros-serdes`)

#### 1.1 Core CDR Implementation
- [ ] Implement `CdrWriter` with alignment handling
- [ ] Implement `CdrReader` with alignment handling
- [ ] Add 4-byte encapsulation header support (`0x00 0x01 0x00 0x00`)
- [ ] Handle endianness (little-endian default)

#### 1.2 Primitive Type Serialization
- [ ] `bool`, `char`, `i8`, `u8`
- [ ] `i16`, `u16`, `i32`, `u32`
- [ ] `i64`, `u64`
- [ ] `f32`, `f64`
- [ ] Strings (4-byte length + data + null terminator)

#### 1.3 Compound Type Support
- [ ] Fixed-size arrays
- [ ] Sequences (variable-length arrays with 4-byte count prefix)
- [ ] Nested structs

#### 1.4 `no_std` Variants
- [ ] `heapless::String<N>` serialization
- [ ] `heapless::Vec<T, N>` serialization
- [ ] `alloc::String` / `alloc::Vec` (behind `alloc` feature)

### 2. Proc Macros (`nano-ros-macros`)

#### 2.1 `#[derive(RosMessage)]`
- [ ] Parse struct fields and types
- [ ] Generate `Serialize` impl
- [ ] Generate `Deserialize` impl
- [ ] Generate `TYPE_NAME` constant
- [ ] Generate `RIHS_HASH` constant (or accept as attribute)

#### 2.2 Attributes
- [ ] `#[ros(type_name = "...")]` - full ROS type name
- [ ] `#[ros(hash = "...")]` - RIHS hash override
- [ ] `#[ros(skip)]` - skip field (if needed)

#### 2.3 `#[derive(RosService)]`
- [ ] Generate request/reply type associations
- [ ] Service type name and hash

### 3. Core Types (`nano-ros-core`)

#### 3.1 Traits
- [ ] `RosMessage` trait with `TYPE_NAME`, `RIHS_HASH`
- [ ] `RosService` trait linking request/reply types
- [ ] Re-export `Serialize` / `Deserialize` from serdes

#### 3.2 Error Types
- [ ] `SerError` - serialization errors
- [ ] `DeserError` - deserialization errors
- [ ] `Error` - unified error type

#### 3.3 Time Types
- [ ] `Time` struct (sec: i32, nanosec: u32)
- [ ] `Duration` struct (sec: i32, nanosec: u32)

### 4. Standard Message Types (`nano-ros-types`)

#### 4.1 `builtin_interfaces`
- [ ] `Time`
- [ ] `Duration`

#### 4.2 `std_msgs`
- [ ] `Header`
- [ ] `String`
- [ ] `Bool`, `Byte`, `Char`
- [ ] `Int8`, `Int16`, `Int32`, `Int64`
- [ ] `UInt8`, `UInt16`, `UInt32`, `UInt64`
- [ ] `Float32`, `Float64`
- [ ] `Empty`

#### 4.3 `geometry_msgs` (minimal set)
- [ ] `Point`, `Point32`
- [ ] `Vector3`
- [ ] `Quaternion`
- [ ] `Pose`, `PoseStamped`
- [ ] `Twist`, `TwistStamped`

### 5. Testing & Validation

#### 5.1 Unit Tests (`nano-ros-serdes`)
- [ ] Round-trip serialization for all primitive types
- [ ] Round-trip for `heapless::String<N>`
- [ ] Round-trip for `heapless::Vec<T, N>`
- [ ] Round-trip for fixed-size arrays
- [ ] Round-trip for sequences
- [ ] Round-trip for nested structs
- [ ] Alignment verification (2, 4, 8 byte boundaries)
- [ ] Encapsulation header parsing
- [ ] Buffer overflow returns error (not panic)
- [ ] Partial read returns error
- [ ] Empty sequence/string handling

#### 5.2 Unit Tests (`nano-ros-macros`)
- [ ] Derive on simple struct (single field)
- [ ] Derive on struct with multiple fields
- [ ] Derive on struct with arrays
- [ ] Derive on struct with sequences
- [ ] Derive on nested structs
- [ ] `#[ros(type_name)]` attribute parsing
- [ ] `#[ros(hash)]` attribute parsing
- [ ] Compile-fail tests for invalid usage

#### 5.3 Unit Tests (`nano-ros-types`)
- [ ] `Time` serialization matches ROS 2
- [ ] `Duration` serialization matches ROS 2
- [ ] `Header` serialization matches ROS 2
- [ ] `std_msgs::String` matches ROS 2
- [ ] All primitive wrappers (`Int32`, etc.) match ROS 2

#### 5.4 Compatibility Tests
- [ ] Serialize with nano-ros, deserialize with Micro-CDR (C)
- [ ] Serialize with Micro-CDR (C), deserialize with nano-ros
- [ ] Capture ROS 2 message bytes via `ros2 topic echo --raw`
- [ ] Verify nano-ros output matches captured bytes
- [ ] Test against rmw_zenoh message format

#### 5.5 `no_std` Build Checks
- [ ] `cargo check --no-default-features -p nano-ros-serdes`
- [ ] `cargo check --no-default-features -p nano-ros-core`
- [ ] `cargo check --no-default-features -p nano-ros-types`
- [ ] `cargo check --no-default-features --target thumbv7em-none-eabihf`
- [ ] `cargo check --no-default-features --features alloc`
- [ ] Verify no `std::` imports without feature gate

#### 5.6 CI Checks
- [ ] `cargo fmt --check`
- [ ] `cargo clippy -- -D warnings`
- [ ] `cargo test --all-features`
- [ ] `cargo doc --no-deps`
- [ ] MSRV check (rust 1.75)
- [ ] Miri for undefined behavior (where applicable)

## Acceptance Criteria

1. All primitive types serialize/deserialize correctly
2. Proc macros generate correct code for message structs
3. Standard message types match ROS 2 wire format
4. All crates compile with `#![no_std]`
5. Unit test coverage > 80% for serdes crate

## Dependencies

- `heapless` - static collections
- `byteorder` - endian handling
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
