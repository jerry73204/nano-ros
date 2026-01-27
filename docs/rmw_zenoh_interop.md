# ROS 2 rmw_zenoh Interoperability

This document describes how nano-ros communicates with standard ROS 2 nodes using `rmw_zenoh_cpp`.

## Status: WORKING

nano-ros ↔ ROS 2 rmw_zenoh communication is fully operational as of January 2025.

## Architecture

```
┌─────────────────────────┐          ┌─────────────────────────┐
│   ROS 2 Node            │          │   nano-ros Node         │
│   (rmw_zenoh_cpp)       │◄────────►│   (zenoh-pico)          │
│                         │  Zenoh   │                         │
│   ros2 topic echo       │  Router  │   native-rs-talker         │
│   /chatter              │ (zenohd) │                         │
└─────────────────────────┘          └─────────────────────────┘
```

Both nodes connect to the same Zenoh router (zenohd) or communicate directly in peer mode.

## Quick Start

```bash
# Terminal 1: Start zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: Run nano-ros talker
cargo run -p native-rs-talker --features zenoh -- --tcp 127.0.0.1:7447

# Terminal 3: Run ROS 2 listener
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

## Protocol Requirements

### 1. Data Topic Key Expression Format

For ROS 2 Humble, the data key expression format is:

```
<domain_id>/<topic_name>/<type_name>/<type_hash>
```

**Important:** For Humble, use `TypeHashNotSupported` as the type hash (no `RIHS01_` prefix).

Example:
```
0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
```

**Note:** Newer ROS 2 versions (Iron+) use `RIHS01_<hash>` format with actual type hashes.

Implementation: `TopicInfo::to_key()` in `nano-ros-transport/src/traits.rs`

### 2. Liveliness Token Format

rmw_zenoh uses Zenoh liveliness tokens for discovery. Without these, `ros2 topic list` won't show nano-ros topics.

**Node Liveliness Token:**
```
@ros2_lv/<domain>/<zid>/0/0/NN/%/%/<node_name>
```

**Publisher Liveliness Token:**
```
@ros2_lv/<domain>/<zid>/0/11/MP/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>
```

**Subscriber Liveliness Token:**
```
@ros2_lv/<domain>/<zid>/0/11/MS/%/%/<node_name>/<topic>/<type>/RIHS01_<hash>/<qos>
```

Key differences from data keyexprs:
- Liveliness tokens **do** use `RIHS01_` prefix
- Topic names use `%` instead of `/` (e.g., `%chatter` not `/chatter`)
- ZenohId must be in LSB-first hex format

Implementation: `Ros2Liveliness` in `nano-ros-transport/src/zenoh.rs`

### 3. QoS String Format

The QoS portion of liveliness tokens uses this format:
```
reliability:durability:history,depth:deadline_sec,deadline_nsec:lifespan_sec,lifespan_nsec:liveliness,liveliness_sec,liveliness_nsec
```

Values:
- Reliability: 1=RELIABLE, 2=BEST_EFFORT
- Durability: 1=TRANSIENT_LOCAL, 2=VOLATILE
- History: 1=KEEP_LAST

For BEST_EFFORT/VOLATILE with KEEP_LAST depth 1:
```
2:2:1,1:,:,:,,
```

**Important:** Empty values default to RELIABLE which causes QoS mismatch. Always specify explicit values.

### 4. CDR Message Format

Messages use CDR (Common Data Representation) little-endian encoding:
```
[0x00, 0x01, 0x00, 0x00]  // CDR encapsulation header (LE)
[... CDR payload ...]     // Message data
```

Implementation: `nano-ros-serdes` crate

### 5. RMW Attachment Format

rmw_zenoh requires metadata attached to each published message. For Humble, the serialization format is:

| Field | Size | Format |
|-------|------|--------|
| sequence_number | 8 bytes | int64, little-endian |
| timestamp | 8 bytes | int64, little-endian (nanoseconds) |
| gid_length | 1 byte | VLE encoded (value: 16) |
| gid | 16 bytes | Random bytes, constant per publisher |

Total: 33 bytes

The serialization uses Zenoh's `ze_serializer` to ensure compatibility with zenoh-cpp's deserializer.

Implementation: `serialize_rmw_attachment()` in `zenoh-pico/src/serializer.rs`

### 6. ZenohId Format

The ZenohId in liveliness tokens must be formatted in LSB-first (little-endian) hex format.

```rust
// Correct: LSB-first
fn to_hex_string(&self) -> String {
    let mut hex = String::new();
    for byte in self.bytes.iter() {  // LSB first
        write!(&mut hex, "{:02x}", byte).ok();
    }
    hex
}
```

## Common Issues and Solutions

### Issue: Discovery works but no messages received

**Symptom:** `ros2 topic list` shows the topic, `ros2 topic info` shows correct QoS, but `ros2 topic echo` receives nothing.

**Cause:** Data keyexpr format mismatch.

**Solution:** For Humble, don't use `RIHS01_` prefix in data keyexprs:
- Wrong: `0/chatter/std_msgs::msg::dds_::Int32_/RIHS01_TypeHashNotSupported`
- Correct: `0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported`

### Issue: Topic not visible in ros2 topic list

**Symptom:** nano-ros publishes but ROS 2 doesn't see the topic.

**Cause:** Liveliness token format incorrect.

**Solutions:**
1. Check ZenohId is LSB-first hex format
2. Ensure topic name uses `%` prefix (e.g., `%chatter`)
3. Verify QoS string has explicit values, not defaults

### Issue: QoS incompatibility warnings

**Symptom:** ROS 2 logs QoS incompatibility or subscriber doesn't receive.

**Cause:** Publisher and subscriber QoS don't match.

**Solution:** Use BEST_EFFORT for both:
```bash
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

### Issue: rmw_zenoh not connecting to router

**Symptom:** rmw_zenoh uses peer mode by default, not connecting to zenohd.

**Solution:** Force client mode:
```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
```

## Test Scripts

```bash
# Full test with Python subscriber
./scripts/test-qos.sh

# Debug with verbose zenohd logging
./scripts/test-rmw-debug.sh

# nano-ros to nano-ros (no ROS 2)
./scripts/test-pubsub.sh
```

## Implementation Status

| Component | Status | Location |
|-----------|--------|----------|
| Data keyexpr format | ✅ Done | `TopicInfo::to_key()` |
| Liveliness tokens | ✅ Done | `Ros2Liveliness` |
| QoS format | ✅ Done | `2:2:1,1:,:,:,,` |
| CDR serialization | ✅ Done | `nano-ros-serdes` |
| RMW attachment | ✅ Done | `serialize_rmw_attachment()` |
| ZenohId format | ✅ Done | `ZenohId::to_hex_string()` |
| Node integration | ✅ Done | `ConnectedNode` |

## Version Compatibility

| ROS 2 Version | Type Hash Format | Status |
|---------------|------------------|--------|
| Humble | `TypeHashNotSupported` | ✅ Working |
| Iron+ | `RIHS01_<sha256>` | ⚠️ Needs type hash computation |

## References

- [rmw_zenoh design docs](https://github.com/ros2/rmw_zenoh/blob/humble/docs/design.md)
- [Zenoh serialization RFC](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Serialization.md)
- [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)
