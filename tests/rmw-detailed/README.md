# Detailed RMW Protocol Tests

Tests that verify specific aspects of the rmw_zenoh protocol implementation
in nano-ros.

## Tests

### liveliness.sh
Tests liveliness token format for ROS 2 discovery:
- Node token: `@ros2_lv/<domain>/<zid>/0/0/NN/%/%/<node>`
- Publisher token: `@ros2_lv/<domain>/<zid>/0/11/MP/%/%/<node>/<topic>/<type>/<hash>/<qos>`
- Subscriber token: `@ros2_lv/<domain>/<zid>/0/11/MS/%/%/<node>/<topic>/<type>/<hash>/<qos>`

### keyexpr.sh
Tests data key expression format:
- Format: `<domain>/<topic>/<type>/<hash>`
- Example: `0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported`
- Wildcard subscriber compatibility

### qos.sh
Tests QoS compatibility:
- BEST_EFFORT publisher → BEST_EFFORT subscriber
- QoS string format in liveliness tokens
- QoS mismatch behavior

### attachment.sh
Tests RMW attachment metadata:
- Sequence number increment
- GID consistency
- Timestamp sanity

## Usage

```bash
# Run individual tests
./tests/rmw-detailed/liveliness.sh
./tests/rmw-detailed/keyexpr.sh
./tests/rmw-detailed/qos.sh
./tests/rmw-detailed/attachment.sh

# Verbose output
./tests/rmw-detailed/liveliness.sh --verbose
```

## Requirements

- zenohd in PATH
- ROS 2 Humble with rmw_zenoh_cpp
- z_sub/z_pub (from zenoh-pico, for some tests)

## Protocol Details

### Liveliness Token Format

```
@ros2_lv/<domain>/<zid>/<nid>/<eid>/<entity_type>/<enclave>/<namespace>/<node_name>/...
```

Where:
- `domain`: ROS domain ID (default: 0)
- `zid`: Zenoh ID in LSB-first hex format
- `nid`: Node ID (always 0 for nano-ros)
- `eid`: Entity ID (always 0 or 11)
- `entity_type`: NN (node), MP (publisher), MS (subscriber)
- `enclave`: Always `%` for default
- `namespace`: Node namespace (e.g., `%` for root)

### Data Key Expression Format

```
<domain>/<topic>/<type>/<hash>
```

Where:
- `domain`: ROS domain ID
- `topic`: Topic name without leading slash
- `type`: DDS-mangled type name (e.g., `std_msgs::msg::dds_::Int32_`)
- `hash`: Type hash (`TypeHashNotSupported` for Humble, `RIHS01_<sha256>` for Iron+)

### QoS String Format

```
reliability:durability:history,depth:deadline_sec,deadline_nsec:lifespan_sec,lifespan_nsec:liveliness,liveliness_sec,liveliness_nsec
```

Example: `2:2:1,1:,:,:,,` means:
- Reliability: 2 (BEST_EFFORT)
- Durability: 2 (VOLATILE)
- History: 1 (KEEP_LAST), depth: 1

### RMW Attachment Format

33 bytes serialized using zenoh serializer:
```
┌──────────────────┬────────┬───────────────────────────────────┐
│ Field            │ Type   │ Description                       │
├──────────────────┼────────┼───────────────────────────────────┤
│ sequence_number  │ i64    │ Incrementing message counter      │
│ timestamp        │ i64    │ Nanosecond timestamp              │
│ rmw_gid_size     │ u8     │ Always 16                         │
│ rmw_gid          │ [u8;16]│ Publisher Global ID               │
└──────────────────┴────────┴───────────────────────────────────┘
```
