# Memory Requirements

This document describes the memory requirements for nano-ros on embedded systems.

## Overview

nano-ros uses static allocation wherever possible to enable deterministic memory usage
and compatibility with systems that lack a heap allocator.

## Configurable Buffer Sizes

All buffer sizes are configurable via const generics with sensible defaults:

| Type                     | Parameter    | Default | Description                                          |
|--------------------------|--------------|---------|------------------------------------------------------|
| `ConnectedNode`          | `MAX_TOKENS` | 16      | Maximum liveliness tokens (publishers + subscribers) |
| `ConnectedSubscriber`    | `RX_BUF`     | 1024    | Receive buffer size in bytes                         |
| `ConnectedServiceServer` | `REQ_BUF`    | 1024    | Request buffer size in bytes                         |
| `ConnectedServiceServer` | `REPLY_BUF`  | 1024    | Reply buffer size in bytes                           |
| `ConnectedServiceClient` | `REQ_BUF`    | 1024    | Request buffer size in bytes                         |
| `ConnectedServiceClient` | `REPLY_BUF`  | 1024    | Reply buffer size in bytes                           |

## Memory Usage Calculation

### ConnectedNode

```
ConnectedNode<MAX_TOKENS> memory usage:
  - name: 64 bytes (heapless::String<64>)
  - namespace: 64 bytes (heapless::String<64>)
  - domain_id: 4 bytes
  - session: ~200 bytes (ZenohSession overhead)
  - zid: 16 bytes (ZenohId)
  - node_token: ~48 bytes (Option<LivelinessToken>)
  - entity_tokens: MAX_TOKENS * 48 bytes (heapless::Vec<LivelinessToken>)

Total: ~400 + (MAX_TOKENS * 48) bytes

Example with MAX_TOKENS=16: ~1,168 bytes
Example with MAX_TOKENS=4: ~592 bytes
```

### ConnectedSubscriber

```
ConnectedSubscriber<M, RX_BUF> memory usage:
  - subscriber: ~64 bytes (ZenohSubscriber handle)
  - rx_buffer: RX_BUF bytes
  - _marker: 0 bytes (PhantomData)

Total: ~64 + RX_BUF bytes

Example with RX_BUF=1024: ~1,088 bytes
Example with RX_BUF=256: ~320 bytes
```

### ConnectedPublisher

```
ConnectedPublisher<M> memory usage:
  - publisher: ~64 bytes (ZenohPublisher handle)
  - _marker: 0 bytes (PhantomData)

Total: ~64 bytes
```

Note: Publishers use a temporary 1024-byte stack buffer during `publish()`.
Use `publish_with_buffer()` to provide a custom buffer.

### ConnectedServiceServer

```
ConnectedServiceServer<S, REQ_BUF, REPLY_BUF> memory usage:
  - server: ~64 bytes (ZenohServiceServer handle)
  - req_buffer: REQ_BUF bytes
  - reply_buffer: REPLY_BUF bytes
  - _marker: 0 bytes (PhantomData)

Total: ~64 + REQ_BUF + REPLY_BUF bytes

Example with defaults (1024, 1024): ~2,112 bytes
Example with (512, 512): ~1,088 bytes
```

### ConnectedServiceClient

```
ConnectedServiceClient<S, REQ_BUF, REPLY_BUF> memory usage:
  - client: ~64 bytes (ZenohServiceClient handle)
  - req_buffer: REQ_BUF bytes
  - reply_buffer: REPLY_BUF bytes
  - _marker: 0 bytes (PhantomData)

Total: ~64 + REQ_BUF + REPLY_BUF bytes

Example with defaults (1024, 1024): ~2,112 bytes
Example with (512, 512): ~1,088 bytes
```

## Example Configurations

### Minimal Configuration (2 publishers, 2 subscribers)

```rust
use nano_ros_node::{ConnectedNode, NodeConfig};

// Minimal node with 4 entity tokens
type MinimalNode = ConnectedNode<4>;

// Subscribers with 256-byte buffers
type SmallSubscriber<M> = ConnectedSubscriber<M, 256>;
```

**Estimated memory:**
- Node: ~592 bytes
- 2 publishers: ~128 bytes
- 2 subscribers (256B buffers): ~640 bytes
- **Total: ~1,360 bytes**

### Standard Configuration (4 publishers, 4 subscribers)

Using defaults:

```rust
use nano_ros_node::{ConnectedNode, ConnectedSubscriber, NodeConfig};

// Default: 16 entity tokens, 1024-byte buffers
let node = ConnectedNode::connect(config, locator)?;
let sub = node.create_subscriber::<MyMsg>("/topic")?;
```

**Estimated memory:**
- Node: ~1,168 bytes
- 4 publishers: ~256 bytes
- 4 subscribers (1KB buffers): ~4,352 bytes
- **Total: ~5,776 bytes**

### Large Message Configuration

```rust
use nano_ros_node::{ConnectedNode, NodeConfig};

// Node with room for 8 entities
type LargeNode = ConnectedNode<8>;

// Create subscriber with 8KB buffer for large messages
let sub = node.create_subscriber_sized::<LargeMsg, 8192>("/large_topic", qos)?;

// Create service with 4KB request/reply buffers
let server = node.create_service_sized::<LargeService, 4096, 4096>("/service")?;
```

## Transport Layer Memory

The zenoh transport layer currently requires `alloc` for:

- Subscriber callback buffers (`Arc<SubscriberBuffer>`)
- Service server callback buffers (`Arc<ServiceServerBuffer>`)
- Dynamic strings for keyexpr storage

Future versions may provide a fully static alternative for no-alloc environments.

## Stack Usage

Consider stack usage for local buffers:

```rust
// publish() uses 1024 bytes of stack
publisher.publish(&msg)?;

// Use publish_with_buffer() for custom stack usage
let mut buf = [0u8; 512];
publisher.publish_with_buffer(&msg, &mut buf)?;
```

## Recommendations

1. **Start small**: Begin with minimal buffer sizes and increase as needed.

2. **Profile actual usage**: Monitor actual message sizes in your application.

3. **Consider worst case**: Size buffers for the largest expected message.

4. **Use sized constructors**: For embedded systems, always specify buffer sizes:
   ```rust
   // Explicit sizing for embedded
   node.create_subscriber_sized::<Msg, 512>("/topic", qos)?;
   ```

5. **Account for CDR overhead**: Messages have a 4-byte CDR header plus alignment padding.

## Feature Flags for Memory Optimization

| Feature                 | Effect                                       |
|-------------------------|----------------------------------------------|
| `--no-default-features` | Disables std, reduces code size              |
| `rtic`                  | Enables RTIC support, no background threads  |
| `sync-critical-section` | Uses critical sections instead of spin locks |

## See Also

- [RTIC Integration Design](./rtic-integration-design.md)
- [Phase 5 Roadmap](./roadmap/phase-5-rtic-integration.md)
