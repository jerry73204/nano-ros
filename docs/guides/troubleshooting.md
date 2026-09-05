# Troubleshooting

Common issues and solutions when working with nros.

## zenoh-pico Multiple Client Issues

### Symptoms

When running multiple zenoh-pico clients (e.g., a talker and listener) simultaneously connecting to the same router, you may see errors like:

```
<err> nros_bsp: Failed to open zenoh session: -3
<err> rust: rustapp: BSP init failed: InitFailed(-5)
```

Or publisher/subscriber declarations fail with:
- Error code `-128` (`_Z_ERR_GENERIC`)
- Error code `-78` (`_Z_ERR_SYSTEM_OUT_OF_MEMORY`)

The first client typically succeeds while subsequent clients fail during `z_declare_publisher()` or `z_declare_subscriber()`.

### Root Cause

This is caused by the **Z_FEATURE_INTEREST** feature in zenoh-pico. The interest protocol implements write filtering to optimize network traffic by only sending data to interested subscribers. However, this feature has issues when multiple clients connect to the same router:

1. The router tracks "interest" for each client
2. When creating a publisher, zenoh-pico creates a write filter context
3. The filter creation calls `_z_session_rc_clone_as_weak()` which can fail
4. This manifests as `_Z_ERR_SYSTEM_OUT_OF_MEMORY` (-78) even when memory is available

The failure occurs in `zenoh-pico/src/net/filtering.c`:
```c
ctx->zn = _z_session_rc_clone_as_weak(zn);
if (_Z_RC_IS_NULL(&ctx->zn)) {
    _z_write_filter_ctx_clear(ctx);
    z_free(ctx);
    _Z_ERROR_RETURN(_Z_ERR_SYSTEM_OUT_OF_MEMORY);
}
```

### Solution

nros disables `Z_FEATURE_INTEREST` by default for all builds:

**Native builds** (build.rs):
```rust
let dst = cmake::Config::new(&zenoh_pico_build)
    // ...
    .define("Z_FEATURE_INTEREST", "0")
    .build();
```

**Zephyr builds**: The `just setup zephyr` recipe automatically patches zenoh-pico's `config.h` to disable these features. If you set up the workspace manually or the patch wasn't applied, run:
```bash
just setup zephyr  # Updates and patches existing workspace
```

Or manually edit `modules/lib/zenoh-pico/include/zenoh-pico/config.h`:
```c
// Change from:
#define Z_FEATURE_INTEREST 1
#define Z_FEATURE_MATCHING 1

// To:
#define Z_FEATURE_INTEREST 0
#define Z_FEATURE_MATCHING 0
```

Then rebuild with `--pristine`:
```bash
west build -b native_sim/native/64 nros/examples/zephyr/rust/talker -d build-talker --pristine \
  -- -DEXTRA_CONF_FILE=prj-zenoh.conf
```

Note: `Z_FEATURE_MATCHING` depends on `Z_FEATURE_INTEREST`, so both must be disabled.

### References

- [rmw_zenoh_pico](https://github.com/micro-ROS/rmw_zenoh_pico) disables this feature by default
- zenoh-pico filtering code: `src/net/filtering.c`
- zenoh-pico interest protocol: `src/session/interest.c`

---

## zenoh-pico Version Compatibility

### Symptoms

Publisher works but `z_publisher_put` fails immediately:
```
zenoh_shim: z_publisher_put failed: -100
<err> rust: rustapp: Publish failed: PublishFailed(-1)
zenoh_shim: z_publisher_put failed: -73
```

Error codes:
- `-100`: `_Z_ERR_TRANSPORT_TX_FAILED` - Transport transmission failed
- `-73`: `_Z_ERR_SESSION_CLOSED` - Session closed after first failure

### Root Cause

Version mismatch between zenoh-pico and zenohd:
- **zenoh-pico**: 1.5.1 (in west.yml)
- **zenohd**: 1.7.x (installed via cargo)

The zenoh protocol may have changed, causing transport-level incompatibilities.

### Solution

Upgrade zenoh-pico to match zenohd version. See [docs/roadmap/phase-15-zenoh-pico-upgrade.md](../roadmap/archived/phase-15-zenoh-pico-upgrade.md) for the upgrade plan.

**Temporary workaround**: Install an older zenohd version:
```bash
cargo install zenoh --version 1.5.1 --features=zenohd
```

---

## Multiple Zephyr Instance Issues

### Ephemeral Port Conflict

**Symptom**: When running multiple Zephyr native_sim instances (e.g., talker and listener), both sessions fail to establish or one gets `-100` (`_Z_ERR_TRANSPORT_TX_FAILED`) errors.

**Root Cause**: Zephyr native_sim uses a test entropy source that produces identical random number sequences unless seeded differently. This causes both instances to select the same ephemeral TCP port when connecting to the router, causing a port conflict.

**Solution**: Pass different `--seed` values to each native_sim instance:
```bash
# Start listener with one seed
./build-listener/zephyr/zephyr.exe --seed=12345

# Start talker with a different seed
./build-talker/zephyr/zephyr.exe --seed=67890
```

The `--seed` parameter initializes the test entropy source with a different value, producing different random numbers and thus different ephemeral ports.

**For automated tests**: The test framework (`packages/testing/nros-tests`) automatically assigns unique seeds to each process.

---

## Network Configuration Issues

### TAP Interface Setup (QEMU bare-metal only)

**Symptom**: QEMU cannot connect to the network.

**Solution**: Run the network setup script:
```bash
sudo ./scripts/qemu/setup-network.sh
```

This creates and configures the `tap0` interface at `192.0.3.1`. QEMU
bare-metal guests live on `192.0.3.10+`.

### Zephyr native_sim networking

Zephyr `native_sim` uses **NSOS** (Native Sim Offloaded Sockets) — socket
calls are forwarded to host syscalls, so there is no emulated L2 stack and
no TAP bridge. Point each example at a host-loopback zenohd / XRCE Agent:

```bash
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7456"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd  # or any host-accessible address
```

Multiple `native_sim` instances can coexist without bridge configuration.

---

## Message Too Large / Truncated Messages

### Symptoms

- `TransportError::MessageTooLarge` when receiving service requests or subscription data
- Messages arrive but with corrupted or incomplete payloads
- Large messages (images, point clouds) silently disappear

### Root Cause

nros uses static buffers at multiple layers, each with configurable size limits.
A message must fit through every layer in the path to be delivered intact.

**Zenoh backend layers:**

| Layer                               | Posix Default | Embedded Default | Env Var                    |
|-------------------------------------|---------------|------------------|----------------------------|
| Defragmentation (`Z_FRAG_MAX_SIZE`) | 65536         | 2048             | `ZPICO_FRAG_MAX_SIZE`      |
| Batch size (`Z_BATCH_UNICAST_SIZE`) | 65536         | 1024             | `ZPICO_BATCH_UNICAST_SIZE` |
| Per-entity shim buffer              | 1024          | 1024             | — (named constant in code) |
| User receive buffer (`RX_BUF`)      | 1024          | 1024             | — (const generic)          |

**XRCE-DDS backend layers:**

| Layer             | Posix Default | Embedded Default | Env Var                    |
|-------------------|---------------|------------------|----------------------------|
| Transport MTU     | 4096          | 512              | `XRCE_TRANSPORT_MTU`       |
| Per-entity buffer | 1024          | 1024             | — (named constant in code) |

### Solutions

**Increase transport-level limits** (set before `cargo build`):

```bash
# Zenoh: allow 128 KB reassembled messages
ZPICO_FRAG_MAX_SIZE=131072 cargo build --features rmw-zenoh,platform-posix

# XRCE: increase MTU to 8 KB
XRCE_TRANSPORT_MTU=8192 cargo build --features rmw-xrce,platform-posix
```

**Increase per-entity buffer sizes** (in code):

```rust
// Zenoh subscriber with 4 KB receive buffer
let sub = node.create_subscriber_sized::<MyMsg, 4096>(SubscriberOptions::new("/topic"))?;

// Zenoh publisher with 4 KB transmit buffer
let pub_ = node.create_publisher_sized::<MyMsg, 4096>(PublisherOptions::new("/topic"))?;
```

**Clean rebuild** after changing env vars (CMake caches old values):

```bash
cargo clean -p zpico-sys   # or: cargo clean -p xrce-sys
cargo build
```

---

## Build Issues

### zenoh-pico Submodule Not Found

**Symptom**:
```
zenoh-pico submodule not found at /path/to/zenoh-pico. Run: git submodule update --init
```

**Solution**:
```bash
git submodule update --init --recursive
```

### CMake Cache Stale

**Symptom**: Changes to CMake defines (like `Z_FEATURE_INTEREST`) don't take effect.

**Solution**: Clean the build cache:
```bash
# Clean the zenoh-pico sys crate build cache
cargo clean -p zpico-sys
touch packages/rmw/zenoh/zpico-sys/build.rs
cargo build
```

For Zephyr builds, clean the west build directory:
```bash
rm -rf build/
west build -b native_sim/native/64 <app>
```

---

## Test Failures

### Tests Hang Forever

**Symptom**: Integration tests hang indefinitely.

**Possible causes**:
1. **No zenohd router running**: Many tests require a zenohd router
2. **Port already in use**: Another process is using port 7447
3. **Network misconfigured**: TAP or bridge interfaces not set up

**Solutions**:
```bash
# Check if zenohd is running
pgrep zenohd

# Check if port 7447 is in use
ss -tlnp | grep 7447

# Verify QEMU bare-metal TAP (only needed for QEMU bare-metal platforms)
ip addr show tap0
```

### Zephyr Tests Skip

**Symptom**: All Zephyr tests are skipped.

**Cause**: The test framework couldn't find the Zephyr workspace or required components.

**Solution**: Ensure the Zephyr workspace is properly configured:
```bash
# The workspace should be at ../nano-ros-workspace relative to nros
ls -la zephyr-workspace  # Should be a symlink

# Or set explicitly
export ZEPHYR_WORKSPACE=/path/to/nano-ros-workspace

# Verify west is available
west --version
```

---

## Rust/C FFI Issues

### Subscriber Callback Crashes

**Symptom**: When a Zephyr listener receives a message, the application crashes with a segfault or produces garbage values when accessing struct fields. Debug output may show valid pointers but invalid field values:
```
bsp_zephyr: sub->callback=0x60, sub->user_data=0x4  # These should be valid addresses!
```

**Root Cause**: The C BSP stores a pointer to the `nros_subscriber_t` struct when you call `nros_bsp_create_subscriber()`. If the Rust code moves the struct after this call (e.g., when returning it from a function), the C code's pointer becomes dangling.

In Rust, values are moved by default:
```rust
// WRONG - struct will move when returned
pub fn create_subscriber(...) -> Result<BspSubscriber<M>, Error> {
    let mut sub = NanoRosSubscriber { ... };
    nros_bsp_create_subscriber(&mut sub, ...);  // C stores pointer to `sub`
    Ok(BspSubscriber { sub, ... })  // `sub` MOVES here - old address is invalid!
}
```

**Solution**: Use static storage or ensure the struct has a stable address before passing it to C:

```rust
use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

// Wrapper to make static storage Sync
struct StaticSubscriber(MaybeUninit<NanoRosSubscriber>);
unsafe impl Sync for StaticSubscriber {}

static mut SUBSCRIBER_STORAGE: StaticSubscriber =
    StaticSubscriber(MaybeUninit::uninit());

fn main() {
    let sub_ptr = unsafe {
        let storage_ptr = addr_of_mut!(SUBSCRIBER_STORAGE);
        let sub = (*storage_ptr).0.as_mut_ptr();
        // Initialize fields...
        sub
    };

    // Now the pointer is stable
    nros_bsp_create_subscriber(sub_ptr, ...);
}
```

Alternative approaches:
- Use `Box::new()` (requires `alloc`) for heap allocation with stable address
- Use `Pin` to prevent moves
- Have the C code use indices instead of pointers

**Why this happens**: Unlike C where variables have fixed addresses, Rust moves values during assignment and return. When the C code stores a pointer during initialization, it doesn't know the Rust struct will be moved later.

### Function Pointer ABI Mismatch

**Symptom**: Callbacks passed between Rust and C crash or receive garbage arguments.

**Solution**: Ensure function pointers use `extern "C"`:
```rust
// CORRECT - uses C calling convention
extern "C" fn my_callback(data: *const u8, len: usize, ctx: *mut c_void) {
    // ...
}

// WRONG - uses Rust calling convention (incompatible with C)
fn my_callback(data: *const u8, len: usize, ctx: *mut c_void) {
    // ...
}
```

---

## zenoh-pico Error Codes

| Code | Name                           | Description                                      |
|------|--------------------------------|--------------------------------------------------|
| -3   | `_Z_ERR_TRANSPORT_OPEN_FAILED` | Could not connect to router                      |
| -78  | `_Z_ERR_SYSTEM_OUT_OF_MEMORY`  | Memory allocation failed (or write filter issue) |
| -128 | `_Z_ERR_GENERIC`               | Generic error                                    |

For the complete list, see `zenoh-pico/include/zenoh-pico/utils/result.h`.
