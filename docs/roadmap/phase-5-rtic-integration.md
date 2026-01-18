# Phase 5: RTIC Integration Roadmap

**Goal**: Enable nano-ros to run on RTIC with formal real-time guarantees and static WCET analysis support.

**Timeline**: 5 weeks
**Status**: Planning

## Overview

This phase adds RTIC (Real-Time Interrupt-driven Concurrency) support to nano-ros, enabling:
- Hardware-accelerated scheduling on ARM Cortex-M
- Compile-time deadlock-free guarantees via SRP
- WCET analysis with RAUK/Symex tools
- Zero-overhead task switching

See [RTIC Integration Design](../rtic-integration-design.md) for detailed architecture.

---

## Phase 5.1: Feature Flag Infrastructure

**Duration**: 1 week
**Priority**: High

### Work Items

- [ ] **5.1.1** Add executor feature flags to `nano-ros-transport/Cargo.toml`
  ```toml
  [features]
  rtic = []
  embassy = []
  polling = []
  ```

- [ ] **5.1.2** Add synchronization feature flags
  ```toml
  [features]
  sync-spin = ["dep:spin"]           # Current default
  sync-critical-section = []          # For RTIC
  sync-rtic = ["rtic"]               # Use RTIC shared resources
  ```

- [ ] **5.1.3** Gate existing `spin::Mutex` usage behind `sync-spin` feature
  - File: `crates/nano-ros-transport/src/zenoh.rs`
  - Locations: 8 instances of `spin::Mutex`

- [ ] **5.1.4** Update CI to test all feature combinations
  ```yaml
  strategy:
    matrix:
      features:
        - "default"
        - "rtic,sync-critical-section"
        - "embassy"
        - "polling"
  ```

- [ ] **5.1.5** Document feature flag usage in README

### Acceptance Criteria
- All existing tests pass with `default` features
- `cargo check --features rtic` compiles (may not link)
- No breaking changes to existing API

---

## Phase 5.2: Replace spin::Mutex

**Duration**: 1 week
**Priority**: High
**Depends on**: 5.1

### Work Items

- [ ] **5.2.1** Implement critical section alternative for `RmwAttachment`
  ```rust
  #[cfg(feature = "sync-critical-section")]
  mod attachment {
      use cortex_m::interrupt;

      pub fn with_attachment<F, R>(attachment: &mut RmwAttachment, f: F) -> R
      where F: FnOnce(&mut RmwAttachment) -> R {
          interrupt::free(|_| f(attachment))
      }
  }
  ```

- [ ] **5.2.2** Refactor `ZenohPublisher` to use trait-based mutex
  - Create `SyncPrimitive` trait
  - Implement for `spin::Mutex` and critical sections
  - Gate implementations behind features

- [ ] **5.2.3** Refactor `SubscriberBuffer` to use trait-based mutex
  - File: `crates/nano-ros-transport/src/zenoh.rs:350`

- [ ] **5.2.4** Refactor `ServiceServerBuffer` to use trait-based mutex
  - File: `crates/nano-ros-transport/src/zenoh.rs:453`

- [ ] **5.2.5** Add lock-free alternative using pure atomics
  ```rust
  #[cfg(feature = "sync-lockfree")]
  pub struct RmwAttachment {
      sequence_number: AtomicI64,
      timestamp: AtomicI64,
      gid: AtomicGid,  // Custom atomic wrapper
  }
  ```

- [ ] **5.2.6** Test critical section implementation on QEMU Cortex-M
  ```bash
  cargo run --example rtic-test --target thumbv7em-none-eabihf
  ```

### Acceptance Criteria
- `spin::Mutex` only used when `sync-spin` feature enabled
- Critical sections work on Cortex-M
- No deadlocks detected in testing
- Performance regression < 5%

---

## Phase 5.3: Zenoh Background Task Removal

**Duration**: 1 week
**Priority**: High
**Depends on**: 5.2

### Work Items

- [ ] **5.3.1** Add `new_without_tasks()` constructor to `ZenohSession`
  ```rust
  impl ZenohSession {
      #[cfg(feature = "rtic")]
      pub fn new_without_tasks(config: Config) -> Result<Self, Error> {
          // Initialize but don't call zp_start_*_task
      }
  }
  ```

- [ ] **5.3.2** Expose `poll_read()` method
  ```rust
  pub fn poll_read(&mut self) -> Result<(), Error> {
      unsafe { zp_read(self.session, ptr::null()); }
      Ok(())
  }
  ```

- [ ] **5.3.3** Expose `send_keepalive()` method
  ```rust
  pub fn send_keepalive(&mut self) -> Result<(), Error> {
      unsafe { zp_send_keep_alive(self.session, ptr::null()); }
      Ok(())
  }
  ```

- [ ] **5.3.4** Create `rtic_tasks!` macro
  ```rust
  #[macro_export]
  macro_rules! rtic_tasks {
      ($mono:ty, $session:ident) => {
          #[task(priority = 1, shared = [$session])]
          async fn nano_ros_poll(mut cx: nano_ros_poll::Context) { ... }

          #[task(priority = 1, shared = [$session])]
          async fn nano_ros_keepalive(mut cx: nano_ros_keepalive::Context) { ... }
      };
  }
  ```

- [ ] **5.3.5** Add Embassy spawn helper function
  ```rust
  #[cfg(feature = "embassy")]
  pub fn spawn_nano_ros_tasks(
      spawner: &Spawner,
      session: &'static SharedSession,
  ) -> Result<(), SpawnError>
  ```

- [ ] **5.3.6** Test polling mode without background threads
  - Verify messages received with manual polling
  - Verify session stays alive with manual keepalive

### Acceptance Criteria
- RTIC mode doesn't spawn any OS threads
- Manual polling receives messages correctly
- Session keepalive works with manual calls
- Embassy helper spawns tasks correctly

---

## Phase 5.4: Static Buffer Allocation

**Duration**: 1 week
**Priority**: Medium
**Depends on**: 5.3

### Work Items

- [ ] **5.4.1** Replace `Vec<LivelinessToken>` with `heapless::Vec`
  - File: `crates/nano-ros-node/src/connected.rs`
  ```rust
  pub struct ConnectedNode<const MAX_TOKENS: usize = 16> {
      _entity_tokens: heapless::Vec<LivelinessToken, MAX_TOKENS>,
  }
  ```

- [ ] **5.4.2** Add const generics for subscriber buffer sizes
  ```rust
  pub struct SubscriberBuffer<const SIZE: usize = 1024> {
      data: [u8; SIZE],
      len: AtomicUsize,
      has_data: AtomicBool,
  }
  ```

- [ ] **5.4.3** Add const generics for service buffer sizes
  ```rust
  pub struct ServiceServerBuffer<const SIZE: usize = 1024> {
      data: [u8; SIZE],
      reply_keyexpr: heapless::String<256>,
      // ...
  }
  ```

- [ ] **5.4.4** Document memory requirements
  - Create `docs/memory-requirements.md`
  - Include typical sizes for different use cases
  - Add calculation examples

- [ ] **5.4.5** Add `#[cfg(feature = "alloc")]` guards
  - Ensure core functionality works without alloc
  - Gate heap-using features properly

- [ ] **5.4.6** Test `no_std + no_alloc` build
  ```bash
  cargo build --no-default-features --features "rtic,sync-critical-section"
  ```

### Acceptance Criteria
- Builds with `--no-default-features --features rtic`
- No heap allocations after `init()` completes
- Memory usage documented
- Const generics provide flexibility

---

## Phase 5.5: Examples and Documentation

**Duration**: 1 week
**Priority**: Medium
**Depends on**: 5.4

### Work Items

- [ ] **5.5.1** Create RTIC example for STM32F4
  - File: `examples/rtic-stm32f4/`
  - Hardware: NUCLEO-F429ZI or similar
  - Features: IMU reading, ROS 2 publish, UART debug

- [ ] **5.5.2** Create Embassy example for STM32F4
  - File: `examples/embassy-stm32f4/`
  - Same hardware as RTIC example
  - Demonstrate Embassy-style async

- [ ] **5.5.3** Create polling example (no executor)
  - File: `examples/polling-cortex-m/`
  - Simple main loop with manual polling
  - Useful for very constrained systems

- [ ] **5.5.4** Document WCET analysis workflow
  - Install RAUK
  - Run analysis on example
  - Interpret results
  - Add to `docs/wcet-analysis.md`

- [ ] **5.5.5** Add integration tests
  - Test RTIC on QEMU
  - Test Embassy on QEMU
  - Test ROS 2 interop (native + QEMU)

- [ ] **5.5.6** Update CLAUDE.md with RTIC section
  - Add RTIC integration status
  - Document feature flags
  - Add build commands

- [ ] **5.5.7** Create example Cargo.toml templates
  - RTIC project template
  - Embassy project template
  - Document required dependencies

### Acceptance Criteria
- RTIC example builds and runs on hardware
- Embassy example builds and runs on hardware
- WCET analysis documented with examples
- All examples tested in CI

---

## Dependencies

### External Crates

```toml
# RTIC (when rtic feature enabled)
rtic = { version = "2", optional = true }
rtic-monotonics = { version = "2", optional = true }
rtic-sync = { version = "1", optional = true }

# Embassy (when embassy feature enabled)
embassy-executor = { version = "0.6", optional = true }
embassy-time = { version = "0.3", optional = true }
embassy-sync = { version = "0.6", optional = true }

# Cortex-M support
cortex-m = { version = "0.7", optional = true }
cortex-m-rt = { version = "0.7", optional = true }
```

### Reference Repositories

Cloned to `external/`:
- `external/rtic/` - RTIC framework (monorepo)
- `external/embassy/` - Embassy framework

---

## Success Metrics

| Metric | Target |
|--------|--------|
| RTIC build time | < 30s incremental |
| Memory overhead | < 1KB vs bare metal |
| Task switch latency | < 1μs (hardware) |
| WCET analysis coverage | > 90% of tasks |
| Test coverage | > 80% |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| zenoh-pico incompatible with single-thread | Low | High | Verify polling API works |
| RTIC API changes | Low | Medium | Pin to stable version |
| Embassy compatibility issues | Medium | Low | Keep as secondary target |
| WCET tools outdated | Medium | Medium | Document manual analysis |

---

## Open Questions

1. **Zenoh session lifetime**: How to manage session in RTIC `#[shared]` vs `#[local]`?
   - Recommendation: `#[shared]` for session, `#[local]` for publishers/subscribers

2. **Multi-publisher pattern**: Should each publisher have own buffer or share?
   - Recommendation: Each publisher owns its buffer, allocated at init

3. **QoS mapping**: How to map ROS 2 QoS to RTIC priorities?
   - Recommendation: Document guidelines, let user decide

4. **Error handling**: What to do on publish failure in real-time task?
   - Recommendation: Return error, let user decide (log/retry/ignore)

---

## References

- [RTIC Integration Design](../rtic-integration-design.md)
- [nano-ros Realtime Design](../nano-ros-realtime-design.md)
- [RTIC Book](https://rtic.rs/2/book/en/)
- [Embassy Documentation](https://embassy.dev/)
