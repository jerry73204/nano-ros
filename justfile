default:
    @just --list

# Install toolchains and tools for development
setup:
    rustup toolchain install stable
    rustup toolchain install nightly
    rustup component add rustfmt clippy rust-src
    rustup component add --toolchain nightly rustfmt miri rust-src
    rustup target add thumbv7em-none-eabihf
    rustup target add thumbv7m-none-eabi
    cargo install cargo-nextest --locked

# Build with no_std (default) - excludes embedded-only packages
build:
    cargo build --workspace --exclude qemu-test --no-default-features

# Build for embedded target (Cortex-M4F)
build-embedded:
    cargo build --workspace --exclude qemu-test --no-default-features --target thumbv7em-none-eabihf

# Check no-alloc build works (catches accidental alloc usage)
check-no-alloc:
    @echo "Checking no-alloc build for transport layer..."
    cargo clippy -p nano-ros-transport --no-default-features --target thumbv7em-none-eabihf
    @echo "Checking no-alloc build with RTIC features..."
    cargo clippy -p nano-ros-transport --no-default-features --features "rtic,sync-critical-section" --target thumbv7em-none-eabihf
    @echo "Checking no-alloc build for node layer..."
    cargo clippy -p nano-ros-node --no-default-features --target thumbv7em-none-eabihf
    @echo "All no-alloc checks passed!"

format:
    cargo +nightly fmt

# Check with no_std (default)
check:
    cargo +nightly fmt --check
    cargo clippy --no-default-features -- -D warnings

# Test (requires std) - excludes embedded-only packages
test:
    cargo nextest run --workspace --exclude qemu-test --no-fail-fast

# Run Miri to detect undefined behavior
test-miri:
    cargo +nightly miri test -p nano-ros-serdes
    cargo +nightly miri test -p nano-ros-core
    cargo +nightly miri test -p nano-ros-types

# Run all quality checks
quality: check test

# Run full test suite including Miri
test-all: test test-miri

# Generate documentation
doc:
    cargo doc --no-deps --open

# Clean build artifacts
clean:
    cargo clean

# Build QEMU test binary for Cortex-M3
build-qemu:
    cd examples/qemu-test && cargo build --release

# Run QEMU test (Cortex-M3)
qemu-test: build-qemu
    qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -semihosting-config enable=on,target=native \
        -kernel target/thumbv7m-none-eabi/release/qemu-test

# Check if QEMU is installed
check-qemu:
    @which qemu-system-arm > /dev/null || (echo "Error: qemu-system-arm not found. Install with: sudo apt install qemu-system-arm" && exit 1)
    @echo "QEMU ARM is installed"

# Run native talker example
run-talker:
    cargo run -p native-talker

# Run native listener example
run-listener:
    cargo run -p native-listener

# Build native examples (requires std)
build-native-examples:
    cargo build -p native-talker -p native-listener

# Build all examples (native + QEMU + embedded)
build-examples: build-native-examples build-embedded-examples
    cd examples/qemu-test && cargo build --release
    @echo "All examples built!"

# Run all tests including QEMU
test-full: test qemu-test
    @echo "All tests passed!"

# Run multi-node test (QEMU + native)
test-multi-node:
    ./scripts/run-multi-node-test.sh

# Setup QEMU network bridge (requires sudo)
setup-network:
    sudo ./scripts/setup-qemu-network.sh

# Teardown QEMU network bridge (requires sudo)
teardown-network:
    sudo ./scripts/teardown-qemu-network.sh

# Build with zenoh transport feature (requires zenoh-pico)
build-zenoh:
    cargo build -p nano-ros-transport --features zenoh,std

# Check zenoh transport compiles
check-zenoh:
    cargo check -p nano-ros-transport --features zenoh,std

# Build zenoh-pico library (for native testing)
build-zenoh-pico:
    @echo "Building zenoh-pico..."
    cd crates/zenoh-pico-sys/zenoh-pico && mkdir -p build && cd build && cmake .. -DBUILD_SHARED_LIBS=OFF && make
    @echo "zenoh-pico built at: crates/zenoh-pico-sys/zenoh-pico/build"

# =============================================================================
# Embedded Examples (STM32F4)
# =============================================================================

# Build RTIC example for STM32F4 (thumbv7em-none-eabihf)
build-rtic:
    @echo "Building RTIC example for STM32F429..."
    cd examples/rtic-stm32f4 && cargo build --release
    @echo "Binary: examples/rtic-stm32f4/target/thumbv7em-none-eabihf/release/rtic-stm32f4-example"

# Build Embassy example for STM32F4 (thumbv7em-none-eabihf)
build-embassy:
    @echo "Building Embassy example for STM32F429..."
    cd examples/embassy-stm32f4 && cargo build --release
    @echo "Binary: examples/embassy-stm32f4/target/thumbv7em-none-eabihf/release/embassy-stm32f4-example"

# Build polling example for STM32F4 (thumbv7em-none-eabihf)
build-polling:
    @echo "Building polling example for STM32F429..."
    cd examples/polling-stm32f4 && cargo build --release
    @echo "Binary: examples/polling-stm32f4/target/thumbv7em-none-eabihf/release/polling-stm32f4-example"

# Build all embedded examples
build-embedded-examples: build-rtic build-embassy build-polling
    @echo "All embedded examples built successfully!"

# Check all embedded examples compile (without full build)
check-embedded-examples:
    @echo "Checking RTIC example..."
    cd examples/rtic-stm32f4 && cargo check --release
    @echo "Checking Embassy example..."
    cd examples/embassy-stm32f4 && cargo check --release
    @echo "Checking polling example..."
    cd examples/polling-stm32f4 && cargo check --release
    @echo "All embedded examples check passed!"

# Clean embedded example build artifacts
clean-embedded-examples:
    rm -rf examples/rtic-stm32f4/target
    rm -rf examples/embassy-stm32f4/target
    rm -rf examples/polling-stm32f4/target
    @echo "Embedded example build artifacts cleaned"

# Show embedded example binary sizes
size-embedded-examples: build-embedded-examples
    @echo ""
    @echo "Binary sizes (release, stripped):"
    @echo "================================="
    @size examples/rtic-stm32f4/target/thumbv7em-none-eabihf/release/rtic-stm32f4-example 2>/dev/null || echo "RTIC: run 'just build-rtic' first"
    @size examples/embassy-stm32f4/target/thumbv7em-none-eabihf/release/embassy-stm32f4-example 2>/dev/null || echo "Embassy: run 'just build-embassy' first"
    @size examples/polling-stm32f4/target/thumbv7em-none-eabihf/release/polling-stm32f4-example 2>/dev/null || echo "Polling: run 'just build-polling' first"

# Flash RTIC example to STM32F4 (requires probe-rs)
flash-rtic:
    cd examples/rtic-stm32f4 && cargo run --release

# Flash Embassy example to STM32F4 (requires probe-rs)
flash-embassy:
    cd examples/embassy-stm32f4 && cargo run --release

# Flash polling example to STM32F4 (requires probe-rs)
flash-polling:
    cd examples/polling-stm32f4 && cargo run --release

# =============================================================================
# Zephyr Examples
# =============================================================================

# Show Zephyr example build instructions
zephyr-help:
    @echo "Zephyr Examples"
    @echo "==============="
    @echo ""
    @echo "The Zephyr examples require the Zephyr SDK and west build tool."
    @echo "See: examples/zephyr-talker/BUILDING.md for full instructions."
    @echo ""
    @echo "Quick start (assuming Zephyr is installed):"
    @echo "  west build -b qemu_cortex_m3 examples/zephyr-talker"
    @echo "  west build -t run"
