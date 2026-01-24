# Common clippy lints for real-time safety
CLIPPY_LINTS := "-D warnings -D clippy::infinite_iter -D clippy::while_immutable_condition -D clippy::never_loop -D clippy::empty_loop -D clippy::unconditional_recursion -W clippy::large_stack_arrays -W clippy::large_types_passed_by_value"

default:
    @just --list

# =============================================================================
# Entry Points
# =============================================================================

# Build everything: workspace (native + embedded) and all examples
build: build-workspace build-workspace-embedded build-examples
    @echo "All builds completed!"

# Format everything: workspace and all examples
format: format-workspace format-examples
    @echo "All formatting completed!"

# Check everything: formatting, clippy (native + embedded + features), and all examples
check: check-workspace check-workspace-embedded check-workspace-features check-examples
    @echo "All checks passed!"

# Test everything: workspace tests, Miri, QEMU, Rust integration, and shell integration tests
test: test-workspace test-miri test-qemu test-rust test-integration
    @echo "All tests passed!"

# Run all quality checks (check + test)
quality: check test

# =============================================================================
# Workspace
# =============================================================================

# Build workspace (no_std, native)
build-workspace:
    cargo build --workspace --no-default-features

# Build workspace for embedded target (Cortex-M4F)
# Excludes zenoh-pico-shim-sys which requires native system headers for CMake build
build-workspace-embedded:
    cargo build --workspace --no-default-features --target thumbv7em-none-eabihf \
        --exclude zenoh-pico-shim-sys

# Format workspace code
format-workspace:
    cargo +nightly fmt

# Check workspace: formatting and clippy (no_std, native)
check-workspace:
    cargo +nightly fmt --check
    cargo clippy --workspace --no-default-features -- {{CLIPPY_LINTS}}

# Check workspace for embedded target (Cortex-M4F)
# Excludes zenoh-pico-shim-sys which requires native system headers for CMake build
check-workspace-embedded:
    @echo "Checking workspace for embedded target..."
    cargo clippy --workspace --no-default-features --target thumbv7em-none-eabihf \
        --exclude zenoh-pico-shim-sys -- {{CLIPPY_LINTS}}

# Check workspace with various feature combinations
check-workspace-features:
    @echo "Checking feature combinations..."
    @echo "  - transport: rtic + sync-critical-section"
    cargo clippy -p nano-ros-transport --no-default-features --features "rtic,sync-critical-section" --target thumbv7em-none-eabihf -- {{CLIPPY_LINTS}}
    @echo "  - node: rtic"
    cargo clippy -p nano-ros-node --no-default-features --features "rtic" --target thumbv7em-none-eabihf -- {{CLIPPY_LINTS}}
    @echo "  - zenoh transport (std)"
    cargo clippy -p nano-ros-transport --features "zenoh,std" -- {{CLIPPY_LINTS}}
    @echo "All feature checks passed!"

# Run workspace tests (requires std)
test-workspace:
    cargo nextest run --workspace --no-fail-fast

# Run Miri to detect undefined behavior
test-miri:
    @echo "Running Miri on safe crates..."
    cargo +nightly miri test -p nano-ros-serdes
    cargo +nightly miri test -p nano-ros-core
    @echo "Miri checks passed!"

# =============================================================================
# Examples
# =============================================================================

# Build all examples
build-examples: build-examples-native build-examples-embedded build-examples-qemu
    @echo "All examples built!"

# Format all examples
format-examples: format-examples-native format-examples-embedded format-examples-qemu
    @echo "All examples formatted!"

# Check all examples
check-examples: check-examples-native check-examples-embedded check-examples-qemu
    @echo "All examples check passed!"

# =============================================================================
# Examples - Native
# =============================================================================

# Build native examples
build-examples-native:
    @echo "Building native examples..."
    cd examples/native-talker && cargo build
    cd examples/native-listener && cargo build
    cd examples/native-service-server && cargo build
    cd examples/native-service-client && cargo build
    cd examples/native-action-server && cargo build
    cd examples/native-action-client && cargo build

# Format native examples
format-examples-native:
    @echo "Formatting native examples..."
    cd examples/native-talker && cargo +nightly fmt
    cd examples/native-listener && cargo +nightly fmt
    cd examples/native-service-server && cargo +nightly fmt
    cd examples/native-service-client && cargo +nightly fmt
    cd examples/native-action-server && cargo +nightly fmt
    cd examples/native-action-client && cargo +nightly fmt

# Check native examples
check-examples-native:
    @echo "Checking native examples..."
    cd examples/native-talker && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}
    cd examples/native-listener && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}
    cd examples/native-service-server && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}
    cd examples/native-service-client && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}
    cd examples/native-action-server && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}
    cd examples/native-action-client && cargo +nightly fmt --check && cargo clippy -- {{CLIPPY_LINTS}}

# =============================================================================
# Examples - Embedded (STM32F4)
# =============================================================================

# Build embedded examples
build-examples-embedded:
    @echo "Building embedded examples..."
    cd examples/rtic-stm32f4 && cargo build --release
    cd examples/embassy-stm32f4 && cargo build --release
    cd examples/polling-stm32f4 && cargo build --release
    cd examples/smoltcp-test && cargo build --release

# Format embedded examples
format-examples-embedded:
    @echo "Formatting embedded examples..."
    cd examples/rtic-stm32f4 && cargo +nightly fmt
    cd examples/embassy-stm32f4 && cargo +nightly fmt
    cd examples/polling-stm32f4 && cargo +nightly fmt
    cd examples/smoltcp-test && cargo +nightly fmt

# Check embedded examples
check-examples-embedded:
    @echo "Checking embedded examples..."
    cd examples/rtic-stm32f4 && cargo +nightly fmt --check && cargo clippy --release -- {{CLIPPY_LINTS}}
    cd examples/embassy-stm32f4 && cargo +nightly fmt --check && cargo clippy --release -- {{CLIPPY_LINTS}}
    cd examples/polling-stm32f4 && cargo +nightly fmt --check && cargo clippy --release -- {{CLIPPY_LINTS}}
    cd examples/smoltcp-test && cargo +nightly fmt --check && cargo clippy --release -- {{CLIPPY_LINTS}}

# Show embedded example binary sizes
size-examples-embedded: build-examples-embedded
    @echo ""
    @echo "Binary sizes (release):"
    @echo "======================="
    @size examples/rtic-stm32f4/target/thumbv7em-none-eabihf/release/rtic-stm32f4-example 2>/dev/null || echo "RTIC: build failed"
    @size examples/embassy-stm32f4/target/thumbv7em-none-eabihf/release/embassy-stm32f4-example 2>/dev/null || echo "Embassy: build failed"
    @size examples/polling-stm32f4/target/thumbv7em-none-eabihf/release/polling-stm32f4-example 2>/dev/null || echo "Polling: build failed"
    @size examples/smoltcp-test/target/thumbv7em-none-eabihf/release/smoltcp-test 2>/dev/null || echo "smoltcp-test: build failed"

# Clean embedded example build artifacts
clean-examples-embedded:
    rm -rf examples/rtic-stm32f4/target
    rm -rf examples/embassy-stm32f4/target
    rm -rf examples/polling-stm32f4/target
    rm -rf examples/smoltcp-test/target
    @echo "Embedded example build artifacts cleaned"

# =============================================================================
# Examples - QEMU (Cortex-M3)
# =============================================================================

# Build QEMU test
build-examples-qemu:
    @echo "Building QEMU test..."
    cd examples/qemu-test && cargo build --release

# Format QEMU test
format-examples-qemu:
    @echo "Formatting QEMU test..."
    cd examples/qemu-test && cargo +nightly fmt

# Check QEMU test
check-examples-qemu:
    @echo "Checking QEMU test..."
    cd examples/qemu-test && cargo +nightly fmt --check && cargo clippy --release -- {{CLIPPY_LINTS}}

# Run QEMU test
test-qemu: build-examples-qemu
    qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -semihosting-config enable=on,target=native \
        -kernel examples/qemu-test/target/thumbv7m-none-eabi/release/qemu-test

# Check if QEMU is installed
check-qemu:
    @which qemu-system-arm > /dev/null || (echo "Error: qemu-system-arm not found. Install with: sudo apt install qemu-system-arm" && exit 1)
    @echo "QEMU ARM is installed"

# Run multi-node test (QEMU + native)
test-multi-node:
    ./scripts/run-multi-node-test.sh

# =============================================================================
# Static Analysis
# =============================================================================

# Analyze stack usage (requires nightly)
analyze-stack:
    @echo "Analyzing stack usage for RTIC example..."
    cd examples/rtic-stm32f4 && \
        RUSTFLAGS="-Z emit-stack-sizes" cargo +nightly build --release 2>&1 | head -20
    @echo ""
    @echo "Note: For full call graph analysis, install cargo-call-stack:"
    @echo "  cargo +nightly install cargo-call-stack"
    @echo "  cd examples/rtic-stm32f4 && cargo +nightly call-stack --release"

# Run all static analysis checks (Miri UB detection)
static-analysis: test-miri
    @echo ""
    @echo "All static analysis checks passed!"

# =============================================================================
# Zenoh
# =============================================================================

# Build zenoh transport
build-zenoh:
    cargo build -p nano-ros-transport --features zenoh,std

# Check zenoh transport
check-zenoh:
    cargo clippy -p nano-ros-transport --features zenoh,std -- {{CLIPPY_LINTS}}

# Build zenoh-pico C library (standalone, for debugging)
build-zenoh-pico:
    @echo "Building zenoh-pico..."
    cd crates/zenoh-pico-shim-sys/zenoh-pico && mkdir -p build && cd build && cmake .. -DBUILD_SHARED_LIBS=OFF && make
    @echo "zenoh-pico built at: crates/zenoh-pico-shim-sys/zenoh-pico/build"

# Test zenoh-pico-shim (requires zenohd running)
test-zenoh-shim:
    @echo "Testing zenoh-pico-shim (requires: zenohd --listen tcp/127.0.0.1:7447)"
    cargo test -p zenoh-pico-shim --features "posix std" -- --test-threads=1

# =============================================================================
# Integration Tests (tests/)
# =============================================================================

# Run all integration tests (requires zenohd, optionally ROS 2)
test-integration:
    ./tests/run-all.sh

# Run quick integration tests
test-integration-quick:
    ./tests/run-all.sh --quick

# Run nano-ros ↔ nano-ros tests only
test-nano2nano:
    ./tests/run-all.sh nano2nano

# Run ROS 2 interop tests (requires ROS 2 + rmw_zenoh_cpp)
test-rmw-interop:
    ./tests/run-all.sh rmw-interop

# Run detailed RMW protocol tests
test-rmw-detailed:
    ./tests/run-all.sh rmw-detailed

# Run Zephyr QEMU tests (requires west workspace)
test-zephyr:
    ./tests/run-all.sh zephyr

# =============================================================================
# Rust Integration Tests (crates/nano-ros-tests)
# =============================================================================

# Run all Rust integration tests
test-rust:
    cargo test -p nano-ros-tests --tests -- --nocapture

# Run Rust emulator tests (QEMU Cortex-M3)
test-rust-emulator:
    cargo test -p nano-ros-tests --test emulator -- --nocapture

# Run Rust native pub/sub tests
test-rust-nano2nano:
    cargo test -p nano-ros-tests --test nano2nano -- --nocapture

# Run Rust platform detection tests
test-rust-platform:
    cargo test -p nano-ros-tests --test platform -- --nocapture

# Run Rust tests via wrapper script (with nice output)
test-rust-full:
    ./tests/rust-tests.sh

# =============================================================================
# Setup & Cleanup
# =============================================================================

# Install toolchains and tools
setup:
    rustup toolchain install stable
    rustup toolchain install nightly
    rustup component add rustfmt clippy rust-src
    rustup component add --toolchain nightly rustfmt miri rust-src
    rustup target add thumbv7em-none-eabihf
    rustup target add thumbv7m-none-eabi
    cargo install cargo-nextest --locked

# Setup QEMU network bridge (requires sudo)
setup-network:
    sudo ./scripts/setup-qemu-network.sh

# Teardown QEMU network bridge (requires sudo)
teardown-network:
    sudo ./scripts/teardown-qemu-network.sh

# Generate documentation
doc:
    cargo doc --no-deps --open

# Clean build artifacts
clean:
    cargo clean

# Show Zephyr build instructions
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
