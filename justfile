# Common clippy lints for real-time safety
CLIPPY_LINTS := "-D warnings -D clippy::infinite_iter -D clippy::while_immutable_condition -D clippy::never_loop -D clippy::empty_loop -D clippy::unconditional_recursion -W clippy::large_stack_arrays -W clippy::large_types_passed_by_value"

default:
    @just --list

# =============================================================================
# Entry Points
# =============================================================================

# Build everything: workspace (native + embedded), C++ bindings, and all examples
build: build-workspace build-workspace-embedded build-cpp build-examples build-examples-cpp
    @echo "All builds completed!"

# Format everything: workspace, C++, and all examples
format: format-workspace format-cpp format-examples format-examples-cpp
    @echo "All formatting completed!"

# Check everything: formatting, clippy (native + embedded + features), C++, and all examples
check: check-workspace check-workspace-embedded check-workspace-features check-cpp check-examples check-examples-cpp
    @echo "All checks passed!"

# Run quick tests: workspace unit tests only (no integration tests)
test-quick: test-workspace
    @echo "Quick tests passed!"

# Test everything: workspace tests, Miri, QEMU, Rust integration, and shell integration tests
test: test-workspace test-miri test-qemu test-rust test-integration
    @echo "All tests passed!"

# Run code quality checks (formatting + clippy + unit tests) - no integration tests
quality: check test-workspace test-miri
    @echo "Quality checks passed!"

# Run full CI suite (quality + all integration tests)
ci: check test
    @echo "Full CI suite passed!"

# =============================================================================
# Workspace
# =============================================================================

# Build workspace (no_std, native)
build-workspace:
    cargo build --workspace --no-default-features

# Build workspace for embedded target (Cortex-M4F)
# Excludes zenoh-pico-shim-sys which requires native system headers for CMake build
# Excludes nano-ros-tests which requires std (test framework dependencies)
build-workspace-embedded:
    cargo build --workspace --no-default-features --target thumbv7em-none-eabihf \
        --exclude zenoh-pico-shim-sys \
        --exclude nano-ros-tests

# Format workspace code
format-workspace:
    cargo +nightly fmt

# Check workspace: formatting and clippy (no_std, native)
check-workspace:
    cargo +nightly fmt --check
    cargo clippy --workspace --no-default-features -- {{CLIPPY_LINTS}}

# Check workspace for embedded target (Cortex-M4F)
# Excludes zenoh-pico-shim-sys which requires native system headers for CMake build
# Excludes nano-ros-tests which requires std (test framework dependencies)
check-workspace-embedded:
    @echo "Checking workspace for embedded target..."
    cargo clippy --workspace --no-default-features --target thumbv7em-none-eabihf \
        --exclude zenoh-pico-shim-sys \
        --exclude nano-ros-tests -- {{CLIPPY_LINTS}}

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
# Examples - Zephyr (native_sim)
# =============================================================================

# Zephyr workspace path (symlink or sibling directory)
ZEPHYR_WORKSPACE := if path_exists("zephyr-workspace") == "true" { "zephyr-workspace" } else { "../nano-ros-workspace" }

# Build Zephyr examples (talker and listener to separate directories)
build-zephyr:
    #!/usr/bin/env bash
    set -e
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    if [ ! -d "$WORKSPACE/zephyr" ]; then
        echo "Error: Zephyr workspace not found at $WORKSPACE"
        echo "Run: ./scripts/zephyr/setup.sh"
        exit 1
    fi
    echo "Building Zephyr examples in $WORKSPACE..."
    cd "$WORKSPACE"
    echo "  Building zephyr-talker -> build-talker/"
    west build -b native_sim/native/64 -d build-talker -p auto nano-ros/examples/zephyr-talker
    echo "  Building zephyr-listener -> build-listener/"
    west build -b native_sim/native/64 -d build-listener -p auto nano-ros/examples/zephyr-listener
    echo "Zephyr examples built successfully!"

# Clean Zephyr build directories
clean-zephyr:
    #!/usr/bin/env bash
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    rm -rf "$WORKSPACE/build-talker" "$WORKSPACE/build-listener"
    echo "Zephyr build directories cleaned"

# Force rebuild Zephyr examples
rebuild-zephyr: clean-zephyr build-zephyr

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
# C++ Bindings
# =============================================================================

# Build C++ bindings (nano-ros-cpp)
build-cpp:
    @echo "Building C++ bindings..."
    cd crates/nano-ros-cpp && cmake -B build && cmake --build build

# Build C++ bindings (release)
build-cpp-release:
    @echo "Building C++ bindings (release)..."
    cd crates/nano-ros-cpp && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

# Format C++ code
format-cpp:
    @echo "Formatting C++ code..."
    @which clang-format > /dev/null || (echo "Error: clang-format not found. Install with: sudo apt install clang-format" && exit 1)
    find crates/nano-ros-cpp/cpp crates/nano-ros-cpp/include/nano_ros \
        -name '*.cpp' -o -name '*.hpp' -o -name '*.h' | \
        xargs clang-format -i --style=file:crates/nano-ros-cpp/.clang-format
    @echo "C++ code formatted"

# Check C++ formatting and run clang-tidy lints
check-cpp: _check-cpp-format _check-cpp-tidy
    @echo "C++ checks passed"

# Check C++ formatting only (does not modify files)
# Note: Excludes generated message headers (std_msgs, builtin_interfaces) which are auto-generated
_check-cpp-format:
    @echo "Checking C++ formatting..."
    @which clang-format > /dev/null || (echo "Error: clang-format not found. Install with: sudo apt install clang-format" && exit 1)
    find crates/nano-ros-cpp/cpp crates/nano-ros-cpp/include/nano_ros \
        -name '*.cpp' -o -name '*.hpp' -o -name '*.h' | \
        xargs clang-format --dry-run --Werror --style=file:crates/nano-ros-cpp/.clang-format
    @echo "C++ formatting check passed"

# Run clang-tidy on C++ code (requires build for compile_commands.json)
_check-cpp-tidy:
    @echo "Running clang-tidy..."
    @which clang-tidy > /dev/null || (echo "Error: clang-tidy not found. Install with: sudo apt install clang-tidy" && exit 1)
    @test -f crates/nano-ros-cpp/build/compile_commands.json || (echo "Error: compile_commands.json not found. Run 'just build-cpp' first." && exit 1)
    cd crates/nano-ros-cpp && clang-tidy -p build cpp/*.cpp
    @echo "clang-tidy check passed"

# Clean C++ bindings build
clean-cpp:
    rm -rf crates/nano-ros-cpp/build
    @echo "C++ bindings build cleaned"

# =============================================================================
# Examples - C++
# =============================================================================

# Build C++ examples
build-examples-cpp: build-cpp
    @echo "Building C++ examples..."
    cd examples/cpp-talker && cmake -B build && cmake --build build
    cd examples/cpp-listener && cmake -B build && cmake --build build
    cd examples/cpp-custom-msg && cmake -B build && cmake --build build

# Format C++ examples
format-examples-cpp:
    @echo "Formatting C++ examples..."
    @which clang-format > /dev/null || (echo "Error: clang-format not found." && exit 1)
    find examples/cpp-talker/src examples/cpp-listener/src examples/cpp-custom-msg/src -name '*.cpp' | \
        xargs clang-format -i --style=file:crates/nano-ros-cpp/.clang-format

# Check C++ examples
check-examples-cpp:
    @echo "Checking C++ examples..."
    @which clang-format > /dev/null || (echo "Error: clang-format not found." && exit 1)
    find examples/cpp-talker/src examples/cpp-listener/src examples/cpp-custom-msg/src -name '*.cpp' | \
        xargs clang-format --dry-run --Werror --style=file:crates/nano-ros-cpp/.clang-format

# Clean C++ examples build
clean-examples-cpp:
    rm -rf examples/cpp-talker/build examples/cpp-listener/build examples/cpp-custom-msg/build
    @echo "C++ examples build cleaned"

# Run C++ talker (requires zenohd)
run-cpp-talker:
    @echo "Running C++ talker (requires zenohd)..."
    examples/cpp-talker/build/cpp_talker

# Run C++ listener (requires zenohd)
run-cpp-listener:
    @echo "Running C++ listener (requires zenohd)..."
    examples/cpp-listener/build/cpp_listener

# Run C++ custom message example (requires zenohd)
run-cpp-custom-msg:
    @echo "Running C++ custom message example (requires zenohd)..."
    examples/cpp-custom-msg/build/cpp_custom_msg

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
# Integration Tests
# =============================================================================

# Run all integration tests (Rust-based, requires zenohd)
test-integration:
    cargo test -p nano-ros-tests --tests -- --nocapture

# Run Zephyr native_sim tests (requires west workspace + TAP network)
test-zephyr:
    ./tests/zephyr/run.sh

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

# Run Rust RMW interop tests (requires ROS 2 + rmw_zenoh_cpp)
test-rust-rmw-interop:
    cargo test -p nano-ros-tests --test rmw_interop -- --nocapture

# Run Rust Zephyr tests (requires west workspace + bridge network)
# Use test-rust-zephyr-full to force rebuild before testing
test-rust-zephyr:
    cargo test -p nano-ros-tests --test zephyr -- --nocapture

# Run Rust Zephyr tests with rebuild
test-rust-zephyr-full: build-zephyr
    cargo test -p nano-ros-tests --test zephyr -- --nocapture

# Run Rust tests via wrapper script (with nice output)
test-rust-full:
    ./tests/rust-tests.sh

# =============================================================================
# Message Bindings
# =============================================================================

# Install cargo-nano-ros (requires ROS 2 environment)
install-cargo-nano-ros:
    @echo "Installing cargo-nano-ros..."
    cargo install --path colcon-nano-ros/packages/cargo-nano-ros --locked

# Regenerate bindings in all examples (requires ROS 2 environment + cargo-nano-ros)
generate-bindings:
    @echo "Regenerating bindings in all examples..."
    @echo "Note: Requires ROS 2 environment sourced and cargo-nano-ros installed"
    cd examples/native-talker && cargo nano-ros generate
    cd examples/native-listener && cargo nano-ros generate
    cd examples/native-service-server && cargo nano-ros generate
    cd examples/native-service-client && cargo nano-ros generate
    cd examples/native-action-server && cargo nano-ros generate
    cd examples/native-action-client && cargo nano-ros generate
    cd examples/qemu-test && cargo nano-ros generate
    cd examples/zephyr-talker && cargo nano-ros generate
    cd examples/zephyr-listener && cargo nano-ros generate
    @echo "All bindings regenerated!"

# =============================================================================
# Setup & Cleanup
# =============================================================================

# Install toolchains and tools
setup:
    @echo "=== Installing Rust toolchains ==="
    rustup toolchain install stable
    rustup toolchain install nightly
    rustup component add rustfmt clippy rust-src
    rustup component add --toolchain nightly rustfmt miri rust-src
    rustup target add thumbv7em-none-eabihf
    rustup target add thumbv7m-none-eabi
    @echo ""
    @echo "=== Installing cargo tools ==="
    cargo install cargo-nextest --locked
    cargo install --path colcon-nano-ros/packages/cargo-nano-ros --locked
    @echo ""
    @echo "=== Checking system dependencies ==="
    @which arm-none-eabi-gcc > /dev/null 2>&1 || (echo "WARNING: arm-none-eabi-gcc not found." && echo "For embedded development, install with: sudo apt install gcc-arm-none-eabi" && echo "")
    @which qemu-system-arm > /dev/null 2>&1 || (echo "WARNING: qemu-system-arm not found." && echo "For QEMU testing, install with: sudo apt install qemu-system-arm" && echo "")
    @which cmake > /dev/null 2>&1 || (echo "WARNING: cmake not found." && echo "For C++ bindings, install with: sudo apt install cmake" && echo "")
    @which clang-format > /dev/null 2>&1 || (echo "WARNING: clang-format not found." && echo "For C++ formatting, install with: sudo apt install clang-format" && echo "")
    @which clang-tidy > /dev/null 2>&1 || (echo "WARNING: clang-tidy not found." && echo "For C++ linting, install with: sudo apt install clang-tidy" && echo "")
    @echo "Setup complete!"

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
clean: clean-cpp
    cargo clean

# Show Zephyr build instructions
zephyr-help:
    @echo "Zephyr Examples"
    @echo "==============="
    @echo ""
    @echo "Prerequisites:"
    @echo "  1. Set up Zephyr workspace: ./scripts/zephyr/setup.sh"
    @echo "  2. Set up bridge network:   sudo ./scripts/zephyr/setup-network.sh"
    @echo ""
    @echo "Build examples:"
    @echo "  just build-zephyr       # Build talker and listener"
    @echo "  just rebuild-zephyr     # Clean and rebuild"
    @echo "  just clean-zephyr       # Remove build directories"
    @echo ""
    @echo "Run tests:"
    @echo "  just test-rust-zephyr      # Run tests (uses existing binaries)"
    @echo "  just test-rust-zephyr-full # Rebuild and run tests"
    @echo ""
    @echo "Manual build (from Zephyr workspace):"
    @echo "  west build -b native_sim/native/64 -d build-talker nano-ros/examples/zephyr-talker"
    @echo "  west build -b native_sim/native/64 -d build-listener nano-ros/examples/zephyr-listener"
