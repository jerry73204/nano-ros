default:
    @just --list

# Install toolchains and tools for development
setup:
    rustup toolchain install stable
    rustup toolchain install nightly
    rustup component add rustfmt clippy rust-src
    rustup component add --toolchain nightly rustfmt miri rust-src
    rustup target add thumbv7em-none-eabihf
    cargo install cargo-nextest --locked

# Build with no_std (default)
build:
    cargo build --no-default-features

# Build for embedded target
build-embedded:
    cargo build --no-default-features --target thumbv7em-none-eabihf

format:
    cargo +nightly fmt

# Check with no_std (default)
check:
    cargo +nightly fmt --check
    cargo clippy --no-default-features -- -D warnings

# Test (requires std)
test:
    cargo nextest run --all-targets --no-fail-fast

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
